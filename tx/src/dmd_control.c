#include "dmd_control.h"

#include <errno.h>
#include <string.h>

#include <nrfx_clock.h>
#include <nrfx_spim.h>
#include <nrfx_spis.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define DMD_WAVEFORM_BYTES 14
#define DMD_SPIM_FREQUENCY_MHZ 16

#ifndef DMD_DRC_B_CLK_PIN
#define DMD_DRC_B_CLK_PIN 8
#endif

#ifndef DMD_DRC_B_DATA_PIN
#define DMD_DRC_B_DATA_PIN 12
#endif

#ifndef DMD_DRC_B_LOADB_PIN
#define DMD_DRC_B_LOADB_PIN 6
#endif

#ifndef DMD_DRC_S_CLK_PIN
#define DMD_DRC_S_CLK_PIN 25
#endif

#ifndef DMD_DRC_S_CSN_PIN
#define DMD_DRC_S_CSN_PIN 26
#endif

#ifndef DMD_DRC_S_DATA_PIN
#define DMD_DRC_S_DATA_PIN 9
#endif

#ifndef DMD_POWER_RAIL_SETTLE_MS
#define DMD_POWER_RAIL_SETTLE_MS 5000
#endif

#ifndef DMD_BOOT_SETTLE_MS
#define DMD_BOOT_SETTLE_MS 5000
#endif

static nrfx_spim_t dmd_spim = NRFX_SPIM_INSTANCE(NRF_SPIM_INST_GET(4));
static nrfx_spis_t dmd_spis = NRFX_SPIS_INSTANCE(NRF_SPIS_INST_GET(1));

static bool dmd_initialized;

static const uint8_t dmd_drc_b_0[DMD_WAVEFORM_BYTES] = {
    0xFF, 0xFF, 0xFF, 0xC6, 0x7F, 0xFF, 0xC0,
    0x7F, 0xFF, 0x05, 0xFF, 0xFF, 0xFF, 0xFF,
};

static const uint8_t dmd_drc_b_1[DMD_WAVEFORM_BYTES] = {
    0xFF, 0xFF, 0xFF, 0x86, 0x7F, 0xFF, 0x80,
    0x7F, 0xFE, 0x05, 0xFF, 0xFF, 0xFF, 0xFF,
};

static uint8_t dmd_drc_b_buffer[DMD_WAVEFORM_BYTES];

static uint8_t dmd_sac_b_buffer[DMD_WAVEFORM_BYTES] = {
    0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const struct gpio_dt_spec dmd_shutdowns[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(shut1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(shut2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(shut3), gpios),
};

static nrfx_spim_config_t dmd_make_spim_config(void) {
  nrfx_spim_config_t spim_config =
      NRFX_SPIM_DEFAULT_CONFIG(DMD_DRC_B_CLK_PIN, DMD_DRC_B_DATA_PIN,
                               NRF_SPIM_PIN_NOT_CONNECTED, DMD_DRC_B_LOADB_PIN);

  spim_config.frequency = NRFX_MHZ_TO_HZ(DMD_SPIM_FREQUENCY_MHZ);
  spim_config.mode = NRF_SPIM_MODE_2;
#if NRF_SPIM_HAS_HW_CSN
  spim_config.use_hw_ss = true;
  spim_config.ss_duration = 8;
#endif

  return spim_config;
}

static void shift_right(uint8_t *data, size_t len, uint8_t shift) {
  while (shift-- > 0) {
    for (size_t i = len - 1; i > 0; --i) {
      uint8_t carry = (data[i - 1] & 1U) ? 0x80 : 0x00;

      data[i] = carry | (data[i] >> 1);
    }

    data[0] >>= 1;
  }
}

static void dmd_stage_bit(bool bit) {
  memcpy(dmd_drc_b_buffer, bit ? dmd_drc_b_1 : dmd_drc_b_0,
         sizeof(dmd_drc_b_buffer));
}

static void dmd_spis_handler(nrfx_spis_event_t const *event, void *context) {
  ARG_UNUSED(context);

  if (event->evt_type != NRFX_SPIS_XFER_DONE) {
    return;
  }

  (void)nrfx_spis_buffers_set(&dmd_spis, dmd_sac_b_buffer,
                              sizeof(dmd_sac_b_buffer), NULL, 0);
}

static int dmd_control_power_up(void) {
  for (size_t i = 0; i < ARRAY_SIZE(dmd_shutdowns); ++i) {
    if (!gpio_is_ready_dt(&dmd_shutdowns[i])) {
      return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&dmd_shutdowns[i], GPIO_OUTPUT_ACTIVE);

    if (ret != 0) {
      return ret;
    }
  }

  k_msleep(DMD_POWER_RAIL_SETTLE_MS);

  for (size_t i = 0; i < ARRAY_SIZE(dmd_shutdowns); ++i) {
    int ret = gpio_pin_set_dt(&dmd_shutdowns[i], 0);

    if (ret != 0) {
      return ret;
    }
  }

  k_msleep(DMD_BOOT_SETTLE_MS);
  return 0;
}

int dmd_control_init(void) {
  if (dmd_initialized) {
    return 0;
  }

  int ret = dmd_control_power_up();

  if (ret != 0) {
    return ret;
  }

  nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS_INST_GET(1)), IRQ_PRIO_LOWEST,
              nrfx_spis_irq_handler, &dmd_spis, 0);

  shift_right(dmd_sac_b_buffer, sizeof(dmd_sac_b_buffer), 2);

  nrfx_spis_config_t spis_config =
      NRFX_SPIS_DEFAULT_CONFIG(DMD_DRC_S_CLK_PIN, NRF_SPIS_PIN_NOT_CONNECTED,
                               DMD_DRC_S_DATA_PIN, DMD_DRC_S_CSN_PIN);

  spis_config.miso_drive = NRF_GPIO_PIN_H0H1;
  spis_config.mode = NRF_SPIS_MODE_2;
  spis_config.bit_order = NRF_SPIS_BIT_ORDER_MSB_FIRST;

  int err = nrfx_spis_init(&dmd_spis, &spis_config, dmd_spis_handler, NULL);

  if (err != 0 && err != -EALREADY) {
    return err;
  }

  err = nrfx_spis_buffers_set(&dmd_spis, dmd_sac_b_buffer,
                              sizeof(dmd_sac_b_buffer), NULL, 0);

  if (err != 0) {
    return err;
  }

  nrfx_spim_config_t spim_config = dmd_make_spim_config();

  err = nrfx_spim_init(&dmd_spim, &spim_config, NULL, NULL);

  if (err != 0 && err != -EALREADY) {
    return err;
  }

  dmd_stage_bit(false);
  dmd_initialized = true;

  return 0;
}

int dmd_send_bit(bool bit) {
  if (!dmd_initialized) {
    int ret = dmd_control_init();

    if (ret != 0) {
      return ret;
    }
  }

  dmd_stage_bit(bit);

  nrfx_spim_xfer_desc_t xfer =
      NRFX_SPIM_XFER_TRX(dmd_drc_b_buffer, sizeof(dmd_drc_b_buffer), NULL, 0);

  int err = nrfx_spim_xfer(&dmd_spim, &xfer, 0);

  return err;
}
