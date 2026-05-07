#include <nrfx_timer.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "dmd_control.h"

#define ALT_HIGH_US 100U
#define ALT_LOW_US 200U
#define PATTERN_TIMER_INSTANCE 2
#define PATTERN_TIMER_IRQ_PRIORITY 1

static nrfx_timer_t pattern_timer =
    NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(PATTERN_TIMER_INSTANCE));

static uint32_t high_ticks;
static uint32_t low_ticks;
static uint32_t next_compare_ticks;
static bool next_bit;
static volatile int pattern_error;

static void pattern_timer_handler(nrf_timer_event_t event_type, void *context) {
  (void)context;

  if (event_type != NRF_TIMER_EVENT_COMPARE0) {
    return;
  }

  bool bit = next_bit;
  next_bit = !next_bit;
  next_compare_ticks += bit ? high_ticks : low_ticks;

  uint32_t now = nrfx_timer_capture(&pattern_timer, NRF_TIMER_CC_CHANNEL1);

  if ((int32_t)(now - next_compare_ticks) >= 0) {
    pattern_error = -EIO;
    nrfx_timer_disable(&pattern_timer);
    return;
  }

  nrfx_timer_compare(&pattern_timer, NRF_TIMER_CC_CHANNEL0, next_compare_ticks,
                     true);

  int ret = dmd_send_bit(bit);

  if (ret != 0) {
    pattern_error = ret;
    nrfx_timer_disable(&pattern_timer);
  }
}

static int run_timer_alternating_pattern(void) {
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(PATTERN_TIMER_INSTANCE)),
              PATTERN_TIMER_IRQ_PRIORITY, nrfx_timer_irq_handler,
              &pattern_timer, 0);

  uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(pattern_timer.p_reg);
  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);

  config.bit_width = NRF_TIMER_BIT_WIDTH_32;

  int ret = nrfx_timer_init(&pattern_timer, &config, pattern_timer_handler);

  if (ret != 0 && ret != -EALREADY) {
    return ret;
  }

  high_ticks = nrfx_timer_us_to_ticks(&pattern_timer, ALT_HIGH_US);
  low_ticks = nrfx_timer_us_to_ticks(&pattern_timer, ALT_LOW_US);
  next_compare_ticks = high_ticks;
  next_bit = false;
  pattern_error = 0;

  nrfx_timer_clear(&pattern_timer);
  nrfx_timer_compare(&pattern_timer, NRF_TIMER_CC_CHANNEL0, next_compare_ticks,
                     true);

  ret = dmd_send_1();

  if (ret != 0) {
    return ret;
  }

  nrfx_timer_enable(&pattern_timer);

  printk("Timer alternating %u us high / %u us low\n", ALT_HIGH_US, ALT_LOW_US);
  return 0;
}

int main(void) {
  printk("oh-my-dmd\n");

  int ret = dmd_control_init();

  if (ret != 0) {
    printk("DMD init failed: %d\n", ret);
    return ret;
  }

  ret = dmd_send_0();

  if (ret != 0) {
    printk("DMD idle state failed: %d\n", ret);
    return ret;
  }

  printk("DMD ready\n");

  ret = run_timer_alternating_pattern();

  if (ret != 0) {
    printk("Alternating pattern failed: %d\n", ret);
    return ret;
  }

  while (pattern_error == 0) {
    k_sleep(K_MSEC(1000));
  }

  printk("Alternating pattern failed: %d\n", pattern_error);
  return pattern_error;
}
