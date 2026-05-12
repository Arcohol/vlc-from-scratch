#include "vlc_tx.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nrfx_timer.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>

#include "dmd_control.h"

#ifndef VLC_TX_MAX_PAYLOAD_LEN
#define VLC_TX_MAX_PAYLOAD_LEN 255
#endif

#ifndef VLC_TX_QUEUE_DEPTH
#define VLC_TX_QUEUE_DEPTH 4
#endif

#ifndef VLC_TX_HALF_BIT_US
#define VLC_TX_HALF_BIT_US 100
#endif

#ifndef VLC_TX_TIMER_INSTANCE
#define VLC_TX_TIMER_INSTANCE 2
#endif

#ifndef VLC_TX_TIMER_IRQ_PRIORITY
#define VLC_TX_TIMER_IRQ_PRIORITY 1
#endif

#ifndef VLC_TX_STAGING_THREAD_PRIORITY
#define VLC_TX_STAGING_THREAD_PRIORITY 1
#endif

#ifndef VLC_TX_STAGING_THREAD_STACK_SIZE
#define VLC_TX_STAGING_THREAD_STACK_SIZE 2048
#endif

#ifndef VLC_TX_WAIT_POLL_MS
#define VLC_TX_WAIT_POLL_MS 10
#endif

#ifndef VLC_TX_REMEMBER_LAST_BIT
#define VLC_TX_REMEMBER_LAST_BIT 1
#endif

#define VLC_TX_PREAMBLE_BYTE 0xAA
#define VLC_TX_PREAMBLE_LEN 4
#define VLC_TX_SFD 0xD5
#define VLC_TX_CRC_INIT 0xFFFF
#define VLC_TX_CRC_POLY 0x1021
#define VLC_TX_FRAME_OVERHEAD (VLC_TX_PREAMBLE_LEN + 1 + 1 + 2)
#define VLC_TX_MAX_FRAME_LEN (VLC_TX_FRAME_OVERHEAD + VLC_TX_MAX_PAYLOAD_LEN)
#define VLC_TX_FRAME_BUFFER_COUNT 2

struct vlc_tx_payload {
  uint8_t len;
  uint8_t data[VLC_TX_MAX_PAYLOAD_LEN];
};

struct vlc_tx_frame {
  uint16_t len;
  uint8_t data[VLC_TX_MAX_FRAME_LEN];
};

K_MSGQ_DEFINE(vlc_tx_payload_queue, sizeof(struct vlc_tx_payload),
              VLC_TX_QUEUE_DEPTH, 4);
K_SEM_DEFINE(vlc_tx_free_frame_sem, VLC_TX_FRAME_BUFFER_COUNT,
             VLC_TX_FRAME_BUFFER_COUNT);
K_SEM_DEFINE(vlc_tx_pending_slot_sem, 1, 1);

static nrfx_timer_t vlc_tx_timer =
    NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(VLC_TX_TIMER_INSTANCE));

static struct vlc_tx_frame vlc_tx_frames[VLC_TX_FRAME_BUFFER_COUNT];
static bool vlc_tx_frame_busy[VLC_TX_FRAME_BUFFER_COUNT];

static volatile int vlc_tx_error;
static bool vlc_tx_initialized;

static volatile int vlc_tx_active_frame = -1;
static volatile int vlc_tx_pending_frame = -1;
static volatile uint32_t vlc_tx_active_chip_index;
static bool vlc_tx_last_bit;
static bool vlc_tx_last_bit_valid;

static uint32_t vlc_tx_half_bit_ticks;
static uint32_t vlc_tx_next_compare_ticks;

static uint16_t vlc_tx_crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= (uint16_t)byte << 8;

  for (uint8_t bit = 0; bit < 8; ++bit) {
    if ((crc & 0x8000U) != 0U) {
      crc = (uint16_t)((crc << 1) ^ VLC_TX_CRC_POLY);
    } else {
      crc = (uint16_t)(crc << 1);
    }
  }

  return crc;
}

static void vlc_tx_build_frame(struct vlc_tx_frame *frame,
                               const struct vlc_tx_payload *payload) {
  uint16_t offset = 0;

  for (uint8_t i = 0; i < VLC_TX_PREAMBLE_LEN; ++i) {
    frame->data[offset++] = VLC_TX_PREAMBLE_BYTE;
  }

  frame->data[offset++] = VLC_TX_SFD;
  frame->data[offset++] = payload->len;

  uint16_t crc = vlc_tx_crc16_update(VLC_TX_CRC_INIT, payload->len);

  for (uint8_t i = 0; i < payload->len; ++i) {
    frame->data[offset++] = payload->data[i];
    crc = vlc_tx_crc16_update(crc, payload->data[i]);
  }

  frame->data[offset++] = (uint8_t)(crc >> 8);
  frame->data[offset++] = (uint8_t)crc;
  frame->len = offset;
}

static bool vlc_tx_chip_for_index(const struct vlc_tx_frame *frame,
                                  uint32_t chip_index) {
  uint32_t bit_index = chip_index / 2U;
  uint8_t byte = frame->data[bit_index / 8U];
  uint8_t bit_offset = 7U - (uint8_t)(bit_index % 8U);
  bool raw_one = (byte & BIT(bit_offset)) != 0U;
  bool second_half = (chip_index & 1U) != 0U;

  return raw_one ? second_half : !second_half;
}

static void vlc_tx_promote_pending_frame(void) {
  if (vlc_tx_active_frame >= 0 || vlc_tx_pending_frame < 0) {
    return;
  }

  vlc_tx_active_frame = vlc_tx_pending_frame;
  vlc_tx_pending_frame = -1;
  vlc_tx_active_chip_index = 0;
  k_sem_give(&vlc_tx_pending_slot_sem);
}

static void vlc_tx_finish_active_frame(void) {
  int frame_index = vlc_tx_active_frame;

  if (frame_index < 0) {
    return;
  }

  vlc_tx_active_frame = -1;
  vlc_tx_active_chip_index = 0;
  vlc_tx_frame_busy[frame_index] = false;
  k_sem_give(&vlc_tx_free_frame_sem);
}

static bool vlc_tx_next_chip(void) {
  vlc_tx_promote_pending_frame();

  if (vlc_tx_active_frame < 0) {
    return true;
  }

  struct vlc_tx_frame *frame = &vlc_tx_frames[vlc_tx_active_frame];
  bool chip = vlc_tx_chip_for_index(frame, vlc_tx_active_chip_index);

  ++vlc_tx_active_chip_index;

  if (vlc_tx_active_chip_index >= ((uint32_t)frame->len * 8U * 2U)) {
    vlc_tx_finish_active_frame();
  }

  return chip;
}

static void vlc_tx_fail(int error) {
  if (error == 0) {
    error = -EIO;
  }

  if (vlc_tx_error == 0) {
    vlc_tx_error = error;
  }

  nrfx_timer_disable(&vlc_tx_timer);
}

static int vlc_tx_send_chip(bool chip) {
#if VLC_TX_REMEMBER_LAST_BIT
  if (vlc_tx_last_bit_valid && vlc_tx_last_bit == chip) {
    return dmd_control_ready() ? 0 : -EBUSY;
  }
#endif

  int ret = dmd_send_bit_async(chip);

  if (ret == 0) {
    vlc_tx_last_bit = chip;
    vlc_tx_last_bit_valid = true;
  }

  return ret;
}

static void vlc_tx_timer_handler(nrf_timer_event_t event_type, void *context) {
  ARG_UNUSED(context);

  if (event_type != NRF_TIMER_EVENT_COMPARE0) {
    return;
  }

  vlc_tx_next_compare_ticks += vlc_tx_half_bit_ticks;

  uint32_t now = nrfx_timer_capture(&vlc_tx_timer, NRF_TIMER_CC_CHANNEL1);

  if ((int32_t)(now - vlc_tx_next_compare_ticks) >= 0) {
    vlc_tx_fail(-EIO);
    return;
  }

  nrfx_timer_compare(&vlc_tx_timer, NRF_TIMER_CC_CHANNEL0,
                     vlc_tx_next_compare_ticks, true);

  int ret = vlc_tx_send_chip(vlc_tx_next_chip());

  if (ret != 0) {
    vlc_tx_fail(ret);
  }
}

static int vlc_tx_reserve_frame(void) {
  int frame_index = -1;
  unsigned int key = irq_lock();

  for (int i = 0; i < VLC_TX_FRAME_BUFFER_COUNT; ++i) {
    if (!vlc_tx_frame_busy[i]) {
      vlc_tx_frame_busy[i] = true;
      frame_index = i;
      break;
    }
  }

  irq_unlock(key);
  return frame_index;
}

static void vlc_tx_release_frame(int frame_index) {
  unsigned int key = irq_lock();

  vlc_tx_frame_busy[frame_index] = false;

  irq_unlock(key);
  k_sem_give(&vlc_tx_free_frame_sem);
}

static void vlc_tx_publish_frame(int frame_index) {
  unsigned int key = irq_lock();

  vlc_tx_pending_frame = frame_index;

  irq_unlock(key);
}

static int vlc_tx_take_sem_interruptible(struct k_sem *sem) {
  while (vlc_tx_error == 0) {
    int ret = k_sem_take(sem, K_MSEC(VLC_TX_WAIT_POLL_MS));

    if (ret == 0) {
      return 0;
    }

    if (ret != -EAGAIN) {
      return ret;
    }
  }

  return -EIO;
}

static k_timeout_t vlc_tx_wait_chunk(k_timepoint_t deadline) {
  k_timeout_t remaining = sys_timepoint_timeout(deadline);

  if (K_TIMEOUT_EQ(remaining, K_NO_WAIT) ||
      K_TIMEOUT_EQ(remaining, K_FOREVER)) {
    return remaining;
  }

  k_timeout_t poll = K_MSEC(VLC_TX_WAIT_POLL_MS);

  return remaining.ticks < poll.ticks ? remaining : poll;
}

static void vlc_tx_staging_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  while (true) {
    struct vlc_tx_payload payload;

    (void)k_msgq_get(&vlc_tx_payload_queue, &payload, K_FOREVER);

    if (vlc_tx_error != 0) {
      continue;
    }

    int ret = vlc_tx_take_sem_interruptible(&vlc_tx_pending_slot_sem);

    if (ret != 0) {
      continue;
    }

    ret = vlc_tx_take_sem_interruptible(&vlc_tx_free_frame_sem);

    if (ret != 0) {
      k_sem_give(&vlc_tx_pending_slot_sem);
      continue;
    }

    int frame_index = vlc_tx_reserve_frame();

    if (frame_index < 0) {
      k_sem_give(&vlc_tx_free_frame_sem);
      k_sem_give(&vlc_tx_pending_slot_sem);
      continue;
    }

    if (vlc_tx_error != 0) {
      vlc_tx_release_frame(frame_index);
      k_sem_give(&vlc_tx_pending_slot_sem);
      continue;
    }

    vlc_tx_build_frame(&vlc_tx_frames[frame_index], &payload);
    vlc_tx_publish_frame(frame_index);
  }
}

K_THREAD_DEFINE(vlc_tx_staging_thread_id, VLC_TX_STAGING_THREAD_STACK_SIZE,
                vlc_tx_staging_thread, NULL, NULL, NULL,
                VLC_TX_STAGING_THREAD_PRIORITY, 0, 0);

int vlc_tx_init(void) {
  if (vlc_tx_initialized) {
    return 0;
  }

  int ret = dmd_control_init();

  if (ret != 0) {
    return ret;
  }

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(VLC_TX_TIMER_INSTANCE)),
              VLC_TX_TIMER_IRQ_PRIORITY, nrfx_timer_irq_handler, &vlc_tx_timer,
              0);

  uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(vlc_tx_timer.p_reg);
  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);

  config.bit_width = NRF_TIMER_BIT_WIDTH_32;

  ret = nrfx_timer_init(&vlc_tx_timer, &config, vlc_tx_timer_handler);

  if (ret != 0 && ret != -EALREADY) {
    return ret;
  }

  vlc_tx_half_bit_ticks =
      nrfx_timer_us_to_ticks(&vlc_tx_timer, VLC_TX_HALF_BIT_US);
  vlc_tx_next_compare_ticks = vlc_tx_half_bit_ticks;
  vlc_tx_error = 0;
  vlc_tx_active_frame = -1;
  vlc_tx_pending_frame = -1;
  vlc_tx_active_chip_index = 0;
  vlc_tx_last_bit = false;
  vlc_tx_last_bit_valid = false;

  ret = vlc_tx_send_chip(true);

  if (ret != 0) {
    return ret;
  }

  nrfx_timer_clear(&vlc_tx_timer);
  nrfx_timer_compare(&vlc_tx_timer, NRF_TIMER_CC_CHANNEL0,
                     vlc_tx_next_compare_ticks, true);
  nrfx_timer_enable(&vlc_tx_timer);

  vlc_tx_initialized = true;
  return 0;
}

int vlc_tx_send(const uint8_t *buf, uint32_t len, k_timeout_t timeout) {
  if (len > VLC_TX_MAX_PAYLOAD_LEN) {
    return -EMSGSIZE;
  }

  if (buf == NULL && len > 0U) {
    return -EINVAL;
  }

  if (k_is_in_isr() && !K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
    return -EINVAL;
  }

  if (!vlc_tx_initialized || vlc_tx_error != 0) {
    return -EIO;
  }

  struct vlc_tx_payload payload = {0};

  payload.len = (uint8_t)len;

  if (len > 0U) {
    memcpy(payload.data, buf, len);
  }

  if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
    return k_msgq_put(&vlc_tx_payload_queue, &payload, K_NO_WAIT);
  }

  k_timepoint_t deadline = sys_timepoint_calc(timeout);

  while (true) {
    if (vlc_tx_error != 0) {
      return -EIO;
    }

    int ret = k_msgq_put(&vlc_tx_payload_queue, &payload, K_NO_WAIT);

    if (ret == 0) {
      return 0;
    }

    if (ret != -ENOMSG) {
      return ret;
    }

    k_timeout_t wait = vlc_tx_wait_chunk(deadline);

    if (K_TIMEOUT_EQ(wait, K_NO_WAIT)) {
      return -EAGAIN;
    }

    ret = k_msgq_put(&vlc_tx_payload_queue, &payload, wait);

    if (ret == 0) {
      return 0;
    }

    if (ret != -EAGAIN && ret != -ENOMSG) {
      return ret;
    }
  }
}

int vlc_tx_get_error(void) { return vlc_tx_error; }
