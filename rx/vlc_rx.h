#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vlc_rx_config.h"

struct RxMessage {
  uint8_t len;
  uint8_t data[VLC_RX_MAX_PAYLOAD_LEN];
};

struct RxStats {
  uint32_t messages;
  uint32_t crc_failures;
  uint32_t queue_drops;
  uint32_t sfd_timeouts;
  uint32_t weak_bits;
  uint32_t lost_center_edges;
  uint32_t length_errors;
  uint16_t signal_swing;
  uint16_t contrast;
};

void vlc_rx_init();
void vlc_rx_reset();
void vlc_rx_push_sample(uint16_t sample);

bool vlc_rx_has_message();
bool vlc_rx_pop_message(RxMessage *message);
void vlc_rx_get_stats(RxStats *stats);
