#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vlc_rx_config.h"

struct RxMessage {
  uint8_t len;
  uint8_t data[VLC_RX_MAX_PAYLOAD_LEN];
};

void vlc_rx_init();
void vlc_rx_reset();
void vlc_rx_push_sample(uint16_t sample);

bool vlc_rx_has_message();
bool vlc_rx_pop_message(RxMessage *message);
