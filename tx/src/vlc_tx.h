#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>

int vlc_tx_init(void);
int vlc_tx_send(const uint8_t *buf, uint32_t len, k_timeout_t timeout);
int vlc_tx_get_error(void);
