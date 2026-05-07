#pragma once

#include <stdbool.h>

int dmd_control_init(void);
int dmd_send_bit(bool bit);

static inline int dmd_send_0(void) { return dmd_send_bit(false); }

static inline int dmd_send_1(void) { return dmd_send_bit(true); }
