#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vlc_rx_config.h"

#ifndef ADC_SAMPLER_BLOCK_SAMPLES
#define ADC_SAMPLER_BLOCK_SAMPLES 128U
#endif

struct AdcBlock {
  const uint16_t *samples;
  uint16_t len;
  uint8_t index;
};

bool adc_sampler_begin();
void adc_sampler_stop();

bool adc_sampler_pop_completed_block(AdcBlock *block);
void adc_sampler_release_block(const AdcBlock *block);

bool adc_sampler_take_overrun();
