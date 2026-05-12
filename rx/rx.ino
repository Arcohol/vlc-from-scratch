#include <Arduino.h>

#include "adc_sampler.h"
#include "vlc_rx.h"

const uint32_t HEARTBEAT_INTERVAL_MS = 5000U;

void setup() {
  SerialUSB.begin(VLC_RX_SERIAL_BAUD);

  uint32_t start_ms = millis();
  while (!SerialUSB && (millis() - start_ms) < 1500U) {
  }

  vlc_rx_init();

  if (!adc_sampler_begin()) {
    SerialUSB.println("adc init failed");
  }
}

void loop() {
  static uint32_t last_heartbeat_ms = 0;
  const uint32_t now_ms = millis();

  if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_ms = now_ms;
    SerialUSB.print("heartbeat uptime_ms=");
    SerialUSB.println(now_ms);
  }

  if (adc_sampler_take_overrun()) {
    vlc_rx_reset();
  }

  AdcBlock block;
  while (adc_sampler_pop_completed_block(&block)) {
    for (uint16_t i = 0; i < block.len; ++i) {
      vlc_rx_push_sample(block.samples[i]);
    }

    adc_sampler_release_block(&block);
  }

  RxMessage message;
  while (vlc_rx_pop_message(&message)) {
    SerialUSB.write(message.data, message.len);
    SerialUSB.write('\r');
    SerialUSB.write('\n');
  }
}
