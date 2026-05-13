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

  if (adc_sampler_take_overrun()) {
    vlc_rx_reset();
  }

  if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_ms = now_ms;
    RxStats rx_stats;
    vlc_rx_get_stats(&rx_stats);

    SerialUSB.print("heartbeat uptime_ms=");
    SerialUSB.print(now_ms);
    SerialUSB.print(" rx_msg=");
    SerialUSB.print(rx_stats.messages);
    SerialUSB.print(" crc_fail=");
    SerialUSB.print(rx_stats.crc_failures);
    SerialUSB.print(" weak=");
    SerialUSB.print(rx_stats.weak_bits);
    SerialUSB.print(" lost=");
    SerialUSB.print(rx_stats.lost_center_edges);
    SerialUSB.print(" sfd_timeout=");
    SerialUSB.print(rx_stats.sfd_timeouts);
    SerialUSB.print(" qdrop=");
    SerialUSB.print(rx_stats.queue_drops);
    SerialUSB.print(" len_err=");
    SerialUSB.print(rx_stats.length_errors);
    SerialUSB.print(" adc_ovr=");
    SerialUSB.print(adc_sampler_get_overrun_count());
    SerialUSB.print(" swing=");
    SerialUSB.print(rx_stats.signal_swing);
    SerialUSB.print(" contrast=");
    SerialUSB.println(rx_stats.contrast);
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
