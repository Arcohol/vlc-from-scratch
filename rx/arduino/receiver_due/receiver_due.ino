#include <Arduino.h>
#include <ManchesterDecoder.h>

#include "adc_sampler.h"
#include "vlc_rx_config.h"

using receiver::DecoderConfig;
using receiver::DecoderStats;
using receiver::ManchesterDecoder;
using receiver::Message;

namespace {

constexpr uint32_t kHeartbeatIntervalMs = 5000U;
constexpr uint8_t kMaxAdcBlocksPerLoop = 4;
constexpr uint8_t kMaxMessagesPerLoop = 1;

DecoderConfig decoder_config;
ManchesterDecoder decoder(decoder_config);

void printPayload(const Message &message) {
  SerialUSB.write(message.payload, message.length);
  SerialUSB.write('\r');
  SerialUSB.write('\n');
}

void printHeartbeat(uint32_t now_ms) {
  const DecoderStats stats = decoder.stats();

  SerialUSB.print("heartbeat uptime_ms=");
  SerialUSB.print(now_ms);
  SerialUSB.print(" rx_msg=");
  SerialUSB.print(stats.messages);
  SerialUSB.print(" crc_fail=");
  SerialUSB.print(stats.crc_failures);
  SerialUSB.print(" weak=");
  SerialUSB.print(stats.weak_bits);
  SerialUSB.print(" lost=");
  SerialUSB.print(stats.lost_center_edges);
  SerialUSB.print(" sfd_timeout=");
  SerialUSB.print(stats.sfd_timeouts);
  SerialUSB.print(" qdrop=");
  SerialUSB.print(stats.queue_drops);
  SerialUSB.print(" len_err=");
  SerialUSB.print(stats.length_errors);
  SerialUSB.print(" adc_ovr=");
  SerialUSB.print(adc_sampler_get_overrun_count());
  SerialUSB.print(" swing=");
  SerialUSB.print(stats.signal_swing);
  SerialUSB.print(" contrast=");
  SerialUSB.println(stats.contrast);
}

void resetDecoderAfterSampleLoss() { decoder.resetStream(); }

} // namespace

void setup() {
  SerialUSB.begin(VLC_RX_SERIAL_BAUD);

  const uint32_t start_ms = millis();
  while (!SerialUSB && (millis() - start_ms) < 1500U) {
  }

  decoder_config.half_bit_us = VLC_RX_HALF_BIT_US;
  decoder_config.sample_rate_hz = VLC_RX_SAMPLE_RATE_HZ;
  decoder_config.preamble_edges_required = VLC_RX_PREAMBLE_MIN_EDGES;
  decoder = ManchesterDecoder(decoder_config);

  SerialUSB.println("receiver starting");

  if (!adc_sampler_begin()) {
    SerialUSB.println("adc init failed");
  } else {
    SerialUSB.print("receiver ready sample_rate_hz=");
    SerialUSB.print(VLC_RX_SAMPLE_RATE_HZ);
    SerialUSB.print(" block_samples=");
    SerialUSB.print(ADC_SAMPLER_BLOCK_SAMPLES);
    SerialUSB.print(" buffers=");
    SerialUSB.println(ADC_SAMPLER_BUFFER_COUNT);
  }
}

void loop() {
  static uint32_t last_heartbeat_ms = 0;

  if (adc_sampler_take_overrun()) {
    resetDecoderAfterSampleLoss();
  }

  AdcBlock block;
  uint8_t blocks_processed = 0;
  while (blocks_processed < kMaxAdcBlocksPerLoop &&
         adc_sampler_pop_completed_block(&block)) {
    decoder.pushSamples(block.samples, block.len);
    adc_sampler_release_block(&block);
    ++blocks_processed;
  }

  Message message;
  uint8_t messages_printed = 0;
  while (messages_printed < kMaxMessagesPerLoop &&
         decoder.popMessage(&message)) {
    printPayload(message);
    ++messages_printed;
  }

  // const uint32_t now_ms = millis();
  // if ((now_ms - last_heartbeat_ms) >= kHeartbeatIntervalMs) {
  //   last_heartbeat_ms = now_ms;
  //   printHeartbeat(now_ms);
  // }
}
