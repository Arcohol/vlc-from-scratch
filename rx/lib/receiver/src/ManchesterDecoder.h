#ifndef RECEIVER_MANCHESTER_DECODER_H
#define RECEIVER_MANCHESTER_DECODER_H

#include <stddef.h>
#include <stdint.h>

namespace receiver {

struct DecoderConfig {
  DecoderConfig();

  uint32_t half_bit_us;
  uint32_t sample_rate_hz;
  uint8_t preamble_edges_required;
  uint8_t threshold_shift;
};

struct Message {
  uint8_t length;
  uint8_t payload[255];
  uint16_t crc;
};

uint16_t crc16CcittFalse(const uint8_t *data, size_t length);
uint16_t crc16CcittFalseUpdate(uint16_t crc, uint8_t byte);

class ManchesterDecoder {
public:
  explicit ManchesterDecoder(const DecoderConfig &config = DecoderConfig());

  void reset();
  bool pushSample(uint16_t sample, Message *out);

  uint16_t threshold() const { return threshold_; }
  uint8_t halfBitSamples() const { return half_bit_samples_; }
  uint8_t fullBitSamples() const { return full_bit_samples_; }

private:
  enum class SyncState : uint8_t {
    SearchingPreamble,
    DecodingBits,
  };

  enum class FrameState : uint8_t {
    SearchingSfd,
    ReadingLength,
    ReadingPayload,
    ReadingCrcHigh,
    ReadingCrcLow,
  };

  static const uint16_t kSfd = 0xD5;
  static const uint16_t kRingSize = 128;
  static const uint16_t kRingMask = kRingSize - 1;

  void resetSync();
  void resetFrame();
  void updateThreshold(uint16_t sample);
  bool quantizedAt(uint32_t sample_index, bool *level) const;
  bool transitionAt(uint32_t edge_index) const;
  void handleTransition(uint32_t edge_index);
  bool tryDecodeScheduledBit(Message *out);
  bool decodeBitAtEdge(uint32_t edge_index, uint8_t *bit);
  bool consumeBit(uint8_t bit, Message *out);
  bool consumeByte(uint8_t byte, Message *out);
  void failAndSearchAgain();

  DecoderConfig config_;
  uint8_t half_bit_samples_;
  uint8_t full_bit_samples_;
  uint8_t center_offset_samples_;
  uint8_t edge_tolerance_samples_;

  bool ring_[kRingSize];
  uint32_t sample_index_;

  bool threshold_initialized_;
  int32_t threshold_q8_;
  uint16_t threshold_;

  bool last_level_;
  bool last_level_valid_;
  bool last_edge_valid_;
  uint32_t last_edge_index_;
  uint8_t preamble_edge_count_;

  SyncState sync_state_;
  uint32_t next_edge_index_;

  FrameState frame_state_;
  uint8_t bit_count_;
  uint8_t current_byte_;
  uint16_t rolling_byte_;
  uint16_t running_crc_;
  uint8_t payload_index_;
  Message working_message_;
};

} // namespace receiver

#endif // RECEIVER_MANCHESTER_DECODER_H
