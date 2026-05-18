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

struct DecoderStats {
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
  void resetStream();
  void pushSample(uint16_t sample);
  bool pushSample(uint16_t sample, Message *out);
  void pushSamples(const uint16_t *samples, size_t count);
  bool hasMessage() const;
  bool popMessage(Message *out);

  DecoderStats stats() const;
  uint16_t threshold() const;
  uint8_t halfBitSamples() const { return half_bit_samples_; }
  uint8_t fullBitSamples() const { return full_bit_samples_; }

private:
  enum class LightState : uint8_t {
    Unknown,
    Low,
    High,
  };

  enum class State : uint8_t {
    SearchPreamble,
    FindSfd,
    ReadLength,
    ReadPayload,
    ReadCrcHigh,
    ReadCrcLow,
  };

  struct EdgeEvent {
    uint32_t sample_index;
    bool rising;
    uint16_t swing;
  };

  static const uint8_t kSfd = 0xD5;
  static const uint16_t kCrcInit = 0xFFFF;
  static const int32_t kQ8 = 256;
  static const uint16_t kMinSignalSwingCounts = 60;
  static const uint16_t kMinHysteresisCounts = 18;
  static const uint16_t kMinDeltaCounts = 30;
  static const uint8_t kContrastAdaptShift = 3;
  static const uint8_t kMaxLostCenterEdges = 5;
  static const uint8_t kSfdSearchMaxBits = 36;
  static const uint8_t kMessageQueueDepth = 4;
  static const uint8_t kMessageQueueMask = kMessageQueueDepth - 1;

  void resetSearch();
  void resetParser();
  void resetMessageQueue();
  bool processSample(uint16_t sample);
  bool updateEdgeTracker(uint16_t sample, EdgeEvent *edge);
  void handleSearchEdge(const EdgeEvent &edge);
  void beginSfdSearch(uint32_t edge_index);
  void pllHandleEdge(const EdgeEvent &edge);
  bool updateScheduledDecode(uint16_t sample);
  bool finishDecodedBit(bool normal_bit, uint16_t strength);
  bool consumeDecodedBit(bool normal_bit);
  bool consumeFrameBit(bool normal_bit);
  bool consumeByte(uint8_t byte);
  bool queueMessage(const Message &message);
  uint16_t currentSwing() const;
  uint16_t hysteresisCounts() const;
  uint16_t decodeThresholdCounts() const;
  void updateContrast(uint16_t strength);
  bool intervalIsGood(uint32_t interval, int32_t period_q8) const;

  DecoderConfig config_;
  uint8_t half_bit_samples_;
  uint8_t full_bit_samples_;
  int32_t nominal_period_q8_;
  int32_t min_period_q8_;
  int32_t max_period_q8_;

  uint32_t sample_index_;
  DecoderStats stats_;

  bool edge_tracker_initialized_;
  int32_t low_q8_;
  int32_t high_q8_;
  LightState light_state_;

  bool search_have_edge_;
  bool search_last_rising_;
  uint8_t search_good_edges_;
  uint32_t search_last_edge_index_;
  int32_t search_period_q8_;

  int64_t center_q8_;
  int32_t period_q8_;
  int16_t pending_phase_correction_q8_;
  bool saw_center_edge_;
  uint8_t lost_center_edges_;
  bool have_first_sample_;
  uint16_t first_sample_;
  uint16_t contrast_estimate_;

  State state_;
  uint8_t sfd_shift_;
  uint8_t sfd_bits_seen_;
  uint8_t byte_acc_;
  uint8_t bit_count_;
  uint16_t crc_calc_;
  uint16_t crc_rx_;
  uint8_t payload_pos_;
  Message working_message_;

  Message message_queue_[kMessageQueueDepth];
  uint8_t message_head_;
  uint8_t message_tail_;
};

} // namespace receiver

#endif // RECEIVER_MANCHESTER_DECODER_H
