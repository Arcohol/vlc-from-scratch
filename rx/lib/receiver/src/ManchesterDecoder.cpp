#include "ManchesterDecoder.h"

#include <string.h>

namespace receiver {

namespace {

const int32_t kQ8Value = 256;

uint8_t clampToUint8(uint32_t value, uint8_t minimum, uint8_t maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return static_cast<uint8_t>(value);
}

int32_t clampI32(int32_t value, int32_t low, int32_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

uint32_t roundedDivide(uint64_t numerator, uint32_t denominator) {
  return static_cast<uint32_t>((numerator + denominator / 2) / denominator);
}

uint32_t roundQ8ToU32(int64_t value_q8) {
  if (value_q8 <= 0) {
    return 0;
  }

  const int64_t rounded = (value_q8 + (kQ8Value / 2)) / kQ8Value;
  if (rounded > 0xFFFFFFFFLL) {
    return 0xFFFFFFFFUL;
  }
  return static_cast<uint32_t>(rounded);
}

uint16_t absI32ToU16(int32_t value) {
  if (value < 0) {
    value = -value;
  }
  if (value > 0xFFFF) {
    return 0xFFFF;
  }
  return static_cast<uint16_t>(value);
}

void copyMessage(Message *destination, const Message &source) {
  if (destination == nullptr) {
    return;
  }

  destination->length = source.length;
  destination->crc = source.crc;
  if (source.length > 0U) {
    memcpy(destination->payload, source.payload, source.length);
  }
}

} // namespace

DecoderConfig::DecoderConfig()
    : half_bit_us(40), sample_rate_hz(200000), preamble_edges_required(12),
      threshold_shift(5) {}

uint16_t crc16CcittFalseUpdate(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (uint8_t i = 0; i < 8; ++i) {
    if ((crc & 0x8000) != 0) {
      crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t crc16CcittFalse(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc = crc16CcittFalseUpdate(crc, data[i]);
  }
  return crc;
}

ManchesterDecoder::ManchesterDecoder(const DecoderConfig &config)
    : config_(config),
      half_bit_samples_(clampToUint8(
          roundedDivide(static_cast<uint64_t>(config.sample_rate_hz) *
                            config.half_bit_us,
                        1000000),
          2, 63)),
      full_bit_samples_(static_cast<uint8_t>(half_bit_samples_ * 2)),
      nominal_period_q8_(static_cast<int32_t>(full_bit_samples_) * kQ8),
      min_period_q8_(static_cast<int32_t>(
                         full_bit_samples_ > 3 ? full_bit_samples_ - 3 : 1) *
                     kQ8),
      max_period_q8_(static_cast<int32_t>(full_bit_samples_ + 3) * kQ8) {
  reset();
}

void ManchesterDecoder::reset() {
  sample_index_ = 0;

  stats_.messages = 0;
  stats_.crc_failures = 0;
  stats_.queue_drops = 0;
  stats_.sfd_timeouts = 0;
  stats_.weak_bits = 0;
  stats_.lost_center_edges = 0;
  stats_.length_errors = 0;
  stats_.signal_swing = 0;
  stats_.contrast = 0;

  edge_tracker_initialized_ = false;
  low_q8_ = 0;
  high_q8_ = 0;
  light_state_ = LightState::Unknown;
  contrast_estimate_ = 0;
  resetMessageQueue();
  resetSearch();
}

void ManchesterDecoder::resetStream() {
  contrast_estimate_ = 0;
  resetSearch();
}

void ManchesterDecoder::resetSearch() {
  state_ = State::SearchPreamble;

  search_have_edge_ = false;
  search_last_rising_ = false;
  search_good_edges_ = 0;
  search_last_edge_index_ = 0;
  search_period_q8_ = nominal_period_q8_;

  center_q8_ = 0;
  period_q8_ = nominal_period_q8_;
  pending_phase_correction_q8_ = 0;
  saw_center_edge_ = false;
  lost_center_edges_ = 0;
  have_first_sample_ = false;
  first_sample_ = 0;

  resetParser();
}

void ManchesterDecoder::resetParser() {
  sfd_shift_ = 0;
  sfd_bits_seen_ = 0;
  byte_acc_ = 0;
  bit_count_ = 0;
  crc_calc_ = kCrcInit;
  crc_rx_ = 0;
  payload_pos_ = 0;
  working_message_.length = 0;
  working_message_.crc = 0;
}

void ManchesterDecoder::resetMessageQueue() {
  message_head_ = 0;
  message_tail_ = 0;
}

bool ManchesterDecoder::processSample(uint16_t sample) {
  EdgeEvent edge;
  if (updateEdgeTracker(sample, &edge)) {
    if (state_ == State::SearchPreamble) {
      handleSearchEdge(edge);
    } else {
      pllHandleEdge(edge);
    }
  }

  const bool emitted = updateScheduledDecode(sample);
  ++sample_index_;
  return emitted;
}

void ManchesterDecoder::pushSample(uint16_t sample) {
  (void)processSample(sample);
}

bool ManchesterDecoder::pushSample(uint16_t sample, Message *out) {
  const bool emitted = processSample(sample);
  if (emitted && out != nullptr) {
    return popMessage(out);
  }
  return emitted;
}

void ManchesterDecoder::pushSamples(const uint16_t *samples, size_t count) {
  if (samples == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    (void)processSample(samples[i]);
  }
}

bool ManchesterDecoder::hasMessage() const {
  return message_head_ != message_tail_;
}

bool ManchesterDecoder::popMessage(Message *out) {
  if (out == nullptr || message_head_ == message_tail_) {
    return false;
  }

  copyMessage(out, message_queue_[message_tail_]);
  message_tail_ =
      static_cast<uint8_t>((message_tail_ + 1U) & kMessageQueueMask);
  return true;
}

DecoderStats ManchesterDecoder::stats() const {
  DecoderStats result = stats_;
  result.signal_swing = currentSwing();
  result.contrast = contrast_estimate_;
  return result;
}

uint16_t ManchesterDecoder::threshold() const {
  if (!edge_tracker_initialized_) {
    return 0;
  }
  return static_cast<uint16_t>((low_q8_ + high_q8_ + kQ8) / (2 * kQ8));
}

uint16_t ManchesterDecoder::currentSwing() const {
  if (!edge_tracker_initialized_ || high_q8_ <= low_q8_) {
    return 0;
  }
  return static_cast<uint16_t>((high_q8_ - low_q8_) >> 8);
}

uint16_t ManchesterDecoder::hysteresisCounts() const {
  uint16_t hysteresis = static_cast<uint16_t>(currentSwing() / 8U);
  if (hysteresis < kMinHysteresisCounts) {
    hysteresis = kMinHysteresisCounts;
  }
  return hysteresis;
}

uint16_t ManchesterDecoder::decodeThresholdCounts() const {
  uint16_t threshold = kMinDeltaCounts;
  if (contrast_estimate_ > 0U) {
    const uint16_t adaptive = static_cast<uint16_t>(contrast_estimate_ / 3U);
    if (adaptive > threshold) {
      threshold = adaptive;
    }
  } else {
    const uint16_t adaptive = static_cast<uint16_t>(currentSwing() / 6U);
    if (adaptive > threshold) {
      threshold = adaptive;
    }
  }
  return threshold;
}

void ManchesterDecoder::updateContrast(uint16_t strength) {
  if (contrast_estimate_ == 0U) {
    contrast_estimate_ = strength;
    return;
  }

  const int32_t delta =
      static_cast<int32_t>(strength) - static_cast<int32_t>(contrast_estimate_);
  contrast_estimate_ =
      static_cast<uint16_t>(static_cast<int32_t>(contrast_estimate_) +
                            (delta >> kContrastAdaptShift));
}

bool ManchesterDecoder::updateEdgeTracker(uint16_t sample, EdgeEvent *edge) {
  const int32_t sample_q8 = static_cast<int32_t>(sample) << 8;

  if (!edge_tracker_initialized_) {
    edge_tracker_initialized_ = true;
    low_q8_ = sample_q8;
    high_q8_ = sample_q8;
    light_state_ = LightState::Unknown;
    return false;
  }

  // Track the low and high light envelopes separately. Fast attack lets the
  // estimates follow real level changes; slow decay prevents noise and slopes
  // from collapsing the estimated signal swing.
  if (sample_q8 < low_q8_) {
    low_q8_ += (sample_q8 - low_q8_) >> 2;
  } else {
    low_q8_ += (sample_q8 - low_q8_) >> config_.threshold_shift;
  }

  if (sample_q8 > high_q8_) {
    high_q8_ += (sample_q8 - high_q8_) >> 2;
  } else {
    high_q8_ += (sample_q8 - high_q8_) >> config_.threshold_shift;
  }

  const uint16_t swing = currentSwing();
  if (swing < kMinSignalSwingCounts) {
    light_state_ = LightState::Unknown;
    return false;
  }

  const int32_t midpoint_q8 = (low_q8_ + high_q8_) / 2;
  const int32_t hysteresis_q8 = static_cast<int32_t>(hysteresisCounts()) << 8;

  if (light_state_ == LightState::Unknown) {
    light_state_ =
        sample_q8 >= midpoint_q8 ? LightState::High : LightState::Low;
    return false;
  }

  if (light_state_ == LightState::Low &&
      sample_q8 > midpoint_q8 + hysteresis_q8) {
    light_state_ = LightState::High;
    edge->sample_index = sample_index_;
    edge->rising = true;
    edge->swing = swing;
    return true;
  }

  if (light_state_ == LightState::High &&
      sample_q8 < midpoint_q8 - hysteresis_q8) {
    light_state_ = LightState::Low;
    edge->sample_index = sample_index_;
    edge->rising = false;
    edge->swing = swing;
    return true;
  }

  return false;
}

bool ManchesterDecoder::intervalIsGood(uint32_t interval,
                                       int32_t period_q8) const {
  const int32_t expected = period_q8 / kQ8;
  int32_t tolerance = expected / 4;
  if (tolerance < 3) {
    tolerance = 3;
  }

  const int32_t error = static_cast<int32_t>(interval) - expected;
  return error >= -tolerance && error <= tolerance;
}

void ManchesterDecoder::handleSearchEdge(const EdgeEvent &edge) {
  if (!search_have_edge_) {
    search_have_edge_ = true;
    search_last_rising_ = edge.rising;
    search_good_edges_ = 1;
    search_last_edge_index_ = edge.sample_index;
    search_period_q8_ = nominal_period_q8_;
    return;
  }

  const uint32_t interval = edge.sample_index - search_last_edge_index_;
  const bool good = (edge.rising != search_last_rising_) &&
                    intervalIsGood(interval, search_period_q8_) &&
                    edge.swing >= kMinSignalSwingCounts;

  if (!good) {
    search_last_rising_ = edge.rising;
    search_good_edges_ = 1;
    search_last_edge_index_ = edge.sample_index;
    search_period_q8_ = nominal_period_q8_;
    return;
  }

  // The 0xAA preamble gives alternating center edges. Average their spacing so
  // the PLL starts near the transmitter's real bit period instead of the
  // nominal hard-coded period.
  const int32_t interval_q8 = static_cast<int32_t>(interval) * kQ8;
  search_period_q8_ += (interval_q8 - search_period_q8_) >> 2;
  search_last_rising_ = edge.rising;
  search_last_edge_index_ = edge.sample_index;

  if (search_good_edges_ < 0xFFU) {
    ++search_good_edges_;
  }

  if (search_good_edges_ >= config_.preamble_edges_required) {
    beginSfdSearch(edge.sample_index);
  }
}

void ManchesterDecoder::beginSfdSearch(uint32_t edge_index) {
  state_ = State::FindSfd;
  resetParser();

  center_q8_ = (static_cast<int64_t>(edge_index) * kQ8) + search_period_q8_;
  period_q8_ = clampI32(search_period_q8_, min_period_q8_, max_period_q8_);
  pending_phase_correction_q8_ = 0;
  saw_center_edge_ = false;
  lost_center_edges_ = 0;
  have_first_sample_ = false;
}

void ManchesterDecoder::pllHandleEdge(const EdgeEvent &edge) {
  if (state_ == State::SearchPreamble) {
    return;
  }

  const uint32_t expected = roundQ8ToU32(center_q8_);
  const int32_t error =
      static_cast<int32_t>(edge.sample_index) - static_cast<int32_t>(expected);
  int32_t tolerance = (period_q8_ / kQ8) / 4;
  if (tolerance < 3) {
    tolerance = 3;
  }

  if (error < -tolerance || error > tolerance) {
    return;
  }

  // Do not jump the clock all the way to the observed edge. A fractional phase
  // nudge plus a slower period adjustment follows transmitter drift without
  // chasing one noisy crossing.
  pending_phase_correction_q8_ =
      static_cast<int16_t>(clampI32(error * 64, -128, 128));
  period_q8_ =
      clampI32(period_q8_ + (error * 8), min_period_q8_, max_period_q8_);
  saw_center_edge_ = true;
}

bool ManchesterDecoder::updateScheduledDecode(uint16_t sample) {
  if (state_ == State::SearchPreamble) {
    return false;
  }

  const int32_t quarter_period_q8 = period_q8_ / 4;
  const uint32_t first_index = roundQ8ToU32(center_q8_ - quarter_period_q8);
  const uint32_t second_index = roundQ8ToU32(center_q8_ + quarter_period_q8);

  if (!have_first_sample_ && sample_index_ >= first_index) {
    first_sample_ = sample;
    have_first_sample_ = true;
  }

  if (have_first_sample_ && sample_index_ >= second_index) {
    const int32_t delta =
        static_cast<int32_t>(sample) - static_cast<int32_t>(first_sample_);
    const uint16_t strength = absI32ToU16(delta);
    const uint16_t threshold = decodeThresholdCounts();

    // Bits are decided from the sign of the half-bit contrast. Low contrast is
    // still tracked, but this stable capture showed that throwing those bits
    // away caused packet erasures while their polarity was still reliable.
    if (strength < threshold) {
      ++stats_.weak_bits;
      return finishDecodedBit(delta > 0, threshold);
    }

    return finishDecodedBit(delta > 0, strength);
  }

  return false;
}

bool ManchesterDecoder::finishDecodedBit(bool normal_bit, uint16_t strength) {
  updateContrast(strength);
  const bool emitted = consumeDecodedBit(normal_bit);

  if (state_ == State::SearchPreamble) {
    return emitted;
  }

  if (saw_center_edge_) {
    lost_center_edges_ = 0;
  } else if (++lost_center_edges_ > kMaxLostCenterEdges) {
    ++stats_.lost_center_edges;
    contrast_estimate_ = 0;
    resetSearch();
    return emitted;
  }

  center_q8_ += period_q8_ + pending_phase_correction_q8_;
  pending_phase_correction_q8_ = 0;
  saw_center_edge_ = false;
  have_first_sample_ = false;
  return emitted;
}

bool ManchesterDecoder::consumeDecodedBit(bool normal_bit) {
  if (state_ == State::FindSfd) {
    sfd_shift_ =
        static_cast<uint8_t>((sfd_shift_ << 1) | (normal_bit ? 1U : 0U));
    ++sfd_bits_seen_;

    if (sfd_shift_ == kSfd) {
      state_ = State::ReadLength;
      byte_acc_ = 0;
      bit_count_ = 0;
      crc_calc_ = kCrcInit;
      crc_rx_ = 0;
      payload_pos_ = 0;
      working_message_.length = 0;
      working_message_.crc = 0;
      return false;
    }

    if (sfd_bits_seen_ > kSfdSearchMaxBits) {
      ++stats_.sfd_timeouts;
      resetSearch();
    }

    return false;
  }

  return consumeFrameBit(normal_bit);
}

bool ManchesterDecoder::consumeFrameBit(bool normal_bit) {
  byte_acc_ = static_cast<uint8_t>((byte_acc_ << 1) | (normal_bit ? 1U : 0U));
  ++bit_count_;

  if (bit_count_ < 8U) {
    return false;
  }

  const uint8_t byte = byte_acc_;
  byte_acc_ = 0;
  bit_count_ = 0;
  return consumeByte(byte);
}

bool ManchesterDecoder::queueMessage(const Message &message) {
  const uint8_t next =
      static_cast<uint8_t>((message_head_ + 1U) & kMessageQueueMask);
  if (next == message_tail_) {
    ++stats_.queue_drops;
    return false;
  }

  Message *queued = &message_queue_[message_head_];
  queued->length = message.length;
  queued->crc = message.crc;
  if (message.length > 0U) {
    memcpy(queued->payload, message.payload, message.length);
  }
  message_head_ = next;
  return true;
}

bool ManchesterDecoder::consumeByte(uint8_t byte) {
  switch (state_) {
  case State::SearchPreamble:
  case State::FindSfd:
    return false;

  case State::ReadLength:
    working_message_.length = byte;
    payload_pos_ = 0;
    crc_calc_ = crc16CcittFalseUpdate(kCrcInit, byte);
    state_ = byte == 0U ? State::ReadCrcHigh : State::ReadPayload;
    return false;

  case State::ReadPayload:
    if (payload_pos_ >= sizeof(working_message_.payload)) {
      ++stats_.length_errors;
      resetSearch();
      return false;
    }
    working_message_.payload[payload_pos_++] = byte;
    crc_calc_ = crc16CcittFalseUpdate(crc_calc_, byte);
    if (payload_pos_ >= working_message_.length) {
      state_ = State::ReadCrcHigh;
    }
    return false;

  case State::ReadCrcHigh:
    crc_rx_ = static_cast<uint16_t>(byte) << 8;
    state_ = State::ReadCrcLow;
    return false;

  case State::ReadCrcLow:
    crc_rx_ |= byte;
    working_message_.crc = crc_rx_;
    if (crc_rx_ == crc_calc_) {
      const bool queued = queueMessage(working_message_);
      if (queued) {
        ++stats_.messages;
      }
      resetSearch();
      return queued;
    }

    ++stats_.crc_failures;
    resetSearch();
    return false;
  }

  return false;
}

} // namespace receiver
