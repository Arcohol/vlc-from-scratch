#include "ManchesterDecoder.h"

namespace receiver {

namespace {

uint8_t clampToUint8(uint32_t value, uint8_t minimum, uint8_t maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return static_cast<uint8_t>(value);
}

uint32_t roundedDivide(uint64_t numerator, uint32_t denominator) {
  return static_cast<uint32_t>((numerator + denominator / 2) / denominator);
}

} // namespace

DecoderConfig::DecoderConfig()
    : half_bit_us(40), sample_rate_hz(200000), preamble_edges_required(10),
      threshold_shift(6) {}

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
      center_offset_samples_(clampToUint8(half_bit_samples_ / 2, 1, 31)),
      edge_tolerance_samples_(clampToUint8(half_bit_samples_ / 4, 1, 6)) {
  reset();
}

void ManchesterDecoder::reset() {
  for (uint16_t i = 0; i < kRingSize; ++i) {
    ring_[i] = false;
  }

  sample_index_ = 0;
  threshold_initialized_ = false;
  threshold_q8_ = 0;
  threshold_ = 0;
  last_level_ = false;
  last_level_valid_ = false;
  last_edge_valid_ = false;
  last_edge_index_ = 0;
  resetSync();
}

void ManchesterDecoder::resetSync() {
  preamble_edge_count_ = 0;
  sync_state_ = SyncState::SearchingPreamble;
  next_edge_index_ = 0;
  resetFrame();
}

void ManchesterDecoder::resetFrame() {
  frame_state_ = FrameState::SearchingSfd;
  bit_count_ = 0;
  current_byte_ = 0;
  rolling_byte_ = 0;
  running_crc_ = 0xFFFF;
  payload_index_ = 0;
  working_message_.length = 0;
  working_message_.crc = 0;
}

bool ManchesterDecoder::pushSample(uint16_t sample, Message *out) {
  if (out != nullptr) {
    out->length = 0;
    out->crc = 0;
  }

  const uint32_t index = sample_index_++;

  if (!threshold_initialized_) {
    threshold_q8_ = static_cast<int32_t>(sample) << 8;
    threshold_ = sample;
    threshold_initialized_ = true;
  }

  const bool level = sample >= threshold_;
  ring_[index & kRingMask] = level;

  // The threshold follows raw ADC values slowly. Manchester is DC-balanced
  // over each bit, so this tracks sunlight drift without chasing individual
  // half-bit transitions.
  updateThreshold(sample);

  if (last_level_valid_ && level != last_level_) {
    handleTransition(index);
  }
  last_level_ = level;
  last_level_valid_ = true;

  if (sync_state_ == SyncState::DecodingBits) {
    return tryDecodeScheduledBit(out);
  }
  return false;
}

void ManchesterDecoder::updateThreshold(uint16_t sample) {
  const int32_t sample_q8 = static_cast<int32_t>(sample) << 8;
  threshold_q8_ += (sample_q8 - threshold_q8_) >> config_.threshold_shift;
  threshold_ = static_cast<uint16_t>((threshold_q8_ + 128) >> 8);
}

bool ManchesterDecoder::quantizedAt(uint32_t requested_index,
                                    bool *level) const {
  const uint32_t newest_index = sample_index_ - 1;
  if (requested_index > newest_index) {
    return false;
  }
  if (newest_index - requested_index >= kRingSize) {
    return false;
  }
  *level = ring_[requested_index & kRingMask];
  return true;
}

bool ManchesterDecoder::transitionAt(uint32_t edge_index) const {
  if (edge_index == 0) {
    return false;
  }

  bool before = false;
  bool after = false;
  return quantizedAt(edge_index - 1, &before) &&
         quantizedAt(edge_index, &after) && before != after;
}

void ManchesterDecoder::handleTransition(uint32_t edge_index) {
  if (sync_state_ != SyncState::SearchingPreamble) {
    last_edge_index_ = edge_index;
    last_edge_valid_ = true;
    return;
  }

  if (last_edge_valid_) {
    const uint32_t run = edge_index - last_edge_index_;
    const uint32_t expected = full_bit_samples_;
    const uint32_t delta = run > expected ? run - expected : expected - run;

    // The 0xAA preamble produces evenly spaced mid-bit edges. A run of these
    // edges gives us bit timing before we try to interpret bytes.
    if (delta <= edge_tolerance_samples_) {
      if (preamble_edge_count_ < 255) {
        ++preamble_edge_count_;
      }
    } else {
      preamble_edge_count_ = 0;
    }

    if (preamble_edge_count_ >= config_.preamble_edges_required) {
      sync_state_ = SyncState::DecodingBits;
      resetFrame();
      next_edge_index_ = edge_index + full_bit_samples_;
    }
  }

  last_edge_index_ = edge_index;
  last_edge_valid_ = true;
}

bool ManchesterDecoder::tryDecodeScheduledBit(Message *out) {
  const uint32_t newest_index = sample_index_ - 1;
  const uint32_t latest_needed = next_edge_index_ + center_offset_samples_ + 1;
  if (newest_index < latest_needed) {
    return false;
  }

  uint8_t bit = 0;
  if (!decodeBitAtEdge(next_edge_index_, &bit)) {
    failAndSearchAgain();
    return false;
  }

  next_edge_index_ += full_bit_samples_;
  return consumeBit(bit, out);
}

bool ManchesterDecoder::decodeBitAtEdge(uint32_t edge_index, uint8_t *bit) {
  static const int8_t candidate_offsets[] = {0, -1, 1};

  uint32_t corrected_edge = edge_index;
  bool found_transition = false;

  // If the transmitter is slightly fast or slow, the real mid-bit edge will
  // walk by one ADC sample from time to time. Snapping to a nearby transition
  // keeps the receiver phase aligned without expensive resampling.
  for (uint8_t i = 0;
       i < sizeof(candidate_offsets) / sizeof(candidate_offsets[0]); ++i) {
    const int32_t candidate =
        static_cast<int32_t>(edge_index) + candidate_offsets[i];
    if (candidate <= 0) {
      continue;
    }
    if (transitionAt(static_cast<uint32_t>(candidate))) {
      corrected_edge = static_cast<uint32_t>(candidate);
      found_transition = true;
      break;
    }
  }

  if (found_transition) {
    next_edge_index_ = corrected_edge;
  }

  bool first_half = false;
  bool second_half = false;
  if (!quantizedAt(corrected_edge - center_offset_samples_, &first_half) ||
      !quantizedAt(corrected_edge + center_offset_samples_, &second_half)) {
    return false;
  }

  if (first_half && !second_half) {
    *bit = 0;
    return true;
  }
  if (!first_half && second_half) {
    *bit = 1;
    return true;
  }
  return false;
}

bool ManchesterDecoder::consumeBit(uint8_t bit, Message *out) {
  if (frame_state_ == FrameState::SearchingSfd) {
    rolling_byte_ = static_cast<uint16_t>(((rolling_byte_ << 1) | bit) & 0xFF);
    if (rolling_byte_ == kSfd) {
      frame_state_ = FrameState::ReadingLength;
      bit_count_ = 0;
      current_byte_ = 0;
      running_crc_ = 0xFFFF;
      payload_index_ = 0;
      working_message_.length = 0;
      working_message_.crc = 0;
    }
    return false;
  }

  current_byte_ = static_cast<uint8_t>((current_byte_ << 1) | bit);
  ++bit_count_;
  if (bit_count_ < 8) {
    return false;
  }

  const uint8_t byte = current_byte_;
  current_byte_ = 0;
  bit_count_ = 0;
  return consumeByte(byte, out);
}

bool ManchesterDecoder::consumeByte(uint8_t byte, Message *out) {
  switch (frame_state_) {
  case FrameState::SearchingSfd:
    return false;

  case FrameState::ReadingLength:
    working_message_.length = byte;
    running_crc_ = crc16CcittFalseUpdate(running_crc_, byte);
    payload_index_ = 0;
    frame_state_ =
        byte == 0 ? FrameState::ReadingCrcHigh : FrameState::ReadingPayload;
    return false;

  case FrameState::ReadingPayload:
    working_message_.payload[payload_index_++] = byte;
    running_crc_ = crc16CcittFalseUpdate(running_crc_, byte);
    if (payload_index_ >= working_message_.length) {
      frame_state_ = FrameState::ReadingCrcHigh;
    }
    return false;

  case FrameState::ReadingCrcHigh:
    working_message_.crc = static_cast<uint16_t>(byte) << 8;
    frame_state_ = FrameState::ReadingCrcLow;
    return false;

  case FrameState::ReadingCrcLow:
    working_message_.crc |= byte;
    // Only CRC-valid frames are emitted. Invalid frames drop back to preamble
    // search so random light changes cannot leak partial payloads upstream.
    if (working_message_.crc == running_crc_) {
      if (out != nullptr) {
        *out = working_message_;
      }
      resetSync();
      return true;
    }
    failAndSearchAgain();
    return false;
  }

  return false;
}

void ManchesterDecoder::failAndSearchAgain() {
  resetSync();
  last_edge_valid_ = false;
  preamble_edge_count_ = 0;
}

} // namespace receiver
