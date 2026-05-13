#include "vlc_rx.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace {

constexpr uint8_t kSfd = 0xD5;
constexpr uint16_t kCrcInit = 0xFFFF;
constexpr uint16_t kCrcPoly = 0x1021;

constexpr uint8_t kMessageQueueDepth = 4;
constexpr uint8_t kMessageQueueMask = kMessageQueueDepth - 1;
static_assert((kMessageQueueDepth & kMessageQueueMask) == 0,
              "message queue depth must be power of two");
static_assert(VLC_RX_MAX_PAYLOAD_LEN <= 255U,
              "RxMessage::len stores the payload length in one byte");

constexpr uint16_t kMinSignalSwingCounts = 60;
constexpr uint16_t kMinHysteresisCounts = 18;
constexpr uint16_t kMinDeltaCounts = 30;
constexpr uint8_t kSignalAdaptShift = 5;
constexpr uint8_t kContrastAdaptShift = 3;
constexpr uint8_t kMaxLostCenterEdges = 5;
constexpr uint8_t kSfdSearchMaxBits = 36;

constexpr int32_t kQ8 = 256;
constexpr int32_t kNominalPeriodQ8 =
    static_cast<int32_t>(VLC_RX_SAMPLES_PER_BIT) * kQ8;
constexpr int32_t kMinPeriodQ8 =
    static_cast<int32_t>(VLC_RX_SAMPLES_PER_BIT - 3U) * kQ8;
constexpr int32_t kMaxPeriodQ8 =
    static_cast<int32_t>(VLC_RX_SAMPLES_PER_BIT + 3U) * kQ8;

enum class LightState : uint8_t {
  Unknown,
  Low,
  High,
};

enum class RxState : uint8_t {
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

struct EdgeTracker {
  bool initialized;
  int32_t low_q8;
  int32_t high_q8;
  LightState state;
};

struct SearchState {
  bool have_edge;
  bool last_rising;
  uint8_t good_edges;
  uint32_t last_edge_index;
  int32_t period_q8;
};

struct TimingState {
  int64_t center_q8;
  int32_t period_q8;
  int16_t pending_phase_correction_q8;
  bool saw_center_edge;
  uint8_t lost_center_edges;
  bool have_first_sample;
  uint16_t first_sample;
};

struct ParserState {
  bool invert_polarity;
  uint8_t byte_acc;
  uint8_t bit_count;
  uint8_t length;
  uint8_t payload_pos;
  uint16_t crc_calc;
  uint16_t crc_rx;
  uint8_t payload[VLC_RX_MAX_PAYLOAD_LEN];
};

EdgeTracker edge_tracker;
SearchState search_state;
TimingState timing_state;
ParserState parser_state;
RxState rx_state;

uint32_t sample_index;
uint16_t contrast_estimate;
uint8_t sfd_shift_normal;
uint8_t sfd_shift_inverted;
uint8_t sfd_bits_seen;

RxMessage message_queue[kMessageQueueDepth];
uint8_t message_head;
uint8_t message_tail;
RxStats rx_stats;

int32_t clamp_i32(int32_t value, int32_t low, int32_t high) {
  if (value < low) {
    return low;
  }

  if (value > high) {
    return high;
  }

  return value;
}

uint16_t abs_i32_to_u16(int32_t value) {
  if (value < 0) {
    value = -value;
  }

  if (value > 0xFFFF) {
    return 0xFFFF;
  }

  return static_cast<uint16_t>(value);
}

uint32_t round_q8_to_u32(int64_t value_q8) {
  if (value_q8 <= 0) {
    return 0;
  }

  int64_t rounded = (value_q8 + (kQ8 / 2)) / kQ8;
  if (rounded > 0xFFFFFFFFLL) {
    return 0xFFFFFFFFUL;
  }

  return static_cast<uint32_t>(rounded);
}

uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;

  for (uint8_t bit = 0; bit < 8; ++bit) {
    if ((crc & 0x8000U) != 0U) {
      crc = static_cast<uint16_t>((crc << 1) ^ kCrcPoly);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

void reset_search() {
  rx_state = RxState::SearchPreamble;
  memset(&search_state, 0, sizeof(search_state));
  search_state.period_q8 = kNominalPeriodQ8;
  memset(&timing_state, 0, sizeof(timing_state));
  timing_state.period_q8 = kNominalPeriodQ8;
  memset(&parser_state, 0, sizeof(parser_state));
  sfd_shift_normal = 0;
  sfd_shift_inverted = 0;
  sfd_bits_seen = 0;
}

uint16_t current_swing() {
  if (!edge_tracker.initialized || edge_tracker.high_q8 <= edge_tracker.low_q8) {
    return 0;
  }

  return static_cast<uint16_t>((edge_tracker.high_q8 - edge_tracker.low_q8) >> 8);
}

uint16_t hysteresis_counts() {
  uint16_t swing = current_swing();
  uint16_t hysteresis = static_cast<uint16_t>(swing / 8U);

  if (hysteresis < kMinHysteresisCounts) {
    hysteresis = kMinHysteresisCounts;
  }

  return hysteresis;
}

uint16_t decode_threshold_counts() {
  uint16_t threshold = kMinDeltaCounts;

  if (contrast_estimate > 0U) {
    uint16_t adaptive = static_cast<uint16_t>(contrast_estimate / 3U);
    if (adaptive > threshold) {
      threshold = adaptive;
    }
  } else {
    uint16_t swing = current_swing();
    uint16_t adaptive = static_cast<uint16_t>(swing / 6U);
    if (adaptive > threshold) {
      threshold = adaptive;
    }
  }

  return threshold;
}

void update_contrast(uint16_t strength) {
  if (contrast_estimate == 0U) {
    contrast_estimate = strength;
    return;
  }

  int32_t delta = static_cast<int32_t>(strength) -
                  static_cast<int32_t>(contrast_estimate);
  contrast_estimate = static_cast<uint16_t>(
      static_cast<int32_t>(contrast_estimate) +
      (delta >> kContrastAdaptShift));
}

bool update_edge_tracker(uint16_t sample, EdgeEvent *edge) {
  int32_t sample_q8 = static_cast<int32_t>(sample) << 8;

  if (!edge_tracker.initialized) {
    edge_tracker.initialized = true;
    edge_tracker.low_q8 = sample_q8;
    edge_tracker.high_q8 = sample_q8;
    edge_tracker.state = LightState::Unknown;
    return false;
  }

  if (sample_q8 < edge_tracker.low_q8) {
    edge_tracker.low_q8 += (sample_q8 - edge_tracker.low_q8) >> 2;
  } else {
    edge_tracker.low_q8 += (sample_q8 - edge_tracker.low_q8) >> kSignalAdaptShift;
  }

  if (sample_q8 > edge_tracker.high_q8) {
    edge_tracker.high_q8 += (sample_q8 - edge_tracker.high_q8) >> 2;
  } else {
    edge_tracker.high_q8 +=
        (sample_q8 - edge_tracker.high_q8) >> kSignalAdaptShift;
  }

  uint16_t swing = current_swing();
  if (swing < kMinSignalSwingCounts) {
    edge_tracker.state = LightState::Unknown;
    return false;
  }

  int32_t midpoint_q8 = (edge_tracker.low_q8 + edge_tracker.high_q8) / 2;
  int32_t hysteresis_q8 = static_cast<int32_t>(hysteresis_counts()) << 8;

  if (edge_tracker.state == LightState::Unknown) {
    edge_tracker.state =
        sample_q8 >= midpoint_q8 ? LightState::High : LightState::Low;
    return false;
  }

  if (edge_tracker.state == LightState::Low &&
      sample_q8 > midpoint_q8 + hysteresis_q8) {
    edge_tracker.state = LightState::High;
    edge->sample_index = sample_index;
    edge->rising = true;
    edge->swing = swing;
    return true;
  }

  if (edge_tracker.state == LightState::High &&
      sample_q8 < midpoint_q8 - hysteresis_q8) {
    edge_tracker.state = LightState::Low;
    edge->sample_index = sample_index;
    edge->rising = false;
    edge->swing = swing;
    return true;
  }

  return false;
}

bool interval_is_good(uint32_t interval, int32_t period_q8) {
  int32_t expected = period_q8 / kQ8;
  int32_t tolerance = expected / 4;

  if (tolerance < 3) {
    tolerance = 3;
  }

  int32_t error = static_cast<int32_t>(interval) - expected;
  return error >= -tolerance && error <= tolerance;
}

void begin_sfd_search(uint32_t edge_index) {
  rx_state = RxState::FindSfd;
  sfd_shift_normal = 0;
  sfd_shift_inverted = 0;
  sfd_bits_seen = 0;

  timing_state.center_q8 =
      (static_cast<int64_t>(edge_index) * kQ8) + search_state.period_q8;
  timing_state.period_q8 =
      clamp_i32(search_state.period_q8, kMinPeriodQ8, kMaxPeriodQ8);
  timing_state.pending_phase_correction_q8 = 0;
  timing_state.saw_center_edge = false;
  timing_state.lost_center_edges = 0;
  timing_state.have_first_sample = false;
}

void handle_search_edge(const EdgeEvent &edge) {
  if (!search_state.have_edge) {
    search_state.have_edge = true;
    search_state.last_rising = edge.rising;
    search_state.good_edges = 1;
    search_state.last_edge_index = edge.sample_index;
    search_state.period_q8 = kNominalPeriodQ8;
    return;
  }

  uint32_t interval = edge.sample_index - search_state.last_edge_index;
  bool good = (edge.rising != search_state.last_rising) &&
              interval_is_good(interval, search_state.period_q8) &&
              edge.swing >= kMinSignalSwingCounts;

  if (!good) {
    search_state.last_rising = edge.rising;
    search_state.good_edges = 1;
    search_state.last_edge_index = edge.sample_index;
    search_state.period_q8 = kNominalPeriodQ8;
    return;
  }

  int32_t interval_q8 = static_cast<int32_t>(interval) * kQ8;
  search_state.period_q8 += (interval_q8 - search_state.period_q8) >> 2;
  search_state.last_rising = edge.rising;
  search_state.last_edge_index = edge.sample_index;

  if (search_state.good_edges < 0xFFU) {
    ++search_state.good_edges;
  }

  if (search_state.good_edges >= VLC_RX_PREAMBLE_MIN_EDGES) {
    begin_sfd_search(edge.sample_index);
  }
}

void pll_handle_edge(const EdgeEvent &edge) {
  if (rx_state == RxState::SearchPreamble) {
    return;
  }

  uint32_t expected = round_q8_to_u32(timing_state.center_q8);
  int32_t error = static_cast<int32_t>(edge.sample_index - expected);
  int32_t tolerance = (timing_state.period_q8 / kQ8) / 4;

  if (tolerance < 3) {
    tolerance = 3;
  }

  if (error < -tolerance || error > tolerance) {
    return;
  }

  timing_state.pending_phase_correction_q8 =
      static_cast<int16_t>(clamp_i32(error * 64, -128, 128));
  timing_state.period_q8 =
      clamp_i32(timing_state.period_q8 + (error * 8), kMinPeriodQ8,
                kMaxPeriodQ8);
  timing_state.saw_center_edge = true;
}

void start_frame_parser(bool invert_polarity) {
  parser_state.invert_polarity = invert_polarity;
  parser_state.byte_acc = 0;
  parser_state.bit_count = 0;
  parser_state.length = 0;
  parser_state.payload_pos = 0;
  parser_state.crc_calc = kCrcInit;
  parser_state.crc_rx = 0;
  rx_state = RxState::ReadLength;
}

bool push_message() {
  uint8_t next = static_cast<uint8_t>((message_head + 1U) & kMessageQueueMask);

  if (next == message_tail) {
    ++rx_stats.queue_drops;
    return false;
  }

  RxMessage *message = &message_queue[message_head];
  message->len = parser_state.length;

  if (parser_state.length > 0U) {
    memcpy(message->data, parser_state.payload, parser_state.length);
  }

  message_head = next;
  ++rx_stats.messages;
  return true;
}

void parser_process_byte(uint8_t byte) {
  switch (rx_state) {
    case RxState::ReadLength:
#if VLC_RX_MAX_PAYLOAD_LEN < 255U
      if (byte > VLC_RX_MAX_PAYLOAD_LEN) {
        ++rx_stats.length_errors;
        reset_search();
        break;
      }
#endif

      parser_state.length = byte;
      parser_state.payload_pos = 0;
      parser_state.crc_calc = crc16_update(kCrcInit, byte);
      rx_state =
          parser_state.length == 0U ? RxState::ReadCrcHigh : RxState::ReadPayload;
      break;

    case RxState::ReadPayload:
      parser_state.payload[parser_state.payload_pos++] = byte;
      parser_state.crc_calc = crc16_update(parser_state.crc_calc, byte);
      if (parser_state.payload_pos >= parser_state.length) {
        rx_state = RxState::ReadCrcHigh;
      }
      break;

    case RxState::ReadCrcHigh:
      parser_state.crc_rx = static_cast<uint16_t>(byte) << 8;
      rx_state = RxState::ReadCrcLow;
      break;

    case RxState::ReadCrcLow:
      parser_state.crc_rx |= byte;
      if (parser_state.crc_rx == parser_state.crc_calc) {
        (void)push_message();
      } else {
        ++rx_stats.crc_failures;
      }
      reset_search();
      break;

    default:
      reset_search();
      break;
  }
}

void consume_frame_bit(bool normal_bit) {
  bool bit = parser_state.invert_polarity ? !normal_bit : normal_bit;
  parser_state.byte_acc = static_cast<uint8_t>((parser_state.byte_acc << 1) |
                                               (bit ? 1U : 0U));
  ++parser_state.bit_count;

  if (parser_state.bit_count == 8U) {
    uint8_t byte = parser_state.byte_acc;
    parser_state.byte_acc = 0;
    parser_state.bit_count = 0;
    parser_process_byte(byte);
  }
}

void consume_decoded_bit(bool normal_bit) {
  if (rx_state == RxState::FindSfd) {
    sfd_shift_normal =
        static_cast<uint8_t>((sfd_shift_normal << 1) | (normal_bit ? 1U : 0U));
    sfd_shift_inverted = static_cast<uint8_t>(
        (sfd_shift_inverted << 1) | (normal_bit ? 0U : 1U));

    ++sfd_bits_seen;

    if (sfd_shift_normal == kSfd) {
      start_frame_parser(false);
      return;
    }

    if (sfd_shift_inverted == kSfd) {
      start_frame_parser(true);
      return;
    }

    if (sfd_bits_seen > kSfdSearchMaxBits) {
      ++rx_stats.sfd_timeouts;
      reset_search();
    }

    return;
  }

  consume_frame_bit(normal_bit);
}

void finish_decoded_bit(bool normal_bit, uint16_t strength) {
  update_contrast(strength);
  consume_decoded_bit(normal_bit);

  if (rx_state == RxState::SearchPreamble) {
    return;
  }

  if (timing_state.saw_center_edge) {
    timing_state.lost_center_edges = 0;
  } else if (++timing_state.lost_center_edges > kMaxLostCenterEdges) {
    ++rx_stats.lost_center_edges;
    contrast_estimate = 0;
    reset_search();
    return;
  }

  timing_state.center_q8 += timing_state.period_q8 +
                            timing_state.pending_phase_correction_q8;
  timing_state.pending_phase_correction_q8 = 0;
  timing_state.saw_center_edge = false;
  timing_state.have_first_sample = false;
}

void update_scheduled_decode(uint16_t sample) {
  if (rx_state == RxState::SearchPreamble) {
    return;
  }

  int32_t quarter_period_q8 = timing_state.period_q8 / 4;
  uint32_t first_index =
      round_q8_to_u32(timing_state.center_q8 - quarter_period_q8);
  uint32_t second_index =
      round_q8_to_u32(timing_state.center_q8 + quarter_period_q8);

  if (!timing_state.have_first_sample && sample_index >= first_index) {
    timing_state.first_sample = sample;
    timing_state.have_first_sample = true;
  }

  if (timing_state.have_first_sample && sample_index >= second_index) {
    int32_t delta = static_cast<int32_t>(sample) -
                    static_cast<int32_t>(timing_state.first_sample);
    uint16_t strength = abs_i32_to_u16(delta);
    uint16_t threshold = decode_threshold_counts();

    if (strength < threshold) {
      ++rx_stats.weak_bits;
      contrast_estimate = 0;
      reset_search();
      return;
    }

    finish_decoded_bit(delta > 0, strength);
  }
}

}  // namespace

void vlc_rx_init() {
  memset(&edge_tracker, 0, sizeof(edge_tracker));
  sample_index = 0;
  contrast_estimate = 0;
  message_head = 0;
  message_tail = 0;
  memset(&rx_stats, 0, sizeof(rx_stats));
  reset_search();
}

void vlc_rx_reset() {
  contrast_estimate = 0;
  reset_search();
}

void vlc_rx_push_sample(uint16_t sample) {
  EdgeEvent edge;

  if (update_edge_tracker(sample, &edge)) {
    if (rx_state == RxState::SearchPreamble) {
      handle_search_edge(edge);
    } else {
      pll_handle_edge(edge);
    }
  }

  update_scheduled_decode(sample);
  ++sample_index;
}

bool vlc_rx_has_message() { return message_head != message_tail; }

bool vlc_rx_pop_message(RxMessage *message) {
  if (message == nullptr || message_head == message_tail) {
    return false;
  }

  *message = message_queue[message_tail];
  message_tail = static_cast<uint8_t>((message_tail + 1U) & kMessageQueueMask);
  return true;
}

void vlc_rx_get_stats(RxStats *stats) {
  if (stats == nullptr) {
    return;
  }

  *stats = rx_stats;
  stats->signal_swing = current_swing();
  stats->contrast = contrast_estimate;
}
