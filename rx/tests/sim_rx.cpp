#include "../vlc_rx.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint16_t kCrcInit = 0xFFFF;
constexpr uint16_t kCrcPoly = 0x1021;

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

std::vector<uint8_t> make_frame(const std::string &payload) {
  std::vector<uint8_t> frame;
  frame.push_back(0xAA);
  frame.push_back(0xAA);
  frame.push_back(0xAA);
  frame.push_back(0xAA);
  frame.push_back(0xD5);
  frame.push_back(static_cast<uint8_t>(payload.size()));

  uint16_t crc = crc16_update(kCrcInit, static_cast<uint8_t>(payload.size()));

  for (char ch : payload) {
    uint8_t byte = static_cast<uint8_t>(ch);
    frame.push_back(byte);
    crc = crc16_update(crc, byte);
  }

  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(static_cast<uint8_t>(crc));
  return frame;
}

struct WaveformOptions {
  double half_samples = 8.0;
  uint16_t low_level = 1000;
  uint16_t high_level = 3000;
  bool invert = false;
  bool corrupt_payload_bit = false;
  int16_t drift_per_sample_times_1024 = 0;
};

void append_level(std::vector<uint16_t> *samples, uint16_t level,
                  uint32_t count, const WaveformOptions &options) {
  for (uint32_t i = 0; i < count; ++i) {
    int32_t drift =
        static_cast<int32_t>(samples->size()) * options.drift_per_sample_times_1024;
    int32_t value = static_cast<int32_t>(level) + (drift / 1024);

    if (options.invert) {
      value = 4095 - value;
    }

    if (value < 0) {
      value = 0;
    } else if (value > 4095) {
      value = 4095;
    }

    samples->push_back(static_cast<uint16_t>(value));
  }
}

void append_half(std::vector<uint16_t> *samples, bool high, double *time,
                 const WaveformOptions &options) {
  double next = *time + options.half_samples;
  uint32_t start_index = static_cast<uint32_t>(*time + 0.5);
  uint32_t end_index = static_cast<uint32_t>(next + 0.5);
  uint32_t count = end_index > start_index ? end_index - start_index : 1;
  append_level(samples, high ? options.high_level : options.low_level, count,
               options);
  *time = next;
}

std::vector<uint16_t> make_waveform(const std::string &payload,
                                    const WaveformOptions &options = {}) {
  std::vector<uint8_t> frame = make_frame(payload);
  std::vector<uint16_t> samples;
  double time = 0.0;

  append_level(&samples, options.high_level, 96, options);

  uint32_t bit_index = 0;
  for (uint8_t byte : frame) {
    for (int bit = 7; bit >= 0; --bit) {
      bool one = (byte & (1U << bit)) != 0U;
      bool first_high = !one;
      bool second_high = one;

      if (options.corrupt_payload_bit && bit_index == 48U) {
        first_high = !first_high;
        second_high = !second_high;
      }

      append_half(&samples, first_high, &time, options);
      append_half(&samples, second_high, &time, options);
      ++bit_index;
    }
  }

  append_level(&samples, options.high_level, 160, options);
  return samples;
}

bool run_decode(const std::vector<uint16_t> &samples, std::string *message) {
  vlc_rx_init();

  for (uint16_t sample : samples) {
    vlc_rx_push_sample(sample);
  }

  RxMessage rx_message;
  if (!vlc_rx_pop_message(&rx_message)) {
    return false;
  }

  message->assign(reinterpret_cast<const char *>(rx_message.data), rx_message.len);
  return true;
}

std::vector<std::string> run_decode_stream(
    const std::vector<std::vector<uint16_t>> &waveforms) {
  std::vector<std::string> messages;
  vlc_rx_init();

  for (const auto &waveform : waveforms) {
    for (uint16_t sample : waveform) {
      vlc_rx_push_sample(sample);
    }

    RxMessage rx_message;
    while (vlc_rx_pop_message(&rx_message)) {
      messages.emplace_back(reinterpret_cast<const char *>(rx_message.data),
                            rx_message.len);
    }
  }

  return messages;
}

void expect_message(const std::string &payload,
                    const WaveformOptions &options = {}) {
  std::string decoded;
  bool ok = run_decode(make_waveform(payload, options), &decoded);

  if (!ok || decoded != payload) {
    std::fprintf(stderr, "decode failed: expected '%s', got '%s'\n",
                 payload.c_str(), decoded.c_str());
    std::abort();
  }
}

void expect_no_message(const std::string &payload,
                       const WaveformOptions &options = {}) {
  std::string decoded;
  bool ok = run_decode(make_waveform(payload, options), &decoded);

  if (ok) {
    std::fprintf(stderr, "unexpected message: '%s'\n", decoded.c_str());
    std::abort();
  }
}

void expect_contrast_drop_recovery() {
  WaveformOptions dim;
  dim.low_level = 1900;
  dim.high_level = 2200;

  std::vector<std::string> messages =
      run_decode_stream({make_waveform("hello"), make_waveform("dim1", dim),
                         make_waveform("dim2", dim)});

  if (messages.size() < 2 || messages.front() != "hello" ||
      messages.back() != "dim2") {
    std::fprintf(stderr, "contrast recovery failed:");
    for (const std::string &message : messages) {
      std::fprintf(stderr, " '%s'", message.c_str());
    }
    std::fprintf(stderr, "\n");
    std::abort();
  }
}

void expect_message_after_long_uptime() {
  vlc_rx_init();

  constexpr uint32_t kSignedQ8OverflowSamples = 0x80000000ULL / 256U;
  for (uint32_t i = 0; i < kSignedQ8OverflowSamples + 512U; ++i) {
    vlc_rx_push_sample(3000);
  }

  std::vector<uint16_t> waveform = make_waveform("late");
  for (uint16_t sample : waveform) {
    vlc_rx_push_sample(sample);
  }

  RxMessage rx_message;
  if (!vlc_rx_pop_message(&rx_message)) {
    std::fprintf(stderr, "long uptime decode failed: no message\n");
    std::abort();
  }

  std::string decoded(reinterpret_cast<const char *>(rx_message.data),
                      rx_message.len);
  if (decoded != "late") {
    std::fprintf(stderr, "long uptime decode failed: got '%s'\n",
                 decoded.c_str());
    std::abort();
  }
}

}  // namespace

int main() {
  expect_message("hello");
  expect_message("dmd");
  expect_message("vlc");

  WaveformOptions inverted;
  inverted.invert = true;
  expect_message("hello", inverted);

  WaveformOptions slow_tx;
  slow_tx.half_samples = 8.25;
  expect_message("hello", slow_tx);

  WaveformOptions fast_tx;
  fast_tx.half_samples = 7.75;
  expect_message("vlc", fast_tx);

  WaveformOptions drifting;
  drifting.drift_per_sample_times_1024 = 1;
  expect_message("dmd", drifting);

  WaveformOptions corrupt;
  corrupt.corrupt_payload_bit = true;
  expect_no_message("hello", corrupt);

  expect_contrast_drop_recovery();
  expect_message_after_long_uptime();

  std::puts("sim_rx: all tests passed");
  return 0;
}
