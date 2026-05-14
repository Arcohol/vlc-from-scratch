#include "ManchesterDecoder.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

using receiver::ManchesterDecoder;
using receiver::Message;

std::vector<uint8_t> buildFrame(const std::vector<uint8_t> &payload,
                                bool corrupt_crc = false) {
  std::vector<uint8_t> frame;
  frame.push_back(0xAA);
  frame.push_back(0xAA);
  frame.push_back(0xAA);
  frame.push_back(0xAA);
  frame.push_back(0xD5);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  std::vector<uint8_t> crc_input;
  crc_input.push_back(static_cast<uint8_t>(payload.size()));
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());
  uint16_t crc = receiver::crc16CcittFalse(crc_input.data(), crc_input.size());
  if (corrupt_crc) {
    crc ^= 0x0001;
  }
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  return frame;
}

std::vector<uint8_t> frameBits(const std::vector<uint8_t> &frame) {
  std::vector<uint8_t> bits;
  for (uint8_t byte : frame) {
    for (int bit = 7; bit >= 0; --bit) {
      bits.push_back(static_cast<uint8_t>((byte >> bit) & 1));
    }
  }
  return bits;
}

struct WaveformOptions {
  double half_samples = 8.0;
  uint16_t low_level = 420;
  uint16_t high_level = 780;
  double drift_per_sample = 0.0;
  bool corrupt_crc = false;
};

void appendSample(std::vector<uint16_t> *samples, uint16_t level,
                  const WaveformOptions &options) {
  double value =
      static_cast<double>(level) +
      options.drift_per_sample * static_cast<double>(samples->size());
  if (value < 0.0) {
    value = 0.0;
  } else if (value > 4095.0) {
    value = 4095.0;
  }
  samples->push_back(static_cast<uint16_t>(value + 0.5));
}

void appendLevel(std::vector<uint16_t> *samples, uint16_t level, uint32_t count,
                 const WaveformOptions &options) {
  for (uint32_t i = 0; i < count; ++i) {
    appendSample(samples, level, options);
  }
}

void appendHalf(std::vector<uint16_t> *samples, bool high, double *time,
                const WaveformOptions &options) {
  const double next = *time + options.half_samples;
  const uint32_t start = static_cast<uint32_t>(*time + 0.5);
  const uint32_t end = static_cast<uint32_t>(next + 0.5);
  const uint32_t count = end > start ? end - start : 1;
  appendLevel(samples, high ? options.high_level : options.low_level, count,
              options);
  *time = next;
}

std::vector<uint16_t> encodeSamples(const std::vector<uint8_t> &payload,
                                    const WaveformOptions &options = {}) {
  const std::vector<uint8_t> bits =
      frameBits(buildFrame(payload, options.corrupt_crc));

  std::vector<uint16_t> samples;
  appendLevel(&samples, options.high_level, 96, options);

  double position = static_cast<double>(samples.size());
  for (uint8_t bit : bits) {
    if (bit == 0) {
      appendHalf(&samples, true, &position, options);
      appendHalf(&samples, false, &position, options);
    } else {
      appendHalf(&samples, false, &position, options);
      appendHalf(&samples, true, &position, options);
    }
  }

  appendLevel(&samples, options.high_level, 160, options);
  return samples;
}

std::vector<Message> decodeAll(const std::vector<uint16_t> &samples) {
  ManchesterDecoder decoder;
  std::vector<Message> messages;
  for (uint16_t sample : samples) {
    Message message;
    if (decoder.pushSample(sample, &message)) {
      messages.push_back(message);
    }
  }
  return messages;
}

std::vector<Message>
decodeStream(const std::vector<std::vector<uint16_t>> &waveforms) {
  ManchesterDecoder decoder;
  std::vector<Message> messages;
  for (const std::vector<uint16_t> &waveform : waveforms) {
    for (uint16_t sample : waveform) {
      Message message;
      if (decoder.pushSample(sample, &message)) {
        messages.push_back(message);
      }
    }
  }
  return messages;
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void requirePayload(const Message &message,
                    const std::vector<uint8_t> &payload) {
  require(message.length == payload.size(), "decoded length mismatch");
  for (size_t i = 0; i < payload.size(); ++i) {
    require(message.payload[i] == payload[i], "decoded payload byte mismatch");
  }
}

void testPrintablePayload() {
  const std::vector<uint8_t> payload = {'h', 'e', 'l', 'l', 'o'};
  const std::vector<Message> messages = decodeAll(encodeSamples(payload));
  require(messages.size() == 1, "printable payload should decode once");
  requirePayload(messages[0], payload);
}

void testEmptyPayload() {
  const std::vector<uint8_t> payload;
  const std::vector<Message> messages = decodeAll(encodeSamples(payload));
  require(messages.size() == 1, "empty payload should decode once");
  requirePayload(messages[0], payload);
}

void testMaxPayload() {
  std::vector<uint8_t> payload(255);
  for (size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>(i);
  }
  const std::vector<Message> messages = decodeAll(encodeSamples(payload));
  require(messages.size() == 1, "255-byte payload should decode once");
  requirePayload(messages[0], payload);
}

void testBinaryPayload() {
  const std::vector<uint8_t> payload = {0x00, 0xFF, 0x10, 0x7E, 0x80};
  const std::vector<Message> messages = decodeAll(encodeSamples(payload));
  require(messages.size() == 1, "binary payload should decode once");
  requirePayload(messages[0], payload);
}

void testCrcReject() {
  const std::vector<uint8_t> payload = {'b', 'a', 'd'};
  WaveformOptions options;
  options.corrupt_crc = true;
  const std::vector<Message> messages =
      decodeAll(encodeSamples(payload, options));
  require(messages.empty(), "CRC-corrupt payload should not decode");
}

void testSlowThresholdDrift() {
  const std::vector<uint8_t> payload = {'d', 'r', 'i', 'f', 't'};
  WaveformOptions options;
  options.drift_per_sample = 0.02;
  const std::vector<Message> messages =
      decodeAll(encodeSamples(payload, options));
  require(messages.size() == 1, "slow ADC drift should decode once");
  requirePayload(messages[0], payload);
}

void testPhaseDrift() {
  const std::vector<uint8_t> payload = {'p', 'h', 'a', 's', 'e'};
  WaveformOptions options;
  options.half_samples = 8.03;
  const std::vector<Message> messages =
      decodeAll(encodeSamples(payload, options));
  require(messages.size() == 1, "fractional timing drift should decode once");
  requirePayload(messages[0], payload);
}

void testFastAndSlowTransmitters() {
  const std::vector<uint8_t> slow_payload = {'s', 'l', 'o', 'w'};
  WaveformOptions slow;
  slow.half_samples = 8.25;
  std::vector<Message> messages = decodeAll(encodeSamples(slow_payload, slow));
  require(messages.size() == 1, "slow transmitter should decode once");
  requirePayload(messages[0], slow_payload);

  const std::vector<uint8_t> fast_payload = {'f', 'a', 's', 't'};
  WaveformOptions fast;
  fast.half_samples = 7.75;
  messages = decodeAll(encodeSamples(fast_payload, fast));
  require(messages.size() == 1, "fast transmitter should decode once");
  requirePayload(messages[0], fast_payload);
}

void testLowContrastRecovery() {
  const std::vector<uint8_t> hello = {'h', 'e', 'l', 'l', 'o'};
  const std::vector<uint8_t> dim1 = {'d', 'i', 'm', '1'};
  const std::vector<uint8_t> dim2 = {'d', 'i', 'm', '2'};

  WaveformOptions dim;
  dim.low_level = 500;
  dim.high_level = 800;

  const std::vector<Message> messages =
      decodeStream({encodeSamples(hello), encodeSamples(dim1, dim),
                    encodeSamples(dim2, dim)});
  require(messages.size() >= 2, "low-contrast stream should recover");
  requirePayload(messages.front(), hello);
  requirePayload(messages.back(), dim2);
}

void testMessageAfterLongUptime() {
  ManchesterDecoder decoder;
  const uint32_t samples_before_message = 0x80000000ULL / 256U + 512U;
  for (uint32_t i = 0; i < samples_before_message; ++i) {
    Message ignored;
    decoder.pushSample(780, &ignored);
  }

  const std::vector<uint8_t> payload = {'l', 'a', 't', 'e'};
  bool decoded = false;
  Message message;
  for (uint16_t sample : encodeSamples(payload)) {
    if (decoder.pushSample(sample, &message)) {
      decoded = true;
      break;
    }
  }

  require(decoded, "message after long uptime should decode");
  requirePayload(message, payload);
}

} // namespace

int main() {
  testPrintablePayload();
  testEmptyPayload();
  testMaxPayload();
  testBinaryPayload();
  testCrcReject();
  testSlowThresholdDrift();
  testPhaseDrift();
  testFastAndSlowTransmitters();
  testLowContrastRecovery();
  testMessageAfterLongUptime();

  std::cout << "receiver_tests: all tests passed\n";
  return 0;
}
