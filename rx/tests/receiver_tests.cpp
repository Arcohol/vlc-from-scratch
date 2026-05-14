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

std::vector<uint16_t> encodeSamples(const std::vector<uint8_t> &payload,
                                    double half_samples = 8.0,
                                    double drift_per_sample = 0.0,
                                    bool corrupt_crc = false) {
  const std::vector<uint8_t> bits = frameBits(buildFrame(payload, corrupt_crc));
  std::vector<bool> levels;
  for (uint8_t bit : bits) {
    if (bit == 0) {
      levels.push_back(true);
      levels.push_back(false);
    } else {
      levels.push_back(false);
      levels.push_back(true);
    }
  }

  std::vector<uint16_t> samples;
  samples.insert(samples.end(), 96, 600);

  double position = static_cast<double>(samples.size());
  for (bool high : levels) {
    const double next_position = position + half_samples;
    const int end = static_cast<int>(std::floor(next_position + 0.5));
    while (static_cast<int>(samples.size()) < end) {
      const double baseline =
          600.0 + drift_per_sample * static_cast<double>(samples.size());
      const double value = high ? baseline + 180.0 : baseline - 180.0;
      samples.push_back(static_cast<uint16_t>(value + 0.5));
    }
    position = next_position;
  }

  samples.insert(samples.end(), 96, samples.empty() ? 600 : samples.back());
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
  const std::vector<Message> messages =
      decodeAll(encodeSamples(payload, 8.0, 0.0, true));
  require(messages.empty(), "CRC-corrupt payload should not decode");
}

void testSlowThresholdDrift() {
  const std::vector<uint8_t> payload = {'d', 'r', 'i', 'f', 't'};
  const std::vector<Message> messages =
      decodeAll(encodeSamples(payload, 8.0, 0.02));
  require(messages.size() == 1, "slow ADC drift should decode once");
  requirePayload(messages[0], payload);
}

void testPhaseDrift() {
  const std::vector<uint8_t> payload = {'p', 'h', 'a', 's', 'e'};
  const std::vector<Message> messages = decodeAll(encodeSamples(payload, 8.03));
  require(messages.size() == 1, "fractional timing drift should decode once");
  requirePayload(messages[0], payload);
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

  std::cout << "receiver_tests: all tests passed\n";
  return 0;
}
