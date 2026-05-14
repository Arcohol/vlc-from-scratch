#include "ManchesterDecoder.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
  receiver::DecoderConfig config;
  uint32_t samples_per_data_bit = 16;
  bool half_bit_override = false;
  bool sample_rate_override = false;
  bool summary_only = false;
  std::vector<std::string> inputs;
};

double parsePositiveDouble(const std::string &text, const std::string &name) {
  char *end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || value <= 0.0) {
    throw std::runtime_error("invalid " + name + ": " + text);
  }
  return value;
}

uint32_t parsePositiveUint(const std::string &text, const std::string &name) {
  char *end = nullptr;
  const unsigned long value = std::strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0' || value == 0) {
    throw std::runtime_error("invalid " + name + ": " + text);
  }
  return static_cast<uint32_t>(value);
}

double periodToMicroseconds(double value, const std::string &unit) {
  if (unit == "ns") {
    return value / 1000.0;
  }
  if (unit == "us" || unit == "µs" || unit == "μs") {
    return value;
  }
  if (unit == "ms") {
    return value * 1000.0;
  }
  if (unit == "s") {
    return value * 1000000.0;
  }
  throw std::runtime_error("unsupported period unit: " + unit);
}

uint32_t parseRate(const std::string &text) {
  static const std::regex rate_re(R"(^([0-9]+(?:\.[0-9]+)?)(hz|k|khz|m|mhz)?$)",
                                  std::regex::icase);
  std::smatch match;
  if (!std::regex_match(text, match, rate_re)) {
    throw std::runtime_error("invalid sample rate: " + text);
  }

  double rate = parsePositiveDouble(match[1].str(), "sample rate");
  std::string unit = match[2].matched ? match[2].str() : "hz";
  std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (unit == "k" || unit == "khz") {
    rate *= 1000.0;
  } else if (unit == "m" || unit == "mhz") {
    rate *= 1000000.0;
  }
  return static_cast<uint32_t>(rate + 0.5);
}

bool inferTimingFromName(const std::string &path, Options *options) {
  bool inferred_any = false;

  if (!options->half_bit_override) {
    static const std::regex period_re(
        R"(^([0-9]+(?:\.[0-9]+)?)(ns|us|µs|μs|ms|s)(?:[_-]|$))",
        std::regex::icase);
    std::smatch match;
    const std::string name = path.substr(path.find_last_of("/\\") + 1);
    if (std::regex_search(name, match, period_re)) {
      const double full_bit_us = periodToMicroseconds(
          parsePositiveDouble(match[1].str(), "period"), match[2].str());
      options->config.half_bit_us =
          static_cast<uint32_t>(full_bit_us / 2.0 + 0.5);
      inferred_any = true;

      if (!options->sample_rate_override) {
        const double rate = static_cast<double>(options->samples_per_data_bit) *
                            1000000.0 / full_bit_us;
        options->config.sample_rate_hz = static_cast<uint32_t>(rate + 0.5);
      }
    }
  }

  if (!options->sample_rate_override) {
    static const std::regex rate_re(
        R"(_([0-9]+(?:p[0-9]+)?)(Hz|kHz|MHz)(?:_part[0-9]+)?\.raw$)",
        std::regex::icase);
    std::smatch match;
    const std::string name = path.substr(path.find_last_of("/\\") + 1);
    if (std::regex_search(name, match, rate_re)) {
      std::string value = match[1].str();
      std::replace(value.begin(), value.end(), 'p', '.');
      options->config.sample_rate_hz = parseRate(value + match[2].str());
      inferred_any = true;
    }
  }

  return inferred_any;
}

Options parseArgs(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto requireValue = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(name + " requires a value");
      }
      return argv[++i];
    };

    if (arg == "--half-bit-us") {
      options.config.half_bit_us =
          parsePositiveUint(requireValue(arg), "half-bit period");
      options.half_bit_override = true;
    } else if (arg == "--sample-rate") {
      options.config.sample_rate_hz = parseRate(requireValue(arg));
      options.sample_rate_override = true;
    } else if (arg == "--samples-per-data-bit") {
      options.samples_per_data_bit =
          parsePositiveUint(requireValue(arg), "samples per data bit");
    } else if (arg == "--summary-only") {
      options.summary_only = true;
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: receiver_sim [options] sample.raw...\n"
                << "  --half-bit-us N            Manchester half-bit period\n"
                << "  --sample-rate RATE         ADC rate, e.g. 200kHz\n"
                << "  --samples-per-data-bit N   used for filename inference\n"
                << "  --summary-only             count messages without "
                   "printing them\n";
      std::exit(0);
    } else if (!arg.empty() && arg[0] == '-') {
      throw std::runtime_error("unknown option: " + arg);
    } else {
      options.inputs.push_back(arg);
    }
  }

  if (options.inputs.empty()) {
    throw std::runtime_error("at least one input file is required");
  }

  std::sort(options.inputs.begin(), options.inputs.end());
  inferTimingFromName(options.inputs.front(), &options);
  return options;
}

std::string asciiPayload(const receiver::Message &message) {
  std::ostringstream out;
  for (uint16_t i = 0; i < message.length; ++i) {
    const uint8_t byte = message.payload[i];
    switch (byte) {
    case '\\':
      out << "\\\\";
      break;
    case '"':
      out << "\\\"";
      break;
    case '\n':
      out << "\\n";
      break;
    case '\r':
      out << "\\r";
      break;
    case '\t':
      out << "\\t";
      break;
    default:
      if (byte >= 32 && byte <= 126) {
        out << static_cast<char>(byte);
      } else {
        out << "\\x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(byte) << std::dec;
      }
      break;
    }
  }
  return out.str();
}

std::string hexPayload(const receiver::Message &message) {
  std::ostringstream out;
  for (uint16_t i = 0; i < message.length; ++i) {
    if (i != 0) {
      out << ' ';
    }
    out << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(message.payload[i]) << std::dec;
  }
  return out.str();
}

void printMessage(const receiver::Message &message, uint64_t sample_index) {
  std::cout << "message sample=" << sample_index
            << " length=" << static_cast<unsigned>(message.length) << " crc=0x"
            << std::hex << std::setw(4) << std::setfill('0') << message.crc
            << std::dec << " ascii=\"" << asciiPayload(message) << "\" hex=["
            << hexPayload(message) << "]\n";
}

void printStats(const receiver::DecoderStats &stats) {
  std::cerr << "decoder stats:"
            << " messages=" << stats.messages
            << " crc_failures=" << stats.crc_failures
            << " sfd_timeouts=" << stats.sfd_timeouts
            << " weak_bits=" << stats.weak_bits
            << " lost_center_edges=" << stats.lost_center_edges
            << " length_errors=" << stats.length_errors
            << " signal_swing=" << stats.signal_swing
            << " contrast=" << stats.contrast << '\n';
}

void processFile(const std::string &path, receiver::ManchesterDecoder *decoder,
                 bool summary_only, uint64_t *sample_index,
                 uint32_t *message_count) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open " + path);
  }

  std::string line;
  uint32_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }

    char *end = nullptr;
    const long value = std::strtol(line.c_str(), &end, 10);
    if (end == line.c_str() || *end != '\0' || value < 0 || value > 65535) {
      std::ostringstream error;
      error << path << ':' << line_number << ": expected uint16 ADC sample";
      throw std::runtime_error(error.str());
    }

    receiver::Message message;
    if (decoder->pushSample(static_cast<uint16_t>(value), &message)) {
      ++(*message_count);
      if (!summary_only) {
        printMessage(message, *sample_index);
      }
    }
    ++(*sample_index);
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    Options options = parseArgs(argc, argv);
    receiver::ManchesterDecoder decoder(options.config);

    std::cerr << "half_bit_us=" << options.config.half_bit_us
              << " sample_rate_hz=" << options.config.sample_rate_hz
              << " half_bit_samples="
              << static_cast<unsigned>(decoder.halfBitSamples()) << '\n';

    uint64_t sample_index = 0;
    uint32_t message_count = 0;
    for (const std::string &path : options.inputs) {
      processFile(path, &decoder, options.summary_only, &sample_index,
                  &message_count);
    }

    std::cout.flush();
    std::cerr << "processed " << sample_index << " samples, decoded "
              << message_count << " message(s)\n";
    if (options.summary_only) {
      printStats(decoder.stats());
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "receiver_sim: " << error.what() << '\n';
    return 1;
  }
}
