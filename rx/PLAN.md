# Arduino Due Manchester Receiver

## Summary
Build a shared portable C++ receiver library, a CMake-built native simulator/test harness, and an Arduino Due sketch that reuses the same decoder source. Defaults: `half_bit_us = 40`, `samples_per_data_bit = 16`, derived `sample_rate_hz = 200000`.

## Key Changes
- Add a CMake project with targets:
  - `receiver`: portable C++ decoder library.
  - `receiver_sim`: CLI simulator for captured `.raw` sample files.
  - `receiver_tests`: native unit tests with synthetic frames.
- Use neutral names, not `Vlc*` prefixes:
  - `DecoderConfig { half_bit_us, sample_rate_hz }`
  - `Message { length, payload[255], crc }`
  - `ManchesterDecoder::pushSample(uint16_t sample, Message* out)`
- Decoder behavior:
  - Maintain adaptive integer threshold from raw ADC samples.
  - Quantize each sample once; decoding uses quantized values.
  - Lock from equal-width preamble edges, then validate SFD `0xD5`.
  - Decode each bit from two center samples, one per Manchester half, with ±1 sample phase correction.
  - Implement only normal polarity: raw `0 = high, low`, raw `1 = low, high`.
  - Parse `length`, payload, and CRC-16/CCITT-FALSE over `length + payload`; emit only CRC-valid messages.
- Code comments:
  - Add concise comments around the decoder state machine, adaptive threshold update, phase correction, CRC check, and ADC DMA setup.
  - Avoid noisy line-by-line comments for obvious assignments or simple control flow.
- Simulator behavior:
  - Reads newline-delimited ADC integer samples from one or more files.
  - Infers `80us` full bit period and `200kHz` sample rate from filenames, with CLI overrides.
  - Processes multiple sorted input files as one continuous stream by default.
  - Prints decoded payload as escaped ASCII plus hex.
- Arduino Due sketch:
  - Uses A0, mapped to SAM ADC channel 7.
  - Configures TC0/TIOA0 to trigger ADC at derived 200 kHz.
  - Uses ADC PDC/DMA with `uint16_t` buffers; `ADC_Handler` queues completed buffers, `loop()` feeds samples to `ManchesterDecoder`.
  - Prints decoded messages over `SerialUSB` on the Native USB Port.

## Test Plan
- CMake:
  - `cmake -S . -B build`
  - `cmake --build build`
  - `ctest --test-dir build`
- Unit tests cover valid frames, empty payload, max 255-byte payload, binary payload, CRC rejection, slow threshold drift, and ±1 sample timing drift.
- Run simulator on `data/processed/80us_20260513_171325_200kHz_part*.raw` and verify CRC-valid messages print.
- Compile Arduino sketch with `arduino-cli compile --fqbn arduino:sam:arduino_due_x --libraries lib arduino/receiver_due`.

## Assumptions
- ADC higher numeric value represents Manchester “high”; no inverted-polarity handling will be implemented.
- Processed `.raw` files are newline-delimited ADC sample integers.
- Filename `80us_..._200kHz...` means 80 us full raw data-bit period, so 40 us Manchester half-bit.
- The Due target is the Native USB Port, using `SerialUSB` and FQBN `arduino:sam:arduino_due_x`.
