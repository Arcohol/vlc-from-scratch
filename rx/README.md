# Arduino Due Manchester Receiver

Portable Manchester receiver code for a visible light communication link, plus
a native simulator/test harness. The Arduino Due sketch uses the reference
ADC/PDC sampler with the shared `ManchesterDecoder` library.

The decoder consumes one ADC sample at a time. It locks to the `0xAA` preamble,
tracks slow light-level drift with adaptive high/low envelopes, uses a small
fixed-point PLL for Manchester timing, validates `0xD5` as the SFD, and emits
only CRC-valid messages.

## Protocol

Raw frame before Manchester encoding:

```text
AA AA AA AA | D5 | length | payload... | CRC_H CRC_L
```

- Raw bits are serialized MSB first.
- CRC is CRC-16/CCITT-FALSE over `length + payload`, appended MSB first.
- Manchester mapping is normal polarity only:
  - raw `0` = high, low
  - raw `1` = low, high
- Default timing is a 40 us Manchester half-bit at 200 kHz ADC sample rate,
  giving 8 samples per half-bit and 16 samples per raw data bit.

## Native Build And Tests

With Nix:

```sh
nix develop
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Without Nix, install a C++ compiler and CMake 3.16 or newer, then run the same
`cmake` and `ctest` commands.

Useful direct commands:

```sh
build/receiver_sim --summary-only data/processed/*.raw
build/receiver_sim data/processed/80us_20260513_171325_200kHz_part0000.raw
```

`receiver_sim` reads newline-delimited unsigned ADC samples. It sorts input
paths and processes them as one continuous stream. It infers timing from names
like `80us_..._200kHz_part0000.raw`; use overrides when needed:

```sh
build/receiver_sim --half-bit-us 40 --sample-rate 200kHz samples.raw
```

## Arduino Due

The sketch targets an Arduino Due on the Native USB Port:

- Photodiode input: `A0`, SAM ADC channel 7.
- ADC trigger: TC0/TIOA0 at the derived 200 kHz sample rate.
- Sampling transport: ADC PDC/DMA into `uint16_t` buffers.
- Output: decoded payloads printed over `SerialUSB`, followed by CRLF.

Install the Arduino SAM core if needed:

```sh
arduino-cli core update-index
arduino-cli core install arduino:sam
```

Compile:

```sh
arduino-cli compile \
  --fqbn arduino:sam:arduino_due_x \
  --libraries lib \
  arduino/receiver_due
```

Find the board port:

```sh
arduino-cli board list
```

Upload, replacing the port with the Native USB Port shown by `board list`:

```sh
arduino-cli upload \
  -p /dev/ttyACM0 \
  --fqbn arduino:sam:arduino_due_x \
  arduino/receiver_due
```

Monitor decoded messages:

```sh
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## Decoder API

The portable entry point is:

```cpp
receiver::DecoderConfig config;
receiver::ManchesterDecoder decoder(config);
receiver::Message message;

decoder.pushSamples(samples, sample_count);
while (decoder.popMessage(&message)) {
  // message.payload[0..message.length) is CRC-valid.
}
```

For simple sample-at-a-time code, `pushSample(adc_sample, &message)` still
returns `true` when that sample completes a CRC-valid message.

`ManchesterDecoder::stats()` exposes counters for decoded messages, CRC
failures, queued-message drops, SFD timeouts, weak bits, lost center edges,
length errors, signal swing, and contrast. These are useful when tuning optics,
gain, or sampling timing.

## Assumptions

- Higher ADC values mean optical high.
- Inverted polarity is intentionally unsupported.
- Captured `.raw` files contain one integer ADC sample per line.
- The decoder is designed to avoid floating point in the sample path so it can
  run comfortably on the Arduino Due.
