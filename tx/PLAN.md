# VLC Transmitter Implementation Plan

## Summary
Implement a TX-only visible-light communication stack for the DMD transmitter. Applications submit payloads through a copy-on-send API; the TX module queues them, builds protocol frames, Manchester encodes raw bits, and emits fixed-period optical chips through a non-blocking DMD SPIM driver. Idle output stays optical high.

## Protocol And API
- Raw frame format before Manchester encoding:
  ```text
  AA AA AA AA | D5 | length | payload... | CRC_H CRC_L
  ```
- Preamble is four bytes: `0xAA 0xAA 0xAA 0xAA`.
- SFD is `0xD5`.
- Length is one byte, allowing payloads from `0` to `255` bytes.
- CRC is CRC-16/CCITT-FALSE over `length + payload`, appended MSB first.
- Raw frame bits are serialized MSB first.
- Manchester mapping: raw `0 = high, low`; raw `1 = low, high`.
- Add `src/vlc_tx.h`:
  ```c
  int vlc_tx_init(void);
  int vlc_tx_send(const uint8_t *buf, uint32_t len, k_timeout_t timeout);
  int vlc_tx_get_error(void);
  ```
- `vlc_tx_send()` copies payloads into an internal queue. `timeout` controls how long to wait for queue space when the queue is full.

## Implementation
- Add `src/vlc_tx.c`/`.h` and include them in `CMakeLists.txt`.
- Convert `dmd_control` SPIM output to non-blocking EasyDMA:
  ```c
  int dmd_send_bit_async(bool bit);
  bool dmd_control_ready(void);
  ```
- Initialize SPIM4 with an `nrfx_spim_event_handler_t`, connect the SPIM4 IRQ, and store high/low 14-byte DMD waveforms in stable RAM buffers.
- Timer ISR fires every `100 us` Manchester half-bit. It schedules the next compare, selects the next optical chip, and calls `dmd_send_bit_async()` without blocking.
- If SPIM is still busy or returns an error, record TX failure and stop the timer.
- When no frame is active, continuously emit high.
- Use a fixed `k_msgq` for copied payload records and a staging thread to build the next raw frame and CRC outside the ISR.
- Use double-buffered frame storage so one prepared frame can transmit while the staging thread prepares the next.
- Set staging above normal app/UART worker threads but below timer/SPIM IRQs.

## Demo Main
- Replace the current alternating pattern with `vlc_tx_init()`.
- Constantly send these three short messages in rotation:
  ```text
  hello
  dmd
  vlc
  ```
- Use `vlc_tx_send(..., K_FOREVER)` so the demo loop backpressures on the TX queue instead of dropping frames.

## Defaults
- `VLC_TX_MAX_PAYLOAD_LEN = 255`
- `VLC_TX_QUEUE_DEPTH = 4`
- `VLC_TX_HALF_BIT_US = 100`
- `VLC_TX_TIMER_INSTANCE = 2`
- `VLC_TX_TIMER_IRQ_PRIORITY = 1`
- `DMD_SPIM_IRQ_PRIORITY = 1`
- `VLC_TX_STAGING_THREAD_PRIORITY = 1`
- Normal app/UART worker threads should use numerically larger, lower Zephyr priorities than staging.

## Test Plan
- Build with:
  ```sh
  nix develop --command west build -b nrf5340dk/nrf5340/cpuapp .
  ```
- Validate frame construction for empty, short, and 255-byte payloads.
- Validate `AA AA AA AA` preamble, `D5` SFD, length byte, CRC bytes, MSB-first bit order, and Manchester mapping.
- Validate `vlc_tx_send()` errors for null buffer, oversized payload, full queue timeout, and failed transmitter state.
- On hardware, verify idle-high output, 100 us half-bit timing, non-blocking SPIM behavior, and continuous transmission of `hello`, `dmd`, and `vlc` without underruns.

## Assumptions
- First iteration is transmitter-only; UART receive integration is deferred.
- `AAAAAAAA` means four preamble bytes of `0xAA`.
- Each 14-byte DMD SPIM transfer completes within the 100 us Manchester half-bit budget.
