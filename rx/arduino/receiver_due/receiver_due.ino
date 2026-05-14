#include <Arduino.h>
#include <ManchesterDecoder.h>

using receiver::DecoderConfig;
using receiver::ManchesterDecoder;
using receiver::Message;

static const uint32_t kHalfBitUs = 40;
static const uint32_t kSamplesPerDataBit = 16;
static const uint32_t kSampleRateHz =
    (1000000UL * kSamplesPerDataBit) / (2UL * kHalfBitUs);
static const uint16_t kAdcBufferSamples = 512;
static const uint8_t kAdcBufferCount = 4;
static const uint8_t kAdcChannel = ADC_CHANNEL_7; // Arduino Due A0.

static uint16_t adc_buffers[kAdcBufferCount][kAdcBufferSamples]
    __attribute__((aligned(4)));
static volatile uint8_t dma_current_buffer = 0;
static volatile uint8_t dma_next_buffer = 1;
static volatile bool buffer_ready[kAdcBufferCount];
static volatile bool buffer_busy[kAdcBufferCount];
static volatile uint8_t ready_queue[kAdcBufferCount];
static volatile uint8_t ready_head = 0;
static volatile uint8_t ready_tail = 0;
static volatile uint8_t ready_count = 0;
static volatile bool dma_overflow = false;
static volatile bool adc_overrun = false;

static DecoderConfig decoder_config;
static ManchesterDecoder decoder(decoder_config);

static void printHexByte(uint8_t value) {
  if (value < 16) {
    SerialUSB.print('0');
  }
  SerialUSB.print(value, HEX);
}

static void printEscapedAsciiByte(uint8_t value) {
  switch (value) {
  case '\\':
    SerialUSB.print("\\\\");
    break;
  case '"':
    SerialUSB.print("\\\"");
    break;
  case '\n':
    SerialUSB.print("\\n");
    break;
  case '\r':
    SerialUSB.print("\\r");
    break;
  case '\t':
    SerialUSB.print("\\t");
    break;
  default:
    if (value >= 32 && value <= 126) {
      SerialUSB.write(value);
    } else {
      SerialUSB.print("\\x");
      printHexByte(value);
    }
    break;
  }
}

static void printMessage(const Message &message) {
  SerialUSB.print("message length=");
  SerialUSB.print(message.length);
  SerialUSB.print(" crc=0x");
  printHexByte(static_cast<uint8_t>(message.crc >> 8));
  printHexByte(static_cast<uint8_t>(message.crc & 0xFF));
  SerialUSB.print(" ascii=\"");
  for (uint16_t i = 0; i < message.length; ++i) {
    printEscapedAsciiByte(message.payload[i]);
  }
  SerialUSB.print("\" hex=[");
  for (uint16_t i = 0; i < message.length; ++i) {
    if (i != 0) {
      SerialUSB.print(' ');
    }
    printHexByte(message.payload[i]);
  }
  SerialUSB.println(']');
}

static bool queueReadyBuffer(uint8_t buffer_index) {
  if (ready_count >= kAdcBufferCount) {
    dma_overflow = true;
    return false;
  }
  ready_queue[ready_tail] = buffer_index;
  ready_tail = (ready_tail + 1) % kAdcBufferCount;
  ++ready_count;
  buffer_ready[buffer_index] = true;
  return true;
}

static int8_t findFreeBuffer() {
  for (uint8_t i = 0; i < kAdcBufferCount; ++i) {
    if (i == dma_current_buffer || i == dma_next_buffer) {
      continue;
    }
    if (!buffer_ready[i] && !buffer_busy[i]) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

static void armNextDmaBuffer(uint8_t buffer_index) {
  dma_next_buffer = buffer_index;
  ADC->ADC_RNPR = reinterpret_cast<uint32_t>(adc_buffers[buffer_index]);
  ADC->ADC_RNCR = kAdcBufferSamples;
}

static void handleDmaBufferComplete() {
  const uint8_t finished = dma_current_buffer;
  dma_current_buffer = dma_next_buffer;
  dma_next_buffer = 0xFF;

  const bool queued_finished = queueReadyBuffer(finished);

  int8_t refill = findFreeBuffer();
  if (refill < 0) {
    // Keep the ADC running, but flag the loss. Once this happens, at least one
    // frame will be invalid because samples have been overwritten or skipped.
    dma_overflow = true;
    if (queued_finished) {
      ready_tail = (ready_tail + kAdcBufferCount - 1) % kAdcBufferCount;
      --ready_count;
      buffer_ready[finished] = false;
    }
    refill = finished;
  }
  armNextDmaBuffer(static_cast<uint8_t>(refill));
}

void ADC_Handler() {
  const uint32_t status = ADC->ADC_ISR;
  if ((status & ADC_ISR_ENDRX) != 0) {
    handleDmaBufferComplete();
  }
  if ((status & ADC_ISR_GOVRE) != 0) {
    adc_overrun = true;
  }
}

static bool popReadyBuffer(uint8_t *buffer_index) {
  noInterrupts();
  if (ready_count == 0) {
    interrupts();
    return false;
  }
  const uint8_t index = ready_queue[ready_head];
  ready_head = (ready_head + 1) % kAdcBufferCount;
  --ready_count;
  buffer_ready[index] = false;
  buffer_busy[index] = true;
  interrupts();

  *buffer_index = index;
  return true;
}

static void releaseBuffer(uint8_t buffer_index) {
  noInterrupts();
  buffer_busy[buffer_index] = false;
  interrupts();
}

static void configureTimerTrigger() {
  pmc_enable_periph_clk(ID_TC0);

  const uint32_t timer_clock_hz = VARIANT_MCK / 2UL; // TIMER_CLOCK1.
  const uint32_t rc = timer_clock_hz / kSampleRateHz;

  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE |
                   TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR |
                   TC_CMR_ACPC_SET);
  TC_SetRA(TC0, 0, rc / 2);
  TC_SetRC(TC0, 0, rc);
  TC_Start(TC0, 0);
}

static void configureAdcDma() {
  pmc_enable_periph_clk(ID_ADC);

  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_set_resolution(ADC, ADC_12_BITS);
  adc_disable_all_channel(ADC);
  adc_enable_channel(ADC, static_cast<adc_channel_num_t>(kAdcChannel));

  // The timer supplies the exact sample cadence. PDC/DMA moves ADC results into
  // RAM so loop() can spend its time decoding and printing complete messages.
  adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
  adc_disable_interrupt(ADC, 0xFFFFFFFF);

  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
  ADC->ADC_RPR = reinterpret_cast<uint32_t>(adc_buffers[0]);
  ADC->ADC_RCR = kAdcBufferSamples;
  ADC->ADC_RNPR = reinterpret_cast<uint32_t>(adc_buffers[1]);
  ADC->ADC_RNCR = kAdcBufferSamples;
  ADC->ADC_IER = ADC_IER_ENDRX | ADC_IER_GOVRE;
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;
}

void setup() {
  SerialUSB.begin(115200);

  decoder_config.half_bit_us = kHalfBitUs;
  decoder_config.sample_rate_hz = kSampleRateHz;
  decoder = ManchesterDecoder(decoder_config);

  for (uint8_t i = 0; i < kAdcBufferCount; ++i) {
    buffer_ready[i] = false;
    buffer_busy[i] = false;
  }

  configureAdcDma();
  configureTimerTrigger();

  SerialUSB.print("receiver ready sample_rate_hz=");
  SerialUSB.print(kSampleRateHz);
  SerialUSB.print(" half_bit_us=");
  SerialUSB.println(kHalfBitUs);
}

void loop() {
  uint8_t buffer_index = 0;
  while (popReadyBuffer(&buffer_index)) {
    for (uint16_t i = 0; i < kAdcBufferSamples; ++i) {
      Message message;
      if (decoder.pushSample(adc_buffers[buffer_index][i], &message)) {
        printMessage(message);
      }
    }
    releaseBuffer(buffer_index);
  }

  if (dma_overflow || adc_overrun) {
    noInterrupts();
    const bool had_dma_overflow = dma_overflow;
    const bool had_adc_overrun = adc_overrun;
    dma_overflow = false;
    adc_overrun = false;
    interrupts();

    SerialUSB.print("warning");
    if (had_dma_overflow) {
      SerialUSB.print(" dma_overflow");
    }
    if (had_adc_overrun) {
      SerialUSB.print(" adc_overrun");
    }
    SerialUSB.println();
  }
}
