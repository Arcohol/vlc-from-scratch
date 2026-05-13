#include "adc_sampler.h"

#include <Arduino.h>

#if !defined(__SAM3X8E__)
#error "adc_sampler.cpp targets the Arduino Due SAM3X8E ADC/PDC peripheral"
#endif

namespace {

constexpr uint8_t kAdcPin = A0;
constexpr uint8_t kBufferCount = 8;
constexpr uint8_t kQueueMask = kBufferCount - 1;
static_assert((kBufferCount & kQueueMask) == 0, "buffer count must be power of two");

#ifndef ADC_MR_TRGSEL_ADC_TRIG1
#define ADC_MR_TRGSEL_ADC_TRIG1 (1U << 1)
#endif

#ifndef ADC_MR_STARTUP_SUT64
#define ADC_MR_STARTUP_SUT64 (4U << 16)
#endif

#ifndef ADC_MR_SETTLING_AST3
#define ADC_MR_SETTLING_AST3 (0U << 20)
#endif

#ifndef ADC_PTCR_RXTEN
#define ADC_PTCR_RXTEN 1U
#endif

#ifndef ADC_PTCR_RXTDIS
#define ADC_PTCR_RXTDIS 2U
#endif

alignas(4) uint16_t sample_buffers[kBufferCount][ADC_SAMPLER_BLOCK_SAMPLES];

volatile uint8_t completed_queue[kBufferCount];
volatile uint8_t completed_head;
volatile uint8_t completed_tail;

volatile uint8_t free_queue[kBufferCount];
volatile uint8_t free_head;
volatile uint8_t free_tail;

volatile uint8_t dma_current;
volatile uint8_t dma_next;
volatile bool dma_has_next;
volatile bool dma_stalled;
volatile bool sampler_running;
volatile bool sampler_overrun;
volatile uint32_t sampler_overrun_count;

bool queue_empty(uint8_t head, uint8_t tail) { return head == tail; }

void note_sampler_overrun() {
  sampler_overrun = true;
  if (sampler_overrun_count < 0xFFFFFFFFUL) {
    ++sampler_overrun_count;
  }
}

bool completed_push_from_isr(uint8_t value) {
  uint8_t head = completed_head;
  uint8_t next = static_cast<uint8_t>((head + 1U) & kQueueMask);

  if (next == completed_tail) {
    return false;
  }

  completed_queue[head] = value;
  completed_head = next;
  return true;
}

bool free_pop_from_isr(uint8_t *value) {
  uint8_t tail = free_tail;

  if (queue_empty(free_head, tail)) {
    return false;
  }

  *value = free_queue[tail];
  free_tail = static_cast<uint8_t>((tail + 1U) & kQueueMask);
  return true;
}

bool free_push_from_main(uint8_t value) {
  uint8_t head = free_head;
  uint8_t next = static_cast<uint8_t>((head + 1U) & kQueueMask);

  if (next == free_tail) {
    return false;
  }

  free_queue[head] = value;
  free_head = next;
  return true;
}

void adc_set_current_buffer(uint8_t index) {
  ADC->ADC_RPR = reinterpret_cast<uint32_t>(sample_buffers[index]);
  ADC->ADC_RCR = ADC_SAMPLER_BLOCK_SAMPLES;
  dma_current = index;
  dma_stalled = false;
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;
}

void adc_set_next_buffer(uint8_t index) {
  ADC->ADC_RNPR = reinterpret_cast<uint32_t>(sample_buffers[index]);
  ADC->ADC_RNCR = ADC_SAMPLER_BLOCK_SAMPLES;
  dma_next = index;
  dma_has_next = true;
}

void adc_prime_next_if_possible() {
  if (!sampler_running) {
    return;
  }

  uint8_t replacement;

  if (dma_stalled) {
    if (!free_pop_from_isr(&replacement)) {
      return;
    }

    adc_set_current_buffer(replacement);
  }

  if (dma_has_next) {
    return;
  }

  if (free_pop_from_isr(&replacement)) {
    adc_set_next_buffer(replacement);
  }
}

uint32_t adc_channel_for_pin(uint8_t pin) {
  return g_APinDescription[pin].ulADCChannelNumber;
}

void configure_timer_trigger() {
  pmc_enable_periph_clk(ID_TC0);

  TC_Stop(TC0, 0);
  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE |
                   TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR |
                   TC_CMR_ACPC_SET);

  uint32_t rc = VARIANT_MCK / 2U / VLC_RX_SAMPLE_RATE_HZ;
  if (rc < 2U) {
    rc = 2U;
  }

  TC_SetRA(TC0, 0, rc / 2U);
  TC_SetRC(TC0, 0, rc);
  TC_Start(TC0, 0);
}

void configure_adc(uint32_t channel) {
  pmc_enable_periph_clk(ID_ADC);

  ADC->ADC_CR = ADC_CR_SWRST;
  ADC->ADC_IDR = 0xFFFFFFFFU;
  ADC->ADC_CHDR = 0xFFFFFFFFU;

  ADC->ADC_MR =
      ADC_MR_TRGEN_EN | ADC_MR_TRGSEL_ADC_TRIG1 | ADC_MR_PRESCAL(1) |
      ADC_MR_STARTUP_SUT64 | ADC_MR_SETTLING_AST3 | ADC_MR_TRACKTIM(3) |
      ADC_MR_TRANSFER(1);

  ADC->ADC_EMR = 0;
  ADC->ADC_CGR = 0;
  ADC->ADC_COR = 0;
  ADC->ADC_CHER = 1UL << channel;
}

void start_pdc() {
  dma_current = 0;
  dma_next = 1;
  dma_has_next = true;
  dma_stalled = false;

  completed_head = completed_tail = 0;
  free_head = free_tail = 0;

  for (uint8_t i = 2; i < kBufferCount; ++i) {
    (void)free_push_from_main(i);
  }

  ADC->ADC_RPR = reinterpret_cast<uint32_t>(sample_buffers[dma_current]);
  ADC->ADC_RCR = ADC_SAMPLER_BLOCK_SAMPLES;
  ADC->ADC_RNPR = reinterpret_cast<uint32_t>(sample_buffers[dma_next]);
  ADC->ADC_RNCR = ADC_SAMPLER_BLOCK_SAMPLES;

  ADC->ADC_IER = ADC_IER_ENDRX;
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;
}

}  // namespace

bool adc_sampler_begin() {
  if (sampler_running) {
    return true;
  }

  uint32_t channel = adc_channel_for_pin(kAdcPin);

  if (channel > 15U) {
    return false;
  }

  sampler_overrun = false;
  sampler_overrun_count = 0;
  configure_timer_trigger();
  configure_adc(channel);
  sampler_running = true;
  start_pdc();

  return true;
}

void adc_sampler_stop() {
  if (!sampler_running) {
    return;
  }

  TC_Stop(TC0, 0);
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS;
  ADC->ADC_IDR = ADC_IDR_ENDRX;
  NVIC_DisableIRQ(ADC_IRQn);
  ADC->ADC_CHDR = 0xFFFFFFFFU;

  sampler_running = false;
}

bool adc_sampler_pop_completed_block(AdcBlock *block) {
  if (block == nullptr) {
    return false;
  }

  uint8_t tail = completed_tail;

  if (queue_empty(completed_head, tail)) {
    return false;
  }

  uint8_t index = completed_queue[tail];
  completed_tail = static_cast<uint8_t>((tail + 1U) & kQueueMask);

  block->samples = sample_buffers[index];
  block->len = ADC_SAMPLER_BLOCK_SAMPLES;
  block->index = index;
  return true;
}

void adc_sampler_release_block(const AdcBlock *block) {
  if (block == nullptr || block->index >= kBufferCount) {
    return;
  }

  noInterrupts();
  bool queued = free_push_from_main(block->index);
  adc_prime_next_if_possible();
  if (!queued) {
    note_sampler_overrun();
  }
  interrupts();
}

bool adc_sampler_take_overrun() {
  noInterrupts();
  bool value = sampler_overrun;
  sampler_overrun = false;
  interrupts();
  return value;
}

uint32_t adc_sampler_get_overrun_count() {
  noInterrupts();
  uint32_t value = sampler_overrun_count;
  interrupts();
  return value;
}

void ADC_Handler() {
  uint32_t status = ADC->ADC_ISR;

  if ((status & ADC_ISR_ENDRX) == 0U) {
    return;
  }

  uint8_t completed = dma_current;
  bool overrun = false;

  if (dma_has_next) {
    dma_current = dma_next;
    dma_has_next = false;
  } else {
    dma_stalled = true;
    overrun = true;
  }

  bool published = completed_push_from_isr(completed);
  if (!published) {
    overrun = true;

    if (dma_stalled) {
      adc_set_current_buffer(completed);
    } else if (!dma_has_next) {
      adc_set_next_buffer(completed);
    }
  }

  if (overrun) {
    note_sampler_overrun();
  }

  adc_prime_next_if_possible();
}
