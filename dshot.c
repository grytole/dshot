#include <stdint.h>
#include <stdbool.h>
#include "dshot.h"

#ifndef DSHOT_DMA
#error "DSHOT DMA device is not defined.\n\tExample: #define DSHOT_DMA (DMA1)"
#endif
#ifndef DSHOT_DMA_CHANNEL
#error "DSHOT DMA channel is not defined.\n\tExample: #define DSHOT_DMA_CHANNEL (DMA_CHANNEL2)"
#endif
#ifndef DSHOT_TIM
#error "DSHOT timer device is not defined.\n\tExample: #define DSHOT_TIM (TIM2)"
#endif
#ifndef DSHOT_GPIO_PORT
#error "DSHOT GPIO port is not defined.\n\tExample: #define DSHOT_GPIO_PORT (GPIOA)"
#endif
#ifndef DSHOT_GPIO_PINS
#error "DSHOT GPIO pins are not defined.\n\tExample: #define DSHOT_GPIO_PINS (GPIO0 | GPIO1 | GPIO2 | GPIO3)"
#endif

#define DSHOT_THROTTLE_BITS  (11)
#define DSHOT_TELEMETRY_BITS (1)
#define DSHOT_CHECKSUM_BITS  (4)
#define DSHOT_PACKET_BITS    (DSHOT_THROTTLE_BITS + DSHOT_TELEMETRY_BITS + DSHOT_CHECKSUM_BITS)
#define DSHOT_GAP_BITS       (2)

#define DSHOT_THROTTLE_MAX   (0x7FF)
#define DSHOT_TELEMETRY_OFF  (0)

#define DSHOT_LUT_SLOTS_PER_BIT (8)
#define DSHOT_LUT_SLOT_BIT_0    (3)
#define DSHOT_LUT_SLOT_BIT_1    (6)
#define DSHOT_LUT_LEN           ((DSHOT_PACKET_BITS + DSHOT_GAP_BITS) * DSHOT_LUT_SLOTS_PER_BIT)

static uint32_t dshot_lut[DSHOT_LUT_LEN] = {};

static void dshot_checksum(uint16_t packet, uint8_t *csum);

/* private functions */

static uint8_t dshot_checksum(uint16_t packet)
{
  return ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0f);
}

/* public functions */

void dshot_init(dshot_baud_e speed)
{
  uint16_t tim_period = (rcc_get_timer_clk_freq(DSHOT_TIM) / speed) - 1;

  /* configure gpio pins */
  gpio_set_mode(DSHOT_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, DSHOT_GPIO_PINS);

  /* configure DSHOT_DMA */
  dma_channel_reset(DSHOT_DMA, DSHOT_DMA_CHANNEL);
  dma_set_peripheral_address(DSHOT_DMA, DSHOT_DMA_CHANNEL, (uint32_t)(&(GPIO_BSRR(DSHOT_GPIO_PORT))));
  dma_set_memory_address(DSHOT_DMA, DSHOT_DMA_CHANNEL, (uint32_t)(dshot_lut));
  dma_set_peripheral_size(DSHOT_DMA, DSHOT_DMA_CHANNEL, DMA_CCR_PSIZE_32BIT);
  dma_set_memory_size(DSHOT_DMA, DSHOT_DMA_CHANNEL, DMA_CCR_MSIZE_32BIT);
  dma_enable_memory_increment_mode(DSHOT_DMA, DSHOT_DMA_CHANNEL);
  dma_disable_peripheral_increment_mode(DSHOT_DMA, DSHOT_DMA_CHANNEL);
  dma_set_read_from_memory(DSHOT_DMA, DSHOT_DMA_CHANNEL);
  dma_set_priority(DSHOT_DMA, DSHOT_DMA_CHANNEL, DMA_CCR_PL_HIGH);

  /* configure timer */
  timer_set_clock_division(DSHOT_TIM, TIM_CR1_CKD_CK_INT);
  timer_set_alignment(DSHOT_TIM, TIM_CR1_CMS_EDGE);
  timer_direction_up(DSHOT_TIM);
  timer_enable_preload(DSHOT_TIM);
  timer_set_period(DSHOT_TIM, tim_period);
  timer_enable_update_event(DSHOT_TIM);
  timer_set_dma_on_update_event(DSHOT_TIM);
  timer_enable_irq(DSHOT_TIM, TIM_DIER_UDE);
  timer_enable_counter(DSHOT_TIM);
}

void dshot_set(uint8_t id, uint16_t value)
{
  uint16_t packet = 0;
  uint8_t csum = 0;
  uint8_t i = 0;
  uint16_t lut_bit_pos = 0;
  uint16_t gpio_pin = 0;

  if (16 > id)
  {
    gpio_pin = (1 << id);

    if (gpio_pin & DSHOT_GPIO_PINS)
    {
      /* set and clamp throttle value */
      packet = value & DSHOT_THROTTLE_MAX;

      /* set telemetry bit to disabled */
      packet = (packet << DSHOT_TELEMETRY_LEN) | DSHOT_TELEMETRY_OFF;

      /* set packet checksum */
      csum = dshot_checksum(packet);
      packet = (packet << DSHOT_CHECKSUM_LEN) | csum;

      /* prepare lookup table */
      for (i = 0; i < DSHOT_PACKET_BITS; i++)
      {
        lut_offset = i * DSHOT_LUT_SLOTS_PER_BIT;

        if (packet & 0x8000)
        {
          /* remove set LO, add clear LO, add set HI, remove clear HI */
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_0] &= ~(gpio_pin);
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_0] |= (gpio_pin << 16);
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_1] |= gpio_pin;
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_1] &= ~(gpio_pin << 16);
        }
        else
        {
          /* add set LO, remove clear LO, remove set HI, add clear HI */
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_0] |= gpio_pin;
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_0] &= ~(gpio_pin << 16);
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_1] &= ~(gpio_pin);
          dshot_lut[lut_offset + DSHOT_LUT_SLOT_BIT_1] |= (gpio_pin << 16);
        }
        packet <<= 1;
      }
    }
  }
}

bool dshot_send(void)
{
  bool result = false;

  /* prevent overlapping transfers */
  if (0 == dma_get_number_of_data(DSHOT_DMA, DSHOT_DMA_CHANNEL))
  {
    dma_disable_channel(DSHOT_DMA, DSHOT_DMA_CHANNEL);
    dma_set_number_of_data(DSHOT_DMA, DSHOT_DMA_CHANNEL, DSHOT_LUT_LEN);
    dma_enable_channel(DSHOT_DMA, DSHOT_DMA_CHANNEL);
    result = true;
  }

  return result;
}

