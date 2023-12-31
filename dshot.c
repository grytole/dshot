#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include "dshot.h"

#define DSHOT_THROTTLE_BITS  (11)
#define DSHOT_TELEMETRY_BITS (1)
#define DSHOT_CHECKSUM_BITS  (4)
#define DSHOT_GAP_BITS       (1)
#define DSHOT_PACKET_BITS    (DSHOT_THROTTLE_BITS + DSHOT_TELEMETRY_BITS + DSHOT_CHECKSUM_BITS)

#define DSHOT_THROTTLE_MAX   (0x7FF)
#define DSHOT_TELEMETRY_OFF  (0)

#define DSHOT_LUT_BIT_0      (15) /* DSHOT150: 2.50us @ 6MHz */
#define DSHOT_LUT_BIT_1      (30) /* DSHOT150: 5.00us @ 6MHz */
#define DSHOT_LUT_BIT_PERIOD (40) /* DSHOT150: 6.67us @ 6MHz */
#define DSHOT_LUT_LEN        (DSHOT_PACKET_BITS + DSHOT_GAP_BITS)

static dshot_config_t dshot_config = {};
static bool dshot_config_initialized = false;
static uint8_t dshot_lut[DSHOT_LUT_LEN] = {};

static uint8_t dshot_checksum(uint16_t packet);

/* private functions */

static uint8_t dshot_checksum(uint16_t packet)
{
  return ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0f);
}

/* public functions */

bool dshot_init(dshot_config_t *config)
{
  bool result = false;
  uint16_t tim_prescaler = 0;
  uint16_t tim_period = 0;

  if (config)
  {
    if ((0 != config->speed) &&
        (0 != config->gpio) &&
        (0 != config->pin) &&
        (0 != config->tim) &&
        (0 != config->dma) &&
        (0 != config->dma_channel))
    {
      dshot_config.speed = config->speed;
      dshot_config.gpio = config->gpio;
      dshot_config.pin = config->pin;
      dshot_config.tim = config->tim;
      dshot_config.tim_channel = config->tim_channel;
      dshot_config.dma = config->dma;
      dshot_config.dma_channel = config->dma_channel;
      dshot_config_initialized = true;

      tim_prescaler = (rcc_get_timer_clk_freq(dshot_config.tim) / dshot_config.speed) - 1;
      tim_period = DSHOT_LUT_BIT_PERIOD - 1;

      /* configure gpio pin */
      gpio_set_mode(dshot_config.gpio, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, dshot_config.pin);
      gpio_clear(dshot_config.gpio, dshot_config.pin);

      /* configure dshot_config.dma */
      dma_channel_reset(dshot_config.dma, dshot_config.dma_channel);
      /* TODO: move dma_set_peripheral_address() to dshot_send() and use corresponding TIM_CCRx register for requested channel */
      dma_set_peripheral_address(dshot_config.dma, dshot_config.dma_channel, (uint32_t)(&(TIM_CCR1(dshot_config.tim))));
      dma_set_memory_address(dshot_config.dma, dshot_config.dma_channel, (uint32_t)(dshot_lut));
      dma_set_peripheral_size(dshot_config.dma, dshot_config.dma_channel, DMA_CCR_PSIZE_16BIT);
      dma_set_memory_size(dshot_config.dma, dshot_config.dma_channel, DMA_CCR_MSIZE_8BIT);
      dma_enable_memory_increment_mode(dshot_config.dma, dshot_config.dma_channel);
      dma_disable_peripheral_increment_mode(dshot_config.dma, dshot_config.dma_channel);
      dma_set_read_from_memory(dshot_config.dma, dshot_config.dma_channel);
      dma_set_priority(dshot_config.dma, dshot_config.dma_channel, DMA_CCR_PL_HIGH);

      /* configure timer */
      timer_set_mode(dshot_config.tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
      timer_enable_preload(dshot_config.tim);
      timer_set_prescaler(dshot_config.tim, tim_prescaler);
      timer_set_period(dshot_config.tim, tim_period);
      timer_enable_oc_output(dshot_config.tim, dshot_config.tim_channel);
      timer_set_oc_mode(dshot_config.tim, dshot_config.tim_channel, TIM_OCM_PWM1);
      timer_enable_oc_preload(dshot_config.tim, dshot_config.tim_channel);
      timer_enable_update_event(dshot_config.tim);
      timer_set_dma_on_update_event(dshot_config.tim);
      timer_enable_irq(dshot_config.tim, TIM_DIER_UDE);
      timer_enable_counter(dshot_config.tim);

      result = true;
    }
  }

  return result;
}

bool dshot_set(uint16_t value)
{
  bool result = false;
  uint16_t packet = 0;
  uint8_t csum = 0;
  uint8_t i = 0;

  if (dshot_config_initialized)
  {
    /* set and clamp throttle value */
    packet = value & DSHOT_THROTTLE_MAX;
    /* set telemetry bit to disabled */
    packet = (packet << DSHOT_TELEMETRY_BITS) | DSHOT_TELEMETRY_OFF;
    /* set packet checksum */
    csum = dshot_checksum(packet);
    packet = (packet << DSHOT_CHECKSUM_BITS) | csum;
    /* set lookup table value */
    for (i = 0; i < DSHOT_PACKET_BITS; i++)
    {
      dshot_lut[i] = (packet & 0x8000) ? DSHOT_LUT_BIT_1 : DSHOT_LUT_BIT_0;
      packet <<= 1;
    }
    result = true;
  }

  return result;
}

bool dshot_send(void)
{
  bool result = false;

  if (dshot_config_initialized)
  {
    /* prevent overlapping transfers */
    if (0 == dma_get_number_of_data(dshot_config.dma, dshot_config.dma_channel))
    {
      dma_disable_channel(dshot_config.dma, dshot_config.dma_channel);
      dma_set_number_of_data(dshot_config.dma, dshot_config.dma_channel, DSHOT_LUT_LEN);
      dma_enable_channel(dshot_config.dma, dshot_config.dma_channel);
      result = true;
    }
  }

  return result;
}

