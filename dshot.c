/* dshot implementation for dma/tim (stm32f103c8t6) */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "dshot.h"


/* defines */

#define DSHOT_150_FREQ (6000000)
#define DSHOT_300_FREQ (12000000)
#define DSHOT_600_FREQ (24000000)

#define DSHOT_THROTTLE_BITS  (11)
#define DSHOT_TELEMETRY_BITS (1)
#define DSHOT_CHECKSUM_BITS  (4)
#define DSHOT_GAP_BITS       (1)
#define DSHOT_PACKET_BITS    (DSHOT_THROTTLE_BITS + DSHOT_TELEMETRY_BITS + DSHOT_CHECKSUM_BITS)

#define DSHOT_THROTTLE_MAX   (0x7FF)
#define DSHOT_TELEMETRY_OFF  (0)

#define DSHOT_LUT_BIT_0      (15) /* DSHOT150: 2.50us @ 6MHz; DSHOT300: 1.25us @ 12MHz; DSHOT600: 0.625us @ 24MHz */
#define DSHOT_LUT_BIT_1      (30) /* DSHOT150: 5.00us @ 6MHz; DSHOT300: 2.50us @ 12MHz; DSHOT600: 1.25us  @ 24MHz */
#define DSHOT_LUT_BIT_PERIOD (40) /* DSHOT150: 6.67us @ 6MHz; DSHOT300: 3.33us @ 12MHz; DSHOT600: 1.67us  @ 24MHz */
#define DSHOT_LUT_LEN        (DSHOT_PACKET_BITS + DSHOT_GAP_BITS)


/* typedefs */

typedef struct {
  bool     enabled;
  enum     tim_oc_id tim_channel_id;
  uint32_t p_dma_src;
  uint32_t p_dma_dst;
  uint32_t gpio_port;
  uint16_t gpio_pin;
  uint8_t  lut[DSHOT_LUT_LEN];
} dshot_channel_t;

typedef struct {
  bool     initialized;
  uint32_t freq;
  uint32_t tim;
  uint32_t dma;
  uint8_t  dma_channel;
  dshot_channel_t channels[DSHOT_NUM_CHANNELS];
} dshot_config_t;


/* variables */

static dshot_config_t dshot_config = {};


/* prototypes */

static bool dshot_init_config(dshot_config_t *p_config, dshot_speed_e speed, uint32_t tim);
static uint8_t dshot_checksum(uint16_t packet);


/* private functions */

static bool dshot_init_config(dshot_config_t *p_config, dshot_speed_e speed, uint32_t tim)
{
  bool result = false;
  bool valid_config_found = false;

  if (NULL != p_config)
  {
    switch (speed)
    {
      case DSHOT_150:
      {
        p_config->freq = DSHOT_150_FREQ;
        break;
      }

      case DSHOT_300:
      {
        p_config->freq = DSHOT_300_FREQ;
        break;
      }

      case DSHOT_600:
      {
        p_config->freq = DSHOT_600_FREQ;
        break;
      }

      default:
      {
        break;
      }
    }

    if (0 != p_config->freq)
    {
      p_config->tim = tim;

      switch (p_config->tim)
      {
        // TODO: case TIM1:
        // TODO: {
        // TODO:   p_config->dma = DMA1;
        // TODO:   p_config->dma_channel = DMA_CHANNEL5;
        // TODO:   p_config->channels[0].gpio_port = GPIO_BANK_TIM1_CH1;
        // TODO:   p_config->channels[0].gpio_pin  = GPIO_TIM1_CH1;
        // TODO:   p_config->channels[1].gpio_port = GPIO_BANK_TIM1_CH2;
        // TODO:   p_config->channels[1].gpio_pin  = GPIO_TIM1_CH2;
        // TODO:   p_config->channels[2].gpio_port = GPIO_BANK_TIM1_CH3;
        // TODO:   p_config->channels[2].gpio_pin  = GPIO_TIM1_CH3;
        // TODO:   p_config->channels[3].gpio_port = GPIO_BANK_TIM1_CH4;
        // TODO:   p_config->channels[3].gpio_pin  = GPIO_TIM1_CH4;
        // TODO:   valid_config_found = true;
        // TODO:   break;
        // TODO: }

        case TIM2:
        {
          p_config->dma = DMA1;
          p_config->dma_channel = DMA_CHANNEL2;
          p_config->channels[0].gpio_port = GPIO_BANK_TIM2_CH1_ETR;
          p_config->channels[0].gpio_pin  = GPIO_TIM2_CH1_ETR;
          p_config->channels[1].gpio_port = GPIO_BANK_TIM2_CH2;
          p_config->channels[1].gpio_pin  = GPIO_TIM2_CH2;
          p_config->channels[2].gpio_port = GPIO_BANK_TIM2_CH3;
          p_config->channels[2].gpio_pin  = GPIO_TIM2_CH3;
          p_config->channels[3].gpio_port = GPIO_BANK_TIM2_CH4;
          p_config->channels[3].gpio_pin  = GPIO_TIM2_CH4;
          valid_config_found = true;
          break;
        }

        case TIM3:
        {
          p_config->dma = DMA1;
          p_config->dma_channel = DMA_CHANNEL3;
          p_config->channels[0].gpio_port = GPIO_BANK_TIM3_CH1;
          p_config->channels[0].gpio_pin = GPIO_TIM3_CH1;
          p_config->channels[1].gpio_port = GPIO_BANK_TIM3_CH2;
          p_config->channels[1].gpio_pin = GPIO_TIM3_CH2;
          p_config->channels[2].gpio_port = GPIO_BANK_TIM3_CH3;
          p_config->channels[2].gpio_pin = GPIO_TIM3_CH3;
          p_config->channels[3].gpio_port = GPIO_BANK_TIM3_CH4;
          p_config->channels[3].gpio_pin = GPIO_TIM3_CH4;
          valid_config_found = true;
          break;
        }

        case TIM4:
        {
          p_config->dma = DMA1;
          p_config->dma_channel = DMA_CHANNEL7;
          p_config->channels[0].gpio_port = GPIO_BANK_TIM4_CH1;
          p_config->channels[0].gpio_pin = GPIO_TIM4_CH1;
          p_config->channels[1].gpio_port = GPIO_BANK_TIM4_CH2;
          p_config->channels[1].gpio_pin = GPIO_TIM4_CH2;
          p_config->channels[2].gpio_port = GPIO_BANK_TIM4_CH3;
          p_config->channels[2].gpio_pin = GPIO_TIM4_CH3;
          p_config->channels[3].gpio_port = GPIO_BANK_TIM4_CH4;
          p_config->channels[3].gpio_pin = GPIO_TIM4_CH4;
          valid_config_found = true;
          break;
        }

        default:
        {
          break;
        }
      }

      if (true == valid_config_found)
      {
        /* configure dshot channels */
        p_config->channels[0].tim_channel_id = TIM_OC1;
        p_config->channels[0].p_dma_src = (uint32_t)(p_config->channels[0].lut);
        p_config->channels[0].p_dma_dst = (uint32_t)(&(TIM_CCR1(p_config->tim)));
        p_config->channels[0].enabled = true;

        p_config->channels[1].tim_channel_id = TIM_OC2;
        p_config->channels[1].p_dma_src = (uint32_t)(p_config->channels[1].lut);
        p_config->channels[1].p_dma_dst = (uint32_t)(&(TIM_CCR2(p_config->tim)));
        p_config->channels[1].enabled = true;

        p_config->channels[2].tim_channel_id = TIM_OC3;
        p_config->channels[2].p_dma_src = (uint32_t)(p_config->channels[2].lut);
        p_config->channels[2].p_dma_dst = (uint32_t)(&(TIM_CCR3(p_config->tim)));
        p_config->channels[2].enabled = true;

        p_config->channels[3].tim_channel_id = TIM_OC4;
        p_config->channels[3].p_dma_src = (uint32_t)(p_config->channels[3].lut);
        p_config->channels[3].p_dma_dst = (uint32_t)(&(TIM_CCR4(p_config->tim)));
        p_config->channels[3].enabled = true;

        p_config->initialized = true;
        result = true;
      }
    }
  }

  return result;
}

static uint8_t dshot_checksum(uint16_t packet)
{
  return ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0f);
}


/* public functions */

bool dshot_init(dshot_speed_e speed, uint32_t tim)
{
  bool result = false;
  uint16_t tim_prescaler = 0;
  uint16_t tim_period = 0;
  uint8_t i = 0;
  dshot_channel_t *channel_cfg = NULL;

  if (true == dshot_init_config(&dshot_config, speed, tim))
  {
    /* configure dma */
    dma_channel_reset(dshot_config.dma, dshot_config.dma_channel);
    dma_set_peripheral_size(dshot_config.dma, dshot_config.dma_channel, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(dshot_config.dma, dshot_config.dma_channel, DMA_CCR_MSIZE_8BIT);
    dma_enable_memory_increment_mode(dshot_config.dma, dshot_config.dma_channel);
    dma_disable_peripheral_increment_mode(dshot_config.dma, dshot_config.dma_channel);
    dma_set_read_from_memory(dshot_config.dma, dshot_config.dma_channel);
    dma_set_priority(dshot_config.dma, dshot_config.dma_channel, DMA_CCR_PL_HIGH);

    /* configure timer */
    tim_prescaler = (rcc_get_timer_clk_freq(dshot_config.tim) / dshot_config.freq) - 1;
    tim_period = DSHOT_LUT_BIT_PERIOD - 1;

    timer_set_mode(dshot_config.tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_enable_preload(dshot_config.tim);
    timer_set_prescaler(dshot_config.tim, tim_prescaler);
    timer_set_period(dshot_config.tim, tim_period);

    for (i = 0; i < DSHOT_NUM_CHANNELS; i++)
    {
      channel_cfg = &(dshot_config.channels[i]);
      if (true == channel_cfg->enabled)
      {
        gpio_set_mode(channel_cfg->gpio_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, channel_cfg->gpio_pin);
        gpio_clear(channel_cfg->gpio_port, channel_cfg->gpio_pin);

        timer_enable_oc_output(dshot_config.tim, channel_cfg->tim_channel_id);
        timer_set_oc_mode(dshot_config.tim, channel_cfg->tim_channel_id, TIM_OCM_PWM1);
        timer_enable_oc_preload(dshot_config.tim, channel_cfg->tim_channel_id);
      }
    }

    /* TODO: for TIM1 handle special case with braking */

    timer_enable_update_event(dshot_config.tim);
    timer_set_dma_on_update_event(dshot_config.tim);
    timer_enable_irq(dshot_config.tim, TIM_DIER_UDE);
    timer_enable_counter(dshot_config.tim);
    result = true;
  }

  return result;
}

bool dshot_set(uint8_t channel, uint16_t value)
{
  bool result = false;
  uint16_t packet = 0;
  uint8_t csum = 0;
  uint8_t i = 0;

  if ((true == dshot_config.initialized) &&
      (DSHOT_NUM_CHANNELS > channel) &&
      (true == dshot_config.channels[channel].enabled))
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
      dshot_config.channels[channel].lut[i] = (packet & 0x8000) ? DSHOT_LUT_BIT_1 : DSHOT_LUT_BIT_0;
      packet <<= 1;
    }
    result = true;
  }

  return result;
}

bool dshot_send(uint8_t channel)
{
  bool result = false;
  dshot_channel_t *channel_cfg = NULL;

  if ((true == dshot_config.initialized) &&
      (DSHOT_NUM_CHANNELS > channel))
  {
    channel_cfg = &(dshot_config.channels[channel]);

    /* prevent overlapping transfers */
    if ((0 == dma_get_number_of_data(dshot_config.dma, dshot_config.dma_channel)) &&
        (true == channel_cfg->enabled))
    {
      dma_disable_channel(dshot_config.dma, dshot_config.dma_channel);
      dma_set_memory_address(dshot_config.dma, dshot_config.dma_channel, channel_cfg->p_dma_src);
      dma_set_peripheral_address(dshot_config.dma, dshot_config.dma_channel, channel_cfg->p_dma_dst);
      dma_set_number_of_data(dshot_config.dma, dshot_config.dma_channel, DSHOT_LUT_LEN);
      dma_enable_channel(dshot_config.dma, dshot_config.dma_channel);
      result = true;
    }
  }

  return result;
}

