#ifndef __DSHOT_H__
#define __DSHOT_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  DSHOT_150 =  6000000,
  DSHOT_300 = 12000000,
  DSHOT_600 = 24000000
} dshot_baud_e;

typedef struct {
  uint32_t speed;
  uint32_t gpio;
  uint16_t pin;
  uint32_t tim;
  uint8_t  tim_channel;
  uint32_t dma;
  uint8_t  dma_channel;
} dshot_config_t;

bool dshot_init(dshot_config_t *config);
bool dshot_set(uint16_t value);
bool dshot_send(void);

#endif /* __DSHOT_H__ */

