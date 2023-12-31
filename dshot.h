#ifndef __DSHOT_H__
#define __DSHOT_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  DSHOT_150 = 1200000,
  DSHOT_300 = 2400000,
  DSHOT_600 = 4800000
} dshot_baud_e;

typedef struct {
  dshot_baud_e speed;
  uint32_t     gpio;
  uint16_t     pins;
  uint32_t     tim;
  uint32_t     dma;
  uint8_t      dma_channel;
} dshot_config_t;

bool dshot_init(dshot_config_t *config);
bool dshot_set(uint16_t pin, uint16_t value);
bool dshot_send(void);

#endif /* __DSHOT_H__ */

