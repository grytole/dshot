#ifndef __DSHOT_H__
#define __DSHOT_H__

#include <stdint.h>
#include <stdbool.h>

#define DSHOT_NUM_CHANNELS (4)

typedef enum {
  DSHOT_150,
  DSHOT_300,
  DSHOT_600
} dshot_speed_e;

bool dshot_init(dshot_speed_e speed, uint32_t tim);
bool dshot_set(uint8_t channel, uint16_t value);
bool dshot_send(uint8_t channel);

#endif /* __DSHOT_H__ */

