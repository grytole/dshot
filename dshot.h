#ifndef __DSHOT_H__
#define __DSHOT_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  DSHOT_150 = 1200000,
  DSHOT_300 = 2400000,
  DSHOT_600 = 4800000
} dshot_baud_e;

void dshot_init(dshot_baud_e speed);

void dshot_set(uint8_t id, uint16_t value);

bool dshot_send(void);

#endif /* __DSHOT_H__ */

