#ifndef __CRSF_H__
#define __CRSF_H__

#include <stdint.h>

void crsf_init(void);
void crsf_update(uint32_t timestamp_ms);
uint16_t crsf_get_channel(uint8_t id);

#endif /* __CRSF_H__ */

