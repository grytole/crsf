#ifndef __CRSF_H__
#define __CRSF_H__

#include <stdint.h>
#include <stdbool.h>

bool crsf_init(uint32_t usart);
void crsf_update(uint32_t timestamp_ms);
uint16_t crsf_get_channel(uint8_t id);

#endif /* __CRSF_H__ */
