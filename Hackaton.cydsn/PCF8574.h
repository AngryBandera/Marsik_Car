#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void PCF8574_begin(void);
uint8_t PCF8574_read8(void);
void PCF8574_write8(uint8_t value);

#ifdef __cplusplus
}
#endif

//  -- END OF FILE --

