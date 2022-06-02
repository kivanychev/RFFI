#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void GPIO_init(void);
void LEDstate(uint32_t state);
void UARTenable(void);

#ifdef __cplusplus
}
#endif
