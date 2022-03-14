#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void pwm_init(void);
void set_duty_a(float duty_cycle);
void set_duty_b(float duty_cycle);

#ifdef __cplusplus
}
#endif
