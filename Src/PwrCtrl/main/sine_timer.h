#pragma once

// ======================================================================
// CONSTANTS
// ======================================================================

#define MAX_SINE_AMPLITUDE  50.0
#define MIN_SINE_AMPLITUDE  5.0

// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

#ifdef __cplusplus
extern "C" {
#endif


void Sine_start_task(void);
void Sine_set_amplitude(float ampl);
void Sine_start_wave();
void Sine_stop_wave();

#ifdef __cplusplus
}
#endif
