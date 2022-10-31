#pragma once

// ======================================================================
// CONSTANTS
// ======================================================================

#define TOP_SINE_AMPLITUDE  50.0
#define MAX_SINE_AMPLITUDE  (TOP_SINE_AMPLITUDE - 0.5)
#define MIN_SINE_AMPLITUDE  4.5

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
