/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Signal type classification from measurement statistics and (optionally)
 * an FFT magnitude spectrum. Pure C, host-testable.
 */

#ifndef _CLASSIFY_H_
#define _CLASSIFY_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include "measure.h"

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  SIG_UNKNOWN,
  SIG_DC,
  SIG_NOISE,
  SIG_SINE,
  SIG_SQUARE,
  SIG_PULSE,
  SIG_TRIANGLE,
  SIG_SAW,
} SignalType;

typedef struct
{
  SignalType type;
  int        thd_x10;  // total harmonic distortion in 0.1% units; -1 if n/a
} SignalClass;

/*- Prototypes --------------------------------------------------------------*/
// mag may be NULL (no spectrum available); fund_bin is the predicted FFT bin
// of the fundamental, 0 when unknown
void classify_signal(const Measure *m, const float *mag, int bins,
    int fund_bin, SignalClass *out);

const char *classify_name(SignalType type);

#endif // _CLASSIFY_H_
