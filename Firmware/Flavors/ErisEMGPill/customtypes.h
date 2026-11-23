#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors
#include "configuration.h"

/*
 * One EMG tick: every pill read on the same tick, in volts at the ADC pin.
 * The pills idle at mid-supply (~1.65 V on 3V3), so remove the DC on the host
 * (high-pass / band-pass) before rectifying or taking an envelope.
 *
 * Fields come out as ch0..chN-1 (EMG.ch0, EMG.ch1 in text mode).
 */
typedef struct EMGPillSample{
  float timestamp;              // ms since t0
  float ch[EMG_NUMCHANNELS];    // V at each pill's OUT
} EMGPillSample_t;

ERIS_DESCRIBE_CHANNELS(EMGPillSample_t, ch, EMG_NUMCHANNELS)

#endif
