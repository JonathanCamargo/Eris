#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors

/*
 * One sample of axis state. position/velocity/target are the three traces you
 * want on a plot when tuning a move, and `sg` is the one you want when tuning
 * the stall threshold -- all four are SIGNAL, so the Serial Plotter draws
 * them. The flags are META, which keeps them off the plot (integer levels
 * flatten the autoscale) while still shipping on the binary path and in DESC.
 *
 * Field order is largest-first so the struct has no compiler padding:
 * 4+4+4+4+2+2+1+1+1+1 = 24 bytes. The trailing `pad` byte is what makes that
 * come out even, and it is declared rather than left implicit because
 * ERIS_DESCRIBE_END checks the byte accounting and the host decodes what the
 * descriptor says.
 */
typedef struct StepperSample{
  float    timestamp;   // ms since t0
  float    position;    // mm
  float    velocity;    // mm/s, signed (sign = direction of travel)
  float    target;      // mm, the commanded G0 destination
  uint16_t sg;          // StallGuard4 SG_RESULT, 0..1023. HIGH = unloaded,
                        // falls as load rises. 0 when the UART is down.
  uint16_t status;      // packed TMC2209 DRV_STATUS flags, see tmc2209.h
  uint8_t  moving;      // 1 while the axis has not reached target
  uint8_t  enabled;     // 1 when the driver outputs are energized
  uint8_t  stalled;     // 1 once a hard stop has been detected (latched)
  uint8_t  pad;         // keeps sizeof() at 24 with no implicit padding
} StepperSample_t;

ERIS_DESCRIBE_BEGIN(StepperSample_t)
  ERIS_TIME    (StepperSample_t, timestamp)
  ERIS_FLOAT   (StepperSample_t, position)
  ERIS_FLOAT   (StepperSample_t, velocity)
  ERIS_FLOAT   (StepperSample_t, target)
  ERIS_U16     (StepperSample_t, sg)
  ERIS_META_U16(StepperSample_t, status)
  ERIS_META_U8 (StepperSample_t, moving)
  ERIS_META_U8 (StepperSample_t, enabled)
  ERIS_META_U8 (StepperSample_t, stalled)
  ERIS_PAD     (StepperSample_t, pad)
ERIS_DESCRIBE_END(StepperSample_t)

#endif
