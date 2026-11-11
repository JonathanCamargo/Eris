#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>
#include <eris_descriptor.h>   // ERIS_DESCRIBE_* field descriptors

/*
 * One calibration point: the FSR divider voltage and the load-cell force,
 * read on the same tick, plus where the axis was.
 *
 * voltage/force/position are SIGNAL, so the Serial Plotter draws them. The
 * rest is META: shipped on the binary path and in DESC, kept off the plot.
 *
 *   raw      HX711 counts with NO tare and NO scale applied. Logged so a
 *            recording can be re-calibrated offline if the scale turns out to
 *            be wrong -- force alone cannot be un-scaled.
 *   fresh    1 = the load cell converted on this tick. 0 = it is absent and
 *            this is an FSR-only sample (force is 0, not a measurement).
 *   overload 1 = the overload guard is holding the axis.
 *
 * 4+4+4+4+4+1+1+1+1 = 24 bytes, no implicit padding. The two pad bytes are
 * declared because ERIS_DESCRIBE_END checks the byte accounting and the host
 * decodes what the descriptor says.
 */
typedef struct FSRCalSample{
  float    timestamp;   // ms since t0
  float    voltage;     // V at the FSR divider tap
  float    force;       // N (tared counts while LOADCELL is uncalibrated)
  float    position;    // mm, axis position at this sample
  int32_t  raw;         // HX711 counts, untared, unscaled
  uint8_t  fresh;
  uint8_t  overload;
  uint8_t  pad0;
  uint8_t  pad1;
} FSRCalSample_t;

ERIS_DESCRIBE_BEGIN(FSRCalSample_t)
  ERIS_TIME    (FSRCalSample_t, timestamp)
  ERIS_FLOAT   (FSRCalSample_t, voltage)
  ERIS_FLOAT   (FSRCalSample_t, force)
  ERIS_FLOAT   (FSRCalSample_t, position)
  ERIS_FIELD_AS(FSRCalSample_t, raw, ERIS_T_I32, ERIS_ROLE_META)
  ERIS_META_U8 (FSRCalSample_t, fresh)
  ERIS_META_U8 (FSRCalSample_t, overload)
  ERIS_PAD     (FSRCalSample_t, pad0)
  ERIS_PAD     (FSRCalSample_t, pad1)
ERIS_DESCRIBE_END(FSRCalSample_t)

#endif
