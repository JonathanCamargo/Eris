#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
#include <eris_customtypes.h>  // eriscommon: floatSample_t, uint8_tSample_t, AnalogSample<N>
/*//////////////////////////////////////////// 

Definition of custom types for data exchange 

User can add more types as needed

////////////////////////////////////////////*/

// floatSample_t and uint8_tSample_t used to be re-declared here, which was a
// hard compile error: eris_flavor.h pulls in eriscommon.h -> eris_customtypes.h
// *before* this file, so both definitions landed in the same translation unit.
// They come from eriscommon now, already described for ASCII/DESC output.


#endif
