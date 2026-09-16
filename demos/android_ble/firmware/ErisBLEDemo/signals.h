#ifndef SIGNALS_H
#define SIGNALS_H

#include "Eris.h"

// Demo signal sources. None need wiring: WAVES is synthetic, ANALOG reads
// floating pins (touch them, or wire a potentiometer), TEMP is on-chip.

namespace Demo {
  extern volatile float freqHz;   // WAVES frequency, set by FREQ <hz>
  extern volatile float amp;      // WAVES amplitude, set by AMP <a>
}

namespace Waves {
  extern ErisBuffer<WavesSample_t> buffer;
  void start(void);
}

namespace Analog {
  extern ErisBuffer<AnalogPairSample_t> buffer;
  void start(void);
}

namespace Temp {
  extern ErisBuffer<floatSample_t> buffer;
  void start(void);
}

#endif
