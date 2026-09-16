#include "Eris.h"
#include "streaming.h"
#include "signals.h"

#include <modules/sinewave.h>
#include <string.h>

namespace Streaming {

#define MAXFNC 8

struct Feature {
  const char * name;   // the token S_F accepts; also the DESC / ASCII label
  void (*fn)();
};

// The feature label is passed to StreamSamples so DESC reports it and ASCII
// mode prefixes it ("WAVES.sine"). It must match the name in the table below.
static void streamSine()   { StreamSamples<floatSample_t,    TXBUFFERSIZE>(SineWave::buffer, packet, "SINE");   }
static void streamWaves()  { StreamSamples<WavesSample_t,   TXBUFFERSIZE>(Waves::buffer,    packet, "WAVES");  }
static void streamAnalog() { StreamSamples<AnalogPairSample_t, TXBUFFERSIZE>(Analog::buffer,   packet, "ANALOG"); }
static void streamTemp()   { StreamSamples<floatSample_t,    TXBUFFERSIZE>(Temp::buffer,     packet, "TEMP");   }

// Add a stream here and it becomes selectable from the phone: the app sends
// S_F with the names you tick and learns each layout from DESC.
static const Feature features[] = {
  { "WAVES",  streamWaves  },
  { "ANALOG", streamAnalog },
  { "TEMP",   streamTemp   },
  { "SINE",   streamSine   },
};

static void (*streamfnc[MAXFNC])();
static uint8_t Nfunctions = 0;

void ClearFunctions() {
  Nfunctions = 0;
}

bool AddFunction(char * arg) {
  if (Nfunctions >= MAXFNC) {
    Error::RaiseError(Error::MEMORY, (char *)"STREAMFNC");
    return false;
  }
  for (const Feature & f : features) {
    if (!strncmp(f.name, arg, MAXSTRCMP)) {
      streamfnc[Nfunctions++] = f.fn;
      return true;
    }
  }
  return false;
}

void Stream() {
  if (ErisAscii::describing()) {
    // DESC: each feature emits its own TEXT packet, so no framing here.
    for (uint8_t i = 0; i < Nfunctions; i++) (*streamfnc[i])();
    return;
  }

  if (ErisAscii::isAscii()) {
    ErisAscii::frameBegin();
    for (uint8_t i = 0; i < Nfunctions; i++) (*streamfnc[i])();
    ErisAscii::frameEnd();
    return;
  }

  packet.start(Packet::PacketType::DATA);
  for (uint8_t i = 0; i < Nfunctions; i++) (*streamfnc[i])();
  packet.send();
}

}
