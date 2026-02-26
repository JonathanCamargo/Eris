#include "Eris.h"
#include "fsr.h"
#include "configuration.h"
#include "serialcommand.h"

namespace Sync{


//Buffer for readings
ErisBuffer<uint8_tSample_t> buffer ;

// Indices and flags

static void ISR_SYNC_CHANGE(){
  ERIS_CRITICAL_ENTER();
  float timestamp = ((float)(micros() - SerialCom::startTime))/1.0e3;
  uint8_tSample_t thisSample;
  thisSample.timestamp=timestamp;
  thisSample.value=digitalRead(PIN_SYNC);
  buffer.append(thisSample);
  ERIS_CRITICAL_EXIT();
}

void start(void){
    pinMode(PIN_SYNC,INPUT_PULLUP);
    buffer.init();
    Serial.println("SYNCREADY");
    attachInterrupt(digitalPinToInterrupt(PIN_SYNC),ISR_SYNC_CHANGE,CHANGE);
 }
}
