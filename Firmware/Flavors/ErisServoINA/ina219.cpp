#include "Eris.h"
#include "ina219.h"

#include <Adafruit_INA219.h>

namespace INA219{

eris_thread_ref_t sampleThread = NULL;

// INA219 sensor instance
static Adafruit_INA219 ina(INA219_I2C_ADDR);

// Buffer for readings
ErisBuffer<inaSample_t> buffer;

// Sampling thread — runs at INA219_SAMPLE_RATE_HZ (100 Hz)
ERIS_THREAD_WA(waSampleINA_T, ERIS_STACK_MEDIUM);
ERIS_THREAD_FUNC(SampleINA_T) {
  while(1){
    float timestamp = ((float)(micros() - t0)) / 1.0e3;

    inaSample_t sample;
    sample.timestamp   = timestamp;
    sample.current_mA  = ina.getCurrent_mA();
    sample.busVoltage_V = ina.getBusVoltage_V();
    sample.power_mW    = ina.getPower_mW();
    buffer.append(sample);

    eris_sleep_ms(INA219_SAMPLE_PERIOD_MS);
  }
}

void start(void){
  ina.begin();
  buffer.init();
  sampleThread = eris_thread_create(waSampleINA_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+1, SampleINA_T, NULL);
  Serial.println("INA219 ready");
}

}
