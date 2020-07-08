#include "Eris.h"
#include "emg.h"
#include "configuration.h"

#include "sdcard.h"

#include <ADS1299.h>
#include <Filters.h>

#include <FeatureExtractor.h>

namespace EMG{
  
thread_t *extractFeaturesEMG = NULL;
thread_t *readSensor1 = NULL;

//Buffer for readings 
ErisBuffer<EMGSample_t> buffer;



static ADS1299 sensor1(PIN_EMG_ADC0_SS,PIN_EMG_ADC0_DRDY);
// Filters for the EMG signal
FilterOnePole *filterlp[EMG_NUMCHANNELS];
FilterOnePole *filterhp[EMG_NUMCHANNELS];


static uint16_t buffersize = sensor1.bufferSize();

// Semaphore for collecting the data in sensor1 buffer
static binary_semaphore_t xsensor1FullSemaphore;

float * data[EMG_NUMCHANNELS];

static void ISR_NewSample(){
	chSysLockFromISR();    
  float timestamp = ((float)(micros() - SDCard::startTime))/1000.0;  
	if (sensor1.collectData()) {
	
		//chBSemSignalI(&xsensor1FullSemaphore);     
    //Sample every channel
    EMGSample_t thisSample;
    thisSample.timestamp=timestamp; 
    for(uint8_t chan = 0; chan < EMG_NUMCHANNELS; chan++) {
        float * data=sensor1.get_data(chan);
        float value=data[0];       
        //value=filterhp[chan]->input(filterlp[chan]->input(value)); 
        //Preview data Serial.println(v);           
        thisSample.ch[chan]=value;      
    }  
    buffer.append(thisSample);
    
  	} 
    #if SDCARD
    //SDCard::addEMG(thisSample);
    SDCard::emgbuffer.append(thisSample);
    #endif
  
    chSysUnlockFromISR();    
}

/*
static THD_WORKING_AREA(waSensor1_T, 256);
static THD_FUNCTION(Sensor1_T, arg) {  
  while(1){
    msg_t msg = chBSemWaitTimeout(&xsensor1FullSemaphore, MS2ST(10));
    if (msg == MSG_TIMEOUT) {
      continue;
    }
     
  for(uint8_t chan = 0; chan < EMG_NUM_CHANNELS; chan++) {      
       data[chan] = sensor1.get_data(chan);            
  }  
  //This should be faster than filling out a new sensor1 buffer
  for (uint16_t i=0;i<buffersize;i++){
    for(uint8_t chan = 0; chan < EMG_NUM_CHANNELS;chan++) {
  	  //float value=test;     
  	  float value=data[chan][i];       
      value=filterhp[chan]->input(filterlp[chan]->input(value)); 
      buffer[chan]->append(value);           
    }          
  }
}
}
*/

  
void start(void){ 
    
    sensor1.setNumChannels(EMG_NUMCHANNELS);
  
    // Start ErisBuffers        
    buffer.init();     
    
    chBSemObjectInit(&xsensor1FullSemaphore,true);    
    
    //Set up ADC chip	    	  
    pinMode(PIN_EMG_ADC0_DRDY,INPUT_PULLUP);
    pinMode(PIN_EMG_RESET,OUTPUT);
    digitalWrite(PIN_EMG_RESET,HIGH);
    delay(1000);

    // Attach the interrupt to the DRDY pin defined by the class
    attachInterrupt(digitalPinToInterrupt(PIN_EMG_ADC0_DRDY),ISR_NewSample,FALLING);

    sensor1.startCollecting();
    
		// create tasks at priority lowest priority
	  //readSensor1=chThdCreateStatic(waSensor1_T, sizeof(waSensor1_T),NORMALPRIO+5, Sensor1_T, NULL);     
	
	}
}
