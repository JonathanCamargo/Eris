#include "Eris.h"
#include "eeg.h"
#include "configuration.h"

#if SDCARD
#include "sdcard.h"
#endif

#include <ADS1299.h>
#include <Filters.h>



namespace EEG{
  
thread_t *extractFeaturesEEG = NULL;
thread_t *readSensor1 = NULL;

//Buffer for readings 
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMEEGCHANNELS];


static ADS1299 sensor1(PIN_ADC0_SS,PIN_ADC0_DRDY);
// Filters for the EEG signal
FilterOnePole *filterlp[NUMEEGCHANNELS];
FilterOnePole *filterhp[NUMEEGCHANNELS];

static uint16_t buffersize = sensor1.bufferSize();

// Semaphore for collecting the data in sensor1 buffer
static binary_semaphore_t xsensor1FullSemaphore;

float * data[NUMEEGCHANNELS]; //Pointer to sensor data

static void ISR_NewSample(){
	chSysLockFromISR();  
	if (sensor1.collectData()) {
		chBSemSignalI(&xsensor1FullSemaphore);
	} 
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waSensor1_T, 256);
static THD_FUNCTION(Sensor1_T, arg) {  
  while(1){
    msg_t msg = chBSemWaitTimeout(&xsensor1FullSemaphore, MS2ST(10));
    if (msg == MSG_TIMEOUT) {
      continue;
    }
     
  for(uint8_t chan = 0; chan < NUMEEGCHANNELS; chan++) {      
       data[chan] = sensor1.get_data(chan);            
  }
  
  //This must be faster than the time it takes for filling out a new sensor1 buffer
  for (uint16_t i=0;i<buffersize;i++){
    for(uint8_t chan = 0; chan < NUMEEGCHANNELS;chan++) {
  	  //float value=test;     
  	  float value=data[chan][i];       
      value=filterhp[chan]->input(filterlp[chan]->input(value)); 
      buffer[chan]->append(value);           

      // Write data to sdcard (implemented in sdcard.cpp)
      #if SDCARD
        SDCard::addEEG(value,chan);
      #endif 
    }             
  }
}
}


  
void start(void){ 
    
    sensor1.setNumChannels(NUMEEGCHANNELS);
  
    for(uint8_t chan = 0; chan < NUMEEGCHANNELS; chan++) {              
        buffer[chan]=new ErisBuffer<float>;        
        buffer[chan]->init();      
        filterlp[chan]=new FilterOnePole(HIGHPASS,6.0,0,500);                
        filterhp[chan]=new FilterOnePole(HIGHPASS,200.0,0,500);                
     }
    
    chBSemObjectInit(&xsensor1FullSemaphore,true);    
    
    //Set up ADC chip	
    // Turn the ADS1299 into Continuous read mode
	  
    pinMode(PIN_ADC0_DRDY,INPUT_PULLUP);
    // Attach the interrupt to the DRDY pin defined by the class
    attachInterrupt(digitalPinToInterrupt(PIN_ADC0_DRDY),ISR_NewSample,FALLING);

    sensor1.startCollecting();
    
		// create tasks at priority lowest priority
	  readSensor1=chThdCreateStatic(waSensor1_T, sizeof(waSensor1_T),NORMALPRIO+5, Sensor1_T, NULL);     
	
	}
}
