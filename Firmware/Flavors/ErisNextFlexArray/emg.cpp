#include "Eris.h"
#include "emg.h"
#include "sdcard.h"
#include "serialcommand.h"
#include "configuration.h"

#include <fastdualbuffer.h>
#include <TimerOne.h>
#include <ADS1256.h>

namespace EMG{

thread_t *extractFeaturesEMG = NULL;
thread_t *readSensor1 = NULL;

//Buffer for readings 
ErisBuffer<float> ** buffer=new ErisBuffer<float> *[NUMEMGCHANNELS]; //Buffer holds emg data ch0 to chi
ErisBuffer<float> buffer_timestamps; //Buffer holds timestamp of ch0 only

ADS1256 sensor1(PIN_ADC0_SS,PIN_ADC0_DRDY, ADS1256_15000SPS);

volatile bool bufferFlag=false; // Use as a flag to enable buffer reading from the main loop.
bool wasFull=false; // Use as a flag to determine if there is buffer overflow.

// Semaphore to feature extractor
static binary_semaphore_t xsensor1FullSemaphore;

// Each emg channel has independent timestamps
static FastDualBuffer<float,ADS1256_BUFFERSIZE> *EMGTimestamps[NUMEMGCHANNELS];
static FastDualBuffer<> *anotherbuffer[NUMEMGCHANNELS];


float * EMGDataBuffersPtr[NUMEMGCHANNELS]; // Pointers to sensor1 internal data buffers
float * EMGTimestampsPtr[NUMEMGCHANNELS]; // Pointers to the timestamps

uint8_t currChannel=0;


static void ISR_NewSample(){
	chSysLockFromISR();
  //Reading is sequential ch0,ch1,ch2,... so on.
  // right now we are reading currChannel  
  EMGTimestamps[currChannel]->add(((float)(micros() - SerialCom::startTime))/1.0e6);
    
  if (sensor1.collectData()) {
     //This thread should wakeup before the second buffer in sensor1 fills up make sure this happens!
     //You can serialprint the pointer to the data here and in the thread confirm that they are the same.
     // on debugging. 
     chBSemSignalI(&xsensor1FullSemaphore); 
	} 
  currChannel=currChannel+1;
  currChannel=(currChannel>=NUMEMGCHANNELS) ? 0:currChannel;
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waSensor1_T, 256);
static THD_FUNCTION(Sensor1_T, arg) {
  while(1){
    msg_t msg = chBSemWaitTimeout(&xsensor1FullSemaphore, MS2ST(10)); 
    if (msg == MSG_TIMEOUT) { 
      continue;
    } 
  // Retrieve pointers for where the info is stored
  for(uint8_t chan = 0; chan < NUMEMGCHANNELS; chan++) {   
       EMGDataBuffersPtr[chan] = sensor1.get_data(chan);
       EMGTimestampsPtr[chan] = EMGTimestamps[chan]->read();
  }
  
  for (uint16_t i=0;i<ADS1256_BUFFERSIZE;i++){
    float timestamp=EMGTimestampsPtr[0][i]; 
    buffer_timestamps.append(timestamp);
    for(uint8_t chan = 0; chan < NUMEMGCHANNELS;chan++) {       
       float value=EMGDataBuffersPtr[chan][i]; 
        buffer[chan]->append(value);    
        #if SDCARD
        SDCard::addEMG(value, EMGTimestampsPtr[chan][i], chan);
        #endif     
    }
    }
	}  
}

  
void start(void){ 
    
    sensor1.setNumChannels(NUMEMGCHANNELS);
        
    chBSemObjectInit(&xsensor1FullSemaphore,true);
    buffer_timestamps.init();
    for (uint8_t emgChan=0;emgChan<NUMEMGCHANNELS;emgChan++){
      buffer[emgChan]=new ErisBuffer<float> ();
      buffer[emgChan]->init();
      EMGTimestamps[emgChan]=new FastDualBuffer<float,ADS1256_BUFFERSIZE>;
    }
    
    //Set up ADC chip	
    // Turn the ADS1256 into Continuous read mode
    pinMode(PIN_ADC0_DRDY,INPUT_PULLUP);    
    // Attach the interrupt to the DRDY pin defined by the class
    attachInterrupt(digitalPinToInterrupt(PIN_ADC0_DRDY),ISR_NewSample,FALLING);
    sensor1.startCollecting();

		// create tasks at priority lowest priority
	  readSensor1=chThdCreateStatic(waSensor1_T, sizeof(waSensor1_T),NORMALPRIO, Sensor1_T, NULL);  
	}

}
