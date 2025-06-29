#include "Eris.h"
#include "imu.h"

#include "features.h"
#include "sdcard.h"
#include "configuration.h"


#include <yostimu.h>
#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace IMU{
  
thread_t *extractFeaturesIMU = NULL;
thread_t *readIMU = NULL;

//time_measurement_t t;

IntervalTimer timer0; // Timer for ADC
//ADC *adc = new ADC(); // adc object

YostIMU imu0(PIN_SS0);
static uint8_t spibuffer[64];


float * data[NUMIMUCHANNELS];
static const uint8_t pin_channels[NUMIMUCHANNELS]={PIN_SS0};//,PIN_SS0,PIN_SS0};


//Buffer for readings 
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMIMUCHANNELS];

//Feature extractor objects
typedef FeatureExtractor* FeatureExtractorPtr;
FeatureExtractorPtr * extractor=new FeatureExtractorPtr[NUMIMUCHANNELS];

static float imuTimestamp;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;
static binary_semaphore_t xfeaturesFullSemaphore;

// Indices and flags

float idx=1;
volatile bool newWindow=false;
static void ISR_NewSample(){    
	chSysLockFromISR();
  newWindow=true;  
  imuTimestamp = ((float)(micros() - SDCard::startTime))/1000.0;  

  // Send commands to read IMU 
  imu0.WriteCommand(GET_ALL);
  // Cannot use DMA during the interrupt :( So just semaphore
  delayMicroseconds(8);  
  
  chBSemSignalI(&xsamplesSemaphore);  
  chSysUnlockFromISR();  
}


static THD_WORKING_AREA(waReadIMU_T, 128);
static THD_FUNCTION(ReadIMU_T, arg) {
  // Thread dedicated to read the IMU data
  while(1){       
    msg_t msg = chBSemWaitTimeout(&xsamplesSemaphore, MS2ST(10));
    if (msg == MSG_TIMEOUT) {
      continue;
    }     
    digitalWrite(8,!digitalRead(8)); 
    imu0.Read(spibuffer,9*sizeof(float));
    while(!imu0.DataReady()){
      //digitalWrite(8,!digitalRead(8));      
      //delayMicroseconds(1);
      chThdSleepMilliseconds(2);
    }
    digitalWrite(8,!digitalRead(8)); 
    float x[9];
    imu0.ReadFloats((uint8_t *)spibuffer,(float*)x,9);     
    buffer[0]->append(x[0]);
    //Serial.println(x[0]);

  
    //for(uint8_t chan = 0; chan < NUMIMUCHANNELS; chan++) {
        //int value=analogRead(pin_channels[chan]);
        //float v=value*3.3/1024;
        //v=idx;
        //buffer[chan]->append(v);
        //#if SDCARD
        //  SDCard::addIMU(v,imuTimestamp,chan);
        //#endif  
        //newWindow=(newWindow)&extractor[chan]->newSample(v);      
    //}  
  /*newWindow=false;
  idx=idx+1;
  if (idx>4000){
        idx=0;
  }
  if (newWindow==true){    
    chBSemSignalI(&xfeaturesFullSemaphore);  
  }
  */
  }
}


static THD_WORKING_AREA(waExtractFeaturesIMU_T, 256);
static THD_FUNCTION(ExtractFeaturesIMU_T, arg) {  
  //Thread dedicated to EMG feature extraction to be activated by featuresFullSemaphore
  // Here features are extracted depending on which channel of emg.
  // Then features are send to be collected at the Features::features vector where 
  // the index of the new feature must be consistent with the feature vector component.
  while(1){       
    msg_t msg = chBSemWaitTimeout(&xfeaturesFullSemaphore, MS2ST(10) );
    if (msg == MSG_TIMEOUT) {
      continue;
    }
    //chTMStartMeasurementX(&t); 
    // Extract features depending on which channel the index of the new feature must be consistent with
    // the feature vector component definition.
    for(uint8_t chan = 0; chan < NUMIMUCHANNELS; chan++) {
      switch (chan){
      float x;
      case 0:
         x=extractor[chan]->zeroCrossing();
         Features::newFeature(x,0);         
        break;
      case 1:
        x=extractor[chan]->rms();
        Features::newFeature(x,1);
        x=extractor[chan]->zeroCrossing();
        Features::newFeature(x,2);
        x=extractor[chan]->slopeSignChanges();
        Features::newFeature(x,3);
        x=extractor[chan]->WL();
        Features::newFeature(x,4);
        break;
      //case 2:
      //  x=extractor[chan]->zerocrossing();
      //  Features::newFeature(x,3);
      //  break;
      default:
        break;
    }             
        extractor[chan]->clear(); // Don't forget to clear the extractors to slide the window!
    }    
    digitalWrite(PIN_LED,!digitalRead(PIN_LED)); //Usefull to measure freq
    chTMStopMeasurementX(&t); 
    
    }
}

void start(void){ 

     for(uint8_t chan = 0; chan < NUMIMUCHANNELS; chan++) {       
        
        buffer[chan]=new ErisBuffer<float>;        
        extractor[chan]=new FeatureExtractor(250,50);
        
        buffer[chan]->init();       
              
     }
    
    chBSemObjectInit(&xsamplesSemaphore,true);
    chBSemObjectInit(&xfeaturesFullSemaphore,true);
    chTMObjectInit(&t);

    // Timer interrupt to take the ADC samples        
    timer0.begin(ISR_NewSample, IMU_FREQUENCY_US);    
    
		// create tasks at priority lowest priority	  
    readIMU=chThdCreateStatic(waReadIMU_T, sizeof(waReadIMU_T),NORMALPRIO+5, ReadIMU_T, NULL);      
    extractFeaturesIMU=chThdCreateStatic(waExtractFeaturesIMU_T, sizeof(waExtractFeaturesIMU_T),NORMALPRIO+5, ExtractFeaturesIMU_T, NULL);    	
    
	}
}
