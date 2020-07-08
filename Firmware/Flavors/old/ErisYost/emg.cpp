#include "Eris.h"
#include "emg.h"

#include "features.h"
#include "sdcard.h"
#include "configuration.h"


#include <ADC.h>
#include <IntervalTimer.h>

#include <FeatureExtractor.h>

namespace EMG{
  
thread_t *extractFeaturesEMG = NULL;
thread_t *readAnalog = NULL;

time_measurement_t t;

IntervalTimer timer0; // Timer for ADC
ADC *adc = new ADC(); // adc object


float * data[NUMEMGCHANNELS];
static const uint8_t pin_channels[NUMEMGCHANNELS]={PIN_CH0,PIN_CH0,PIN_CH0,PIN_CH0,PIN_CH0,PIN_CH0,PIN_CH0,PIN_CH0};

//Buffer for readings 
ErisBuffer<float> **buffer=new ErisBuffer<float> *[NUMEMGCHANNELS];

//Buffers for features
ErisBuffer<float> **rmsbuffer=new ErisBuffer<float> *[NUMEMGCHANNELS];

//Feature extractor objects
typedef FeatureExtractor* FeatureExtractorPtr;
FeatureExtractorPtr * extractor=new FeatureExtractorPtr[NUMEMGCHANNELS];

static float emgTimestamp;

// Semaphore to feature extractor
static binary_semaphore_t xsamplesSemaphore;
static binary_semaphore_t xfeaturesFullSemaphore;

// Indices and flags

float idx=1;
volatile bool newWindow=false;
static void ISR_NewSample(){    
	chSysLockFromISR();
  newWindow=true;
  chBSemSignalI(&xsamplesSemaphore);	
  emgTimestamp = ((float)(micros() - SDCard::startTime))/1000.0;  
  for(uint8_t chan = 0; chan < NUMEMGCHANNELS; chan++) {
      int value=analogRead(pin_channels[chan]);
      float v=value*3.3/1024;
      v=idx;
      buffer[chan]->append(v);
      #if SDCARD
        SDCard::addEMG(v,emgTimestamp,chan);
      #endif  
      newWindow=(newWindow)&extractor[chan]->newSample(v);      
  }  
  idx=idx+1;
  if (idx>4000){
        idx=0;
  }
  if (newWindow==true){    
    chBSemSignalI(&xfeaturesFullSemaphore);  
  }
  chSysUnlockFromISR();  
}

static THD_WORKING_AREA(waExtractFeaturesEMG_T, 256);
static THD_FUNCTION(ExtractFeaturesEMG_T, arg) {  
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
    for(uint8_t chan = 0; chan < NUMEMGCHANNELS; chan++) {
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
    //chTMStopMeasurementX(&t); 
    
    }
}

void start(void){ 

     for(uint8_t chan = 0; chan < NUMEMGCHANNELS; chan++) {       
        
        buffer[chan]=new ErisBuffer<float>;
        rmsbuffer[chan]=new ErisBuffer<float>;
        extractor[chan]=new FeatureExtractor(250,50);
        
        buffer[chan]->init();
        rmsbuffer[chan]->init();
              
     }
    
    chBSemObjectInit(&xsamplesSemaphore,true);
    chBSemObjectInit(&xfeaturesFullSemaphore,true);
    chTMObjectInit(&t);

    // Timer interrupt to take the ADC samples        
    //Set up ADC
    adc->setAveraging(4); // set number of averages
    adc->setResolution(10); // set bits of resolution   
    timer0.begin(ISR_NewSample, ADC_FREQUENCY_US);    
    
		// create tasks at priority lowest priority	  
    extractFeaturesEMG=chThdCreateStatic(waExtractFeaturesEMG_T, sizeof(waExtractFeaturesEMG_T),NORMALPRIO+5, ExtractFeaturesEMG_T, NULL);    	
	}
}
