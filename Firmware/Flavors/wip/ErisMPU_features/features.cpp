#include "Eris.h"
#include "features.h"
#include <FeaturesHelper.h>

#include "fsr.h"
#include "imu.h"

// Features.cpp contains all the functionality for managing the feature extraction process
// FeaturesHelpers contains a list of channels (pointers to different extractors) that can be used in different processes.
// 
// The outline of the operation is as follows:
// 1. Each module in Eris contains feature extraction for every possible channel and on sampling calls the newSample function for 
// any needed extractor. The extractors collect data on their own.
// 2. Extractors(i.e. channels) register to feature helpers using the RegisterExtractor function, and thus the helpers will contain 
// a first-come-first-served list of extractors within. Also define the mask for each channel (this mask defines what features are going to be extracted).
// 3. Call a helper getFeatures function to retrieve the features for every registered channel. Features are computed with masking strategy (see FeatureExtractor). 
// with an specific features helper. 

namespace Features{

  thread_t * extractRegression = NULL;

  FeaturesHelper RegressionFeatures;
  FeaturesHelper ClassificationFeatures;

  static Packet packet;
  bool en_features = false;            //Flag that enables features

  static THD_WORKING_AREA(waExtractRegression_T, 1024);
  static THD_FUNCTION(ExtractRegression_T, arg) {   
      //static systime_t nextTime= chVTGetSystemTimeX(); // T0  
  while(1){
    //nextTime+=MS2ST(FEATURES_REGRESSION_PERIOD_MS); This was giving problems :( Still don't understand
    
    if (en_features){
      float timestamp = ((float)(micros() - t0))/1.0e3;     
      uint8_t numFeatures=RegressionFeatures.numFeatures();      
      float * features=RegressionFeatures.getFeatures();
      // Send the features
      chSysLockFromISR();
      packet.start(Packet::PacketType::REGRESSION);
      packet.append((uint8_t *)&timestamp,sizeof(float));
      packet.append((uint8_t *)&numFeatures,sizeof(uint8_t));
      packet.append((uint8_t *)features, numFeatures*sizeof(float));   
      packet.send();
      chSysUnlockFromISR();
    }
    chThdSleepMilliseconds(FEATURES_REGRESSION_PERIOD_MS);    
  }
  }

  void clearExtractors(){
    RegressionFeatures.clearExtractors();
    ClassificationFeatures.clearExtractors();    
  }

bool HelperSelector(FeaturesHelperPtr &featuresHelperPtr, uint8_t index){
  //Function to select the helper based on index
  switch(index){
        case 0:
          featuresHelperPtr = &RegressionFeatures;
          break;
        case 1:
          featuresHelperPtr = &ClassificationFeatures;
          break;
        default:
          return false;
      }           
      
      return true;
      
}

void Default(){
  #include "features_defaults.cpp" //These should contain the variables used here // in the future we could generate this from a script
    
  //// Initialize the configuration of helpers, masks and windows //to a default state (defined in configuration.h (used to avoid sending configuration commands)
  FeaturesHelperPtr featuresHelperPtr=&RegressionFeatures;  

   uint8_t index=0; // Start with no features registered
   FSR::RegisterExtractors(featuresHelperPtr);  
   for (uint8_t i=index;i<featuresHelperPtr->getNumChannels();i++){
     featuresHelperPtr->setMask(i,default_fsr_mask);
     featuresHelperPtr->setWindow(i,default_fsr_window_ms);
   }
   index=featuresHelperPtr->getNumChannels();
   
   
   IMU::RegisterExtractors(featuresHelperPtr);  
   for (uint8_t i=index;i<featuresHelperPtr->getNumChannels();i++){
     featuresHelperPtr->setMask(i,default_imu_mask);
     featuresHelperPtr->setWindow(i,default_imu_window_ms);
   }
   index=featuresHelperPtr->getNumChannels();

  
   
}

   
  void start(){        
    // create task at normal priority
    extractRegression=chThdCreateStatic(waExtractRegression_T, sizeof(waExtractRegression_T),NORMALPRIO+1, ExtractRegression_T, NULL);
  }
  
}
