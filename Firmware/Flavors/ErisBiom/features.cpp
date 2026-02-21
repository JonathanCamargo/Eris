#include "Eris.h"
#include "features.h"
#include <FeaturesHelper.h>

#include "biom.h"

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

  // Regression Settings
  uint8_t regIdx = 0; // index for which regression helper
  uint8_t regInc = 50; // slide increment in ms

  // Classification Settings
  uint8_t classIdx = 0;

  FeaturesHelper RegressionFeatures[FEATURES_NUMREG]; //FEATURES_NUMREGS defined in configuration.h
  FeaturesHelper ClassificationFeatures[FEATURES_NUMCLASS];

  static Packet packet;
  bool en_features = false;            //Flag that enables features

  static THD_WORKING_AREA(waExtractRegression_T, 1024);
  static THD_FUNCTION(ExtractRegression_T, arg) {   
    //static systime_t nextTime= chVTGetSystemTimeX(); // T0  
    while(1){
      //nextTime+=MS2ST(FEATURES_REGRESSION_PERIOD_MS); This was giving problems :( Still don't understand
      
      if (en_features){
        digitalWrite(PIN_LED, HIGH); 
        float timestamp = ((float)(micros() - t0))/1.0e3;     
        uint8_t numFeatures=RegressionFeatures[regIdx].numFeatures();      
        float * features=RegressionFeatures[regIdx].getFeatures();
        // Send the features
        chSysLockFromISR();
        packet.start(Packet::PacketType::REGRESSION);
        packet.append((uint8_t *)&timestamp,sizeof(float));
        packet.append((uint8_t *)&numFeatures,sizeof(uint8_t));
        packet.append((uint8_t *)features, numFeatures*sizeof(float));   

        
        //Serial.print(features[0]);
        //Serial.print(',');
        //Serial.println(features[1]);
        
        
        packet.send();
        
        chSysUnlockFromISR();
        digitalWrite(PIN_LED, LOW);
      }
      chThdSleepMilliseconds(regInc);    
    }
  }

  void ExtractClassification() {
    if (en_features){
      
      float timestamp = ((float)(micros() - t0))/1.0e3;     
      uint8_t numFeatures=ClassificationFeatures[classIdx].numFeatures();      
      float * features=ClassificationFeatures[classIdx].getFeatures();
      // Send the features
      chSysLockFromISR();
      packet.start(Packet::PacketType::CLASSIFIER);
      packet.append((uint8_t *)&timestamp,sizeof(float));
      packet.append((uint8_t *)&numFeatures,sizeof(uint8_t));
      packet.append((uint8_t *)features, numFeatures*sizeof(float));

      //Serial.print(features[0]);
      //Serial.print(',');
      //Serial.println(features[1]);

      packet.send();
      
      chSysUnlockFromISR();
    }
  }

  void clearExtractors(){
    for(uint8_t idx; idx < FEATURES_NUMREG; idx++) {
      RegressionFeatures[idx].clearExtractors();
    }
    for(uint8_t idx; idx < FEATURES_NUMCLASS; idx++) {
      ClassificationFeatures[idx].clearExtractors();  
    }  
  }

  bool HelperSelector(FeaturesHelperPtr &featuresHelperPtr, uint8_t index){
    // Function to select the helper array based on index
    // Returns the helper within that array that is at the current index (regIdx or classIdx)
    switch(index){
      case 0:
        featuresHelperPtr = &RegressionFeatures[regIdx];
        break;
      case 1:
        featuresHelperPtr = &ClassificationFeatures[classIdx];
        break;
      default:
        return false;
    }
    return true;     
  }

bool SetHelperIndex(uint8_t helperIdx, uint8_t idx) {
  // if helperIdx and idx are valid:
  // helperIdx is which helper array
  // sets the index for this helper array to idx
  uint8_t num;
  switch(helperIdx){
      case 0:
        if(idx < FEATURES_NUMREG) {
          regIdx = idx;
        } else {
          return false;
        }
        break;
      case 1:
        if(idx < FEATURES_NUMCLASS) {
          classIdx = idx;
        } else {
          return false;
        }
        break;
      default:
        return false;
    }
    return true;
}

void Default(){

  #include "features_defaults.cpp" //These should contain the variables used here // in the future we could generate this from a script
  regInc = default_biom_reg_inc_ms;
  

  //Regression
  for (uint8_t idx; idx < FEATURES_NUMREG; idx++) {
    //// Initialize the configuration of helpers, masks and windows //to a default state (defined in configuration.h (used to avoid sending configuration commands)
    FeaturesHelperPtr featuresHelperPtr=&RegressionFeatures[idx];
  
     uint8_t index=0; // Start with no features registered
     Biom::RegisterExtractors(featuresHelperPtr);
     for (uint8_t i=index;i<featuresHelperPtr->getNumChannels();i++){
       featuresHelperPtr->setMask(i,default_biom_mask);
       featuresHelperPtr->setWindow(i,default_biom_reg_win_ms);
     }
     index=featuresHelperPtr->getNumChannels();
  }

  //Classification
  for (uint8_t idx; idx < FEATURES_NUMCLASS; idx++) {
    //// Initialize the configuration of helpers, masks and windows //to a default state (defined in configuration.h (used to avoid sending configuration commands)
    FeaturesHelperPtr featuresHelperPtr=&ClassificationFeatures[idx];
  
     uint8_t index=0; // Start with no features registered
     Biom::RegisterExtractors(featuresHelperPtr);
     for (uint8_t i=index;i<featuresHelperPtr->getNumChannels();i++){
       featuresHelperPtr->setMask(i,default_biom_mask);
       featuresHelperPtr->setWindow(i,default_biom_class_win_ms);
     }
     index=featuresHelperPtr->getNumChannels();
  }
}

   
  void start(){

    //Create FeaturesHelpers
    for (uint8_t idx; idx < FEATURES_NUMREG; idx++) {
      //RegressionFeatures[idx] = new FeaturesHelper();
    }
    for (uint8_t idx; idx < FEATURES_NUMCLASS; idx++) {
      //ClassificationFeatures[idx] = new FeaturesHelper();
    }
    
    // create task at normal priority
    extractRegression=chThdCreateStatic(waExtractRegression_T, sizeof(waExtractRegression_T),NORMALPRIO+1, ExtractRegression_T, NULL);
  }
  
}
