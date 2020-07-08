#include "Eris.h"
#include "features.h"

#include "serialcommand.h"

#include <FeatureExtractor.h>

namespace Features{

  thread_t *extractFeatures = NULL;
  // Mutex to feature extractor
  mutex_t extractMtx;
  
  typedef FeatureExtractor * FeatureExtractorPtr;
  FeatureExtractorPtr * extractor = new FeatureExtractorPtr[BIOM_NUMCHANNELS];
  
  //The first index is 1 if the requested mask is for the classifier, 0 if for regression. the second specifies the channel, and the third is the index of the requested mask. For classifiers, this  
  //signifies the gait location. For regression, this signifies the ambulation mode. The regression mask for channel 0, ambulation mode 0 is mask[0][0][0] 
  bool mask[2][BIOM_NUMCHANNELS][FEATS_NUMCLASSLOCATIONS][FEATS_NUM];

  /////// Regression Settings
  // Regression is done automatically at a set increment of time
  uint8_t regIdx = 0;//This gets updated at the same time as the regression window and increment, so we need to keep track of it. Used to specify which mask to use.
  uint16_t regWin = 200;
  uint8_t regInc = 50; //increment in ms

  /////// Classification Settings
  // Classification is done based on queries over serial
  uint8_t classIdx = 0; //This gets updated within the SerialCommand query callback

  // Storage variables
  float * features[BIOM_NUMCHANNELS]; //Content of the feature data vector
  float * lastFeatures[BIOM_NUMCHANNELS]; //Content of the feature data vector
  static bool isReady[BIOM_NUMCHANNELS]; //To check if every feature component is ready

  static Packet packet;

  long tempTime = 0;

  // Flags
  bool en_features = false;//Flag that enables features
  volatile bool extractingFeatures = false; // Flag to show if features are being extracted at this moment

  static THD_WORKING_AREA(waExtractFeatures_T, 256);
  static THD_FUNCTION(ExtractFeatures_T, arg) {
    //Thread dedicated to feature extraction to be activated when all channels are ready
    systime_t nextTime = chVTGetSystemTimeX(); // T0
    while(1) {
      nextTime += MS2ST(regInc);

      if(en_features) {
        //wait until mutex can be locked
        chMtxLock(&extractMtx);

        tempTime = micros();
        extractHelper(regWin, 0); //extract features for regression (type 0)
        
        chMtxUnlock(&extractMtx);
      }
      chThdSleepUntil(nextTime);
    }
  }
  
  void extractHelper(uint16_t win, uint8_t type) {
      // Extracts features. The first argument is the window size, second is type (0-regression, 1-classification).

      //Obtain gait location or amb mode index for classification or regression, respectively
      uint8_t idx = (bool)type ? classIdx : regIdx;
      
      for(uint8_t chan = 0; chan < BIOM_NUMCHANNELS; chan++) {
        //This adds the feature vector for the current channel to its spot in the feature array. The array is of size [NUMCHANNELS] X [FEATS_NUM]
        //ExtractAll takes a window size and a bool array of size FEATS_NUM. For example, mask[0][1][2] points to the mask for regression, channel 1, ambulation mode 2. 
        //It returns a pointer to the first element in the computed feature vector
        
        Features::newFeature(extractor[chan]->extractAll(win, mask[type][chan][idx]), chan);
      }
      
      digitalWrite(PIN_LED,!digitalRead(PIN_LED)); //Useful to measure freq
      extractingFeatures = false;
  }

  void newSample(float val, uint8_t chan) {
    extractor[chan]->newSample(val);
  }
  
  void newFeature(float * featPtr, uint8_t channel){
    
    //featPtr is the address of the first component in a feature vector, like the one returned by extractAll. Channel specifies which channel these features belong to. 
    features[channel] = featPtr;
    isReady[channel] = true;

    //check to see if all features are ready
    bool allReady = true;
    for (uint8_t i = 0; i < BIOM_NUMCHANNELS; i++){
      allReady = (allReady & isReady[i]);
    }
    
    if (allReady == true){
      send();
    }
  }
  
  void send(){
    //Send the features in packet form

    if (SerialCom::stream_en == true || SerialCom::classifQuery == true){
      bool type = SerialCom::classifQuery;
      
      //Starts a packet. there are different packet types for classifier and regression.
      packet.start(type ? Packet::PacketType::CLASSIFIER : Packet::PacketType::REGRESSION);

      //Fill in the packet
      for (uint8_t i = 0; i < BIOM_NUMCHANNELS; i++) {
        //Iterates over each channel and appends its respective feature vector depending on the mask
        for (uint8_t j = 0; i < FEATS_NUM; j++) {
          if(mask[(uint8_t)type][i][type ? classIdx : regIdx][j]) {
            packet.append((uint8_t *)&features[i][j], sizeof(float));
          }
        }  
      }
      packet.send();
    }
    
    for (uint8_t i=0;i<BIOM_NUMCHANNELS;i++){
      lastFeatures[i]=features[i];
      isReady[i]=false;
    }
    //Serial.println(lastFeatures[0][6]);//waveform length on index shows window size
    Serial.println(micros() - tempTime);
  }

  void clearExtractors() {
    //Usefull for when we pause feature extraction
    for(uint8_t chan = 0; chan < BIOM_NUMCHANNELS; chan++) {
      extractor[chan]->clear();
    }
  }
  
  void start() {
    
    for (uint8_t i = 0; i < BIOM_NUMCHANNELS; i++) {
      extractor[i] = new FeatureExtractor();
      isReady[i] = false;
    }

    chMtxObjectInit(&extractMtx);
    
    //Initializes the mask so that every value is false. It should be changed to the actual masks using the F_MASK command
    memset(mask, 1, 2*BIOM_NUMCHANNELS*6*FEATS_NUM);

    // create tasks
    extractFeatures=chThdCreateStatic(waExtractFeatures_T, sizeof(waExtractFeatures_T), NORMALPRIO+5, ExtractFeatures_T, NULL);
  }

}
