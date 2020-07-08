// Feature extraction Library
/*
Use FeatureExtractor to create an R1 extraction object
Send newSample to the extractor with newSample function
check the return value to find if the window is ready for
extraction. Then use the feature extraction functions to
obtain the features you need. If you are doing relatively
high sampling rate you might want to call the feature extraction
on a new thread.

Don't forget to call the clear function after you get the features.

*/

#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include "Arduino.h"
#include <CircularBuffer.h>


#define BUFFERSIZE 600
// BUFFERSIZE should be bigger than WINDOW especially if you want to get
// multiple features, so that you ensure enough space to avoid overflow.

#define NUMFEATS 9 //number of features we can calculate
#define NUMMAXEXTRACTORS 128

class FeatureExtractor {
  public:

  // Constructors and configuration
  FeatureExtractor();
  
  //Send a new value to the extractor
  // Returns true when increment is complete
  void newSample(float value);

  // Features
  float* extractAll(uint16_t win, bool mask[]);
  uint8_t ssChangeHelper(float prev, float curr, float next);
  uint8_t zCrossHelper(float curr, float next); 

  // Clear the window
  void clear();

  ~FeatureExtractor();

  //Variables
  
  //dictionary enum of feature names
  enum featNames {feat_last, feat_meanAbs, feat_min, feat_max, feat_std, feat_rms, feat_wfLength, feat_ssChange, feat_zCross};
  float features[NUMFEATS]; 

  private:
  static uint8_t numExtractors; 

  CircularBuffer<float,BUFFERSIZE> samples_;
  float ovfBuf[50]; //buffer to handle overflow while we are extracting
  uint8_t ovfCount = 0;
  bool isExtracting = false;
};

typedef FeatureExtractor * FeatureExtractorPtr;
#endif
