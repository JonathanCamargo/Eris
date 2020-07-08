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


//#define DEBUG
#undef BUFFERSIZE
#undef WINDOW
#undef INCREMENT

#define BUFFERSIZE 300
// BUFFERSIZE should be bigger than WINDOW especially if you want to get
// multiple features, so that you ensure enough space to avoid overflow.

#define WINDOW 250
#define INCREMENT 50

class FeatureExtractor {
  public:

  FeatureExtractor();
  FeatureExtractor(uint8_t window, uint8_t increment);
  
  //Send a new value to the extractor
  // Returns true when window is complete
  bool newSample(float value);

  // Features
  float rms();
  float waveformLength();
  float slopeSignChanges();
  float zeroCrossing();
  float minimum();
  float maximum();
  float WL();

  // Clear the window
  bool clear();

  ~FeatureExtractor();

  private:
  CircularBuffer<float,BUFFERSIZE> samples_;
  uint8_t window_;
  uint8_t inc_;

};

#endif
