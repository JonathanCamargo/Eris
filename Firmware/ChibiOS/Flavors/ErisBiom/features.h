#ifndef FEATURES_H
#define FEATURES_H

#include <Arduino.h>
#include <FeatureExtractor.h>
#include <FeaturesHelper.h>

namespace Features{
  
extern bool en_features;

//Settings
extern uint8_t regIdx;
extern uint8_t regInc;
extern uint8_t classIdx;

extern FeaturesHelper RegressionFeatures[FEATURES_NUMREG];
extern FeaturesHelper ClassificationFeatures[FEATURES_NUMCLASS];

void ExtractClassification();

//void RegisterExtractor(FeatureExtractor * extractor, int samplingFrequency_hz); // Register an extractor in

void clearExtractors(); //Clear all the extractors registered in the helpers

bool HelperSelector(FeaturesHelperPtr &featuresHelperPtr, uint8_t index); // Select a feature helper by index saving the pointer in featuresHelperPtr
bool SetHelperIndex(uint8_t helperIdx, uint8_t idx); // Set the index of the helper array specified by helperIdx

void Default(); // Initialize the configuration of helpers, masks and windows to a default state (defined in configuration.h (used to avoid sending configuration commands)

void send();
void start();


}

#endif
