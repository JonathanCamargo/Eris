#ifndef FEATURES_H
#define FEATURES_H

#include <Arduino.h>
#include <FeaturesHelper.h>

namespace Features{

extern bool en_features;

extern FeaturesHelper RegressionFeatures;
extern FeaturesHelper ClassificationFeatures;

//void RegisterExtractor(FeatureExtractor * extractor, int samplingFrequency_hz); // Register an extractor in

void clearExtractors(); //Clear all the extractors registered in the helpers

bool HelperSelector(FeaturesHelperPtr &featuresHelperPtr, uint8_t index); // Select a feature helper by index saving the pointer in featuresHelperPtr

void Default(); // Initialize the configuration of helpers, masks and windows to a default state (defined in configuration.h (used to avoid sending configuration commands)

void send();
void start();


}

#endif
