#ifndef FEATURES_H
#define FEATURES_H

#include <Arduino.h>



namespace Features{

extern float features[FEATURESSIZE];
extern float lastFeatures[FEATURESSIZE];

void send();
void newFeature(float value,uint8_t index);

}

#endif
