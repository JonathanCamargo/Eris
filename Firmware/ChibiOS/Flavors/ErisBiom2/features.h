#ifndef FEATURES_H
#define FEATURES_H

#include <Arduino.h>

namespace Features{

extern float * features[BIOM_NUMCHANNELS];
extern float * lastFeatures[BIOM_NUMCHANNELS];
extern bool en_features;

extern bool mask[2][BIOM_NUMCHANNELS][6][FEATS_NUM];

extern uint8_t regIdx; 
extern uint16_t regWin;
extern uint8_t regInc;

extern uint8_t classIdx;

extern mutex_t extractMtx;
extern binary_semaphore_t xfeaturesFullSemaphore;

void send();
void start();
void newFeature(float* featPtr,uint8_t channel);

void newSample(float val, uint8_t chan);

void extractHelper(uint16_t win, uint8_t type);
void clearExtractors();

}

#endif
