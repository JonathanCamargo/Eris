#ifndef FSR_H
#define FSR_H

#include "Eris.h"
#include <eriscommon.h>
#include <FeatureExtractor.h>
#include <FeaturesHelper.h>

#include <Arduino.h>

namespace FSR{

extern ErisBuffer<FSRSample_t> buffer; 
extern FeatureExtractorPtr extractor[FSR_NUMCHANNELS];

void RegisterExtractors(FeaturesHelper * featuresHelper);

void start(void);
}

#endif
