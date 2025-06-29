#ifndef BIOM_H
#define BIOM_H

#include "Eris.h"
#include <eriscommon.h>
#include <FeatureExtractor.h>
#include <FeaturesHelper.h>

#include <Arduino.h>

namespace Biom{

  extern ErisBuffer<BiomSample_t> buffer;
  
  extern FeatureExtractorPtr extractor[BIOM_NUMCHANNELS];

  void RegisterExtractors(FeaturesHelper * featuresHelper);
  
  void start(void);
}

#endif
