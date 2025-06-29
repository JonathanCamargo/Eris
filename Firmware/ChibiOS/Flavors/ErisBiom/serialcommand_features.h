#ifndef SERIALCOMMAND_FEATURES_H
#define SERIALCOMMAND_FEATURES_H

#include <FeaturesHelper.h>

void EnableFeatures(); // Globally enable feature extraction
void DisableFeatures(); // Globally disable feature extraction
void ClassifQuery();
void ChangeMask(); // Change the mask for a given channel within a FeaturesHelper
bool ChangeMask_parser(FeaturesHelperPtr &featuresHelper,uint8_t & channelIdx, bool * mask);
void ChangeWindow();
void ChangeIdx();
void FeaturesInfo(); // Display all the information of a FeaturesHelper
void Register(); // Register a module's extractor into a FeaturesHelper (basically just calls module's RegisterFeatures function)


#endif
