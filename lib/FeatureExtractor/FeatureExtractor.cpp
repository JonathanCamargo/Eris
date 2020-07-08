#include "Arduino.h"
#include "FeatureExtractor.h"
#include <math.h>

//Supported Features:
// Last
// Mean Absolute Value
// Min
// Max
// Std
// RMS
// Waveform length
// Slope sign change
// Zero crossing

//Constructors:
// Create circular buffer

uint8_t FeatureExtractor::numExtractors = 0;

// Use defaults to create the Feature Object
FeatureExtractor::FeatureExtractor(){
}


//Send a new value to the extractor
// Returns true when window is complete user decides what to do
// the library does not enforce any behavior when window size is achieved.
void FeatureExtractor::newSample(float value){
    if(!this->isExtracting) {
        //if we aren't extracting, add to normal buffer
        this->samples_.push(value);
    } else { 
        //if we are currently extracting, add to overflow buffer
        this->ovfBuf[ovfCount] = value;
        this->ovfCount++;
    }
}

//////////////////// Features //////////////////////////////////

//Slope Sign Changes Helper
//Returns 0 or 1 depending on if slope sign has changed
uint8_t FeatureExtractor::ssChangeHelper(float prev, float curr, float next) {
    return (uint8_t)((curr > next && curr > prev) || (curr < next && curr < prev)); 
}

//Zero Crossings Helper
//Returns 0 or 1 depending on if sign has changed
uint8_t FeatureExtractor::zCrossHelper(float prev, float curr) {
    return (uint8_t)(prev*curr < 0); 
}

//Extract All
float * FeatureExtractor::extractAll(uint16_t win, bool mask[]) {

    //Clear last features
    memset(features, 0, sizeof(features));

    //Only extracts features if we have enough samples. This will only be false only at the very 
    //beginning of operation
    if(this->samples_.size() >= win) {
        float val_i = 0;
        float val_prev = 0;
        float val_next = 0;
        float mean = 0; 

        //Let FeatureExtractor know we are currently extracting (add new samples to overflow)
        this->isExtracting = true;

        //Compute mean
        float sum = 0;
        float sumAbs = 0;
        for (uint16_t i = 0; i < win; i++) {
            val_i = this->samples_[this->samples_.size() - i - 1];
            sum += val_i;
            sumAbs += abs(val_i); 
        }
        mean = sum/win;

        if(mask[feat_meanAbs]) {
            features[feat_meanAbs] = sumAbs/win;
        }

        //Initial values for min and max
        features[feat_min] = this->samples_[this->samples_.size() - 1];
        features[feat_max] = this->samples_[this->samples_.size() - 1];

        for (uint16_t i = 0; i < win; i++) {
            // Iterate over the window once to compute as many features as we can simultaneously
            // Only compute the features set by mask
            // Features gets filled 
            val_i = this->samples_[this->samples_.size() - i - 1];

            //These features only depend on current value
            if(mask[feat_min]) {
                features[feat_min] = min(val_i, features[feat_min]);
            } else {
                features[feat_min] = 0; 
            }
            
            if(mask[feat_max]) {
                features[feat_max] = max(val_i, features[feat_max]);
            } else {
                features[feat_max] = 0; 
            }

            if(mask[feat_std]) {
                features[feat_std] += pow(val_i - mean, 2);
            }
            
            if(mask[feat_rms]) {
                features[feat_rms] += pow(val_i, 2); 
            }

            if(mask[feat_wfLength] && i > 0) {
                val_prev = this->samples_[this->samples_.size() - i - 2];
                features[feat_wfLength] += abs(val_i - val_prev); 
            }
            
            //These features depend on current and next/previous values
            if (i < win - 1) { 
                val_prev = this->samples_[this->samples_.size() - i - 2];
                if (i > 0) {
                    if (mask[feat_ssChange]) {
                        val_next = this->samples_[this->samples_.size() - i]; 
                        features[feat_ssChange] += ssChangeHelper(val_prev, val_i, val_next); 
                    }
                }
            
                if (mask[feat_zCross]) {
                     features[feat_zCross] += zCrossHelper(val_prev, val_i);   
                }
            }
        }
        //Complete any remaining calculations
        features[feat_std] = sqrt(features[feat_std]/(win - 1) );
        features[feat_rms] = sqrt(features[feat_rms]/win);

        if(mask[feat_last]) { //add last feature if last is selected
            features[feat_last] = this->samples_[this->samples_.size() - 1];
        }
    }

    for(uint8_t i = 0; i < this->ovfCount; i++) {
        //if any samples were added during extraction, then move them from 
        //overflow buffer to the circular buffer
        this->samples_.push(this->ovfBuf[i]);
    } 
    this->ovfCount = 0;

    this->isExtracting = false;
    return features;
}

//Clear
// Clears the entire window.
void FeatureExtractor::clear(){
    this->samples_.clear();
    //Clear last features
    memset(features, 0, sizeof(features));
}

FeatureExtractor::~FeatureExtractor(void){

}
  
