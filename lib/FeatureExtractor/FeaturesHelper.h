#ifndef FEATURES_HELPER_H
#define FEATURES_HELPER_H

#include <FeatureExtractor.h>

#define HELPER_MAX_FEATURES 128 // Maximum number of features that can be grouped in this helper
#define HELPER_MAX_CHANNELS 40 // Maximum number of channels that can be grouped by this helper
#define HELPER_MAX_STR_LEN 5
#define DEFAULT_WINDOW_MS 200 // Default window size in ms on registration of extractors
class FeaturesHelper{  
  
  //Feaures helper knows a set of extractors, masks and  window sizes and manages
  // the feature extraction process.
  // Mask is a boolean array that defines which features are computed (see FeatureExtractor.h mask)
  // Window size is given in milliseconds for constructor but stored in number of samples.
  //
  // Special thanks to James Lewis and Will Flanagan (Sensor fusion) for the base code of masking for the feature extractor.    
  //
  // <jon-cama@gatech.edu>
  
  private:
	uint8_t numchannels; // Number of channels currently registered in this helper
	FeatureExtractor * extractors[HELPER_MAX_CHANNELS];
	float frequency_hz[HELPER_MAX_CHANNELS];
    bool mask[HELPER_MAX_CHANNELS][NUMFEATS];
    uint16_t window[HELPER_MAX_CHANNELS];
	char channelName[HELPER_MAX_CHANNELS][HELPER_MAX_STR_LEN + 1]; // To store a human readable label to the channels
                                                                   // length + 1 for the null character termination 
    
  public:
    
	FeaturesHelper();
    
  
    float * getFeatures(void); //Extract features and compute and return a vector with them
	uint8_t numFeatures(void);
    void RegisterExtractor(FeatureExtractor * extractor, float samplingFrequency_hz,char * chanName=NULL);
	
	void clearExtractors();
	
	uint8_t maskSize(void); // Return the size of masks
	uint8_t getNumChannels(void); // Return the number of registered channels
	
	bool * getMask(uint8_t i); // Return the mask for a given channel
	uint16_t getWindow(uint8_t i); // Return the window size for a given channel
    char * getChannelName(uint8_t i);

    void setMask(uint8_t i,bool * mask);      //Modify the mask for ith extractor
    void setWindow(uint8_t i,uint16_t window_ms);     //Modify the window for ith extractor
    void setAllWindows(uint16_t window_ms); //Modify the window size for all extractors
	
	float features[HELPER_MAX_FEATURES];
	
	
  
  
 
};

typedef FeaturesHelper* FeaturesHelperPtr;


#endif
