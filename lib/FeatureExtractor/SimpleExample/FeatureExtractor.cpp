#include "Arduino.h"
#include "FeatureExtractor.h"

//Included Features:
// WL
// Min
// Max
// SlopeChange
// SignChange

//Constructors:
// Create circular buffer
// assign private members or default window and increment



// Use defaults to create the Feature Object
FeatureExtractor::FeatureExtractor(){
	this->window_=WINDOW;
	this->inc_=INCREMENT;
  
}

FeatureExtractor::FeatureExtractor(uint8_t window, uint8_t inc){
	this->window_=window;
	this->inc_=inc;
}

  //Send a new value to the extractor
  // Returns true when window is complete user decides what to do
  // the library does not enforce any behavior when window size is achieved.
  bool FeatureExtractor::newSample(float value){
	  if (!this->samples_.push(value)){
		Serial.println("Critical error");
	  }
	  if (this->samples_.size()>=this->window_){
		return true;
	  }
	  else{
		  return false;
	  }

  }

  // Features

//////////
//RMS
/////////
  float FeatureExtractor::rms(){
	  float cumsum=0;
	  if (this->samples_.size()<this->window_){
		  //Not enough data to compute features do nothing and return NaN
		return NAN;
	  }
	  for (uint8_t i=0;i<this->window_;i++){
		  cumsum=cumsum + pow(this->samples_[i],2);
	  }
	  return sqrt(cumsum/this->window_);
  }

/////////
//Minimum
/////////
float FeatureExtractor::minimum(){
if (this->samples_.size()<this->window_){
      //Not enough data to compute features do nothing and return NaN
    return NAN;
    }   
    float minV = samples_[0];
    for(uint8_t ii = 1; ii < this-> window_; ii++)
    {
        if(samples_[ii] < minV)
        {
          minV = samples_[ii];
        }      
    }
return(minV);
}

/////////
//Maximum
/////////
float FeatureExtractor::maximum(){
if (this->samples_.size()<this->window_){
      //Not enough data to compute features do nothing and return NaN
    return NAN;
    }   

    float maxV = samples_[0];
    for(uint8_t ii = 1; ii < this-> window_; ii++)
    {
        if(samples_[ii] > maxV)
        {
          maxV = samples_[ii];
        }  
    }
return(maxV);
}

////////////////////
//Waveform Length
////////////////////

float FeatureExtractor::WL(){
 float cumLength = 0;

if (this->samples_.size()<this->window_){
      //Not enough data to compute features do nothing and return NaN
    return NAN;}

for(uint8_t ii = 1; ii < this-> window_; ii++)
    {       
      float change =  (this->samples_[ii]-this->samples_[ii-1]);
      change = abs(change);
      cumLength += change;
    }
return(cumLength);

}

//////////////////////
//Slope Sign Changes
//////////////////////

float FeatureExtractor::slopeSignChanges()
{   
    if (this->samples_.size()<this->window_){
		  //Not enough data to compute features do nothing and return NaN
		return NAN;
    }   
   
    
    uint8_t signCounter = 0;
    for(uint8_t ii = 1; ii < this-> window_-1; ii++)
    {
        //The following conditions check if a specific sample respresents a slope change based on the numbers next to it; 

        bool condition1 = this->samples_[ii] > this->samples_[ii+1];
        bool condition2 = this->samples_[ii] > this->samples_[ii-1];
        bool condition3 = this->samples_[ii] < this->samples_[ii+1];
        bool condition4 = this->samples_[ii] < this->samples_[ii-1];
        
        //In the paper they had another condition that tested a condition in comparision to a specific threshold
        //I am asking will about the threshold contion for our specific usage but the 
        //threshold condition would be :
        //bool thresh1 = abs(samples[ii]-samples[++ii]) => threshold
        //bool thresh2 = abs(samples[ii]-samples[--ii]) => threshold
        if((condition1&&condition2)==true || (condition3&&condition4) == true )// && thresh1||thresh2 == true)
        {
            ++signCounter;
            
        }
    }

    return(signCounter);
}

/////////
//Zero Crossings
/////////

float FeatureExtractor::zeroCrossing()
{   
  if (this->samples_.size()<this->window_){
      //Not enough data to compute features do nothing and return NaN
    return NAN;
    }   

    
    uint8_t zeroCounter=0;
    for(uint8_t ii=0; ii<this->window_-1; ii++)
    {
     //the following conditions check to see if a specific sample represents a zero crossing based on the signs in the adjacent samples.
        bool condition1 = this->samples_[ii] > 0;
        bool condition2 = this->samples_[ii+1] <= 0;
        bool condition3 = this->samples_[ii] < 0;
        bool condition4 = this->samples_[ii+1] >= 0;
        //threshold condition would be :
        //bool thresh = abs(samples[ii]-samples[++ii]) => threshold
        
       
        

        
        
        if((condition1&&condition2==true) || (condition3&&condition4 == true) )// && thresh == true)
        {
            ++zeroCounter;
        }
    }
    return (float)zeroCounter;
}

/////////
//Clear
/////////

  // Clear the window. This is removing the first inc samples
  // Not the entire window, since they overlap by inc.
  bool FeatureExtractor::clear(){
	  if (this->samples_.size()<this->inc_){
		  // Not enough data to clear do nothing
		  return false;
	  }
	  for (uint8_t i=0; i<this->inc_;i++){		  
		  this->samples_.shift();
	  }
	  return true;
  }

  FeatureExtractor::~FeatureExtractor(void){
  }
  
