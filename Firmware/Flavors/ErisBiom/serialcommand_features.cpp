#include "Eris.h"
#include "serialcommand.h"
#include "serialcommand_features.h"

#include "biom.h"

#include "features.h"
#include <FeaturesHelper.h>

void EnableFeatures(){  
  Features::en_features =true;
}

void DisableFeatures(){
  Features::en_features =false;
  Features::clearExtractors();
}

void Register(){
  // Register all modules' extractors into a FeaturesHelper (basically just calls module's RegisterFeatures function)
  // e.g. F_R 0 FSR IMU //Registers the 2 modules in that order
  char* args[8]; //max possible modules in eris
  uint8_t index = 0;
  do {
    args[index] = SerialCom::sCmd.next();     
    index++; 
  } while (index < 8 && *args[index-1]!=NULL );
  index--;
 
  if (index<2){
    Error::RaiseError(COMMAND,(char *)"No arguments were given");
    return;
  }
  FeaturesHelperPtr featuresHelperPtr;
  // First argument is the helper index
  uint8_t helperIdx=atoi(args[0]);  
  bool isgood=Features::HelperSelector(featuresHelperPtr,helperIdx);
    
  if (!isgood){
    Error::RaiseError(COMMAND,(char *)"REG");
    return;
  }
  // following arguments are the modules to be included in the registering process

  for ( uint8_t i=1; i<index; i++){
    char *arg=args[i];    
    if (!strncmp("BIOM",arg,MAXSTRCMP)){
      Biom::RegisterExtractors(featuresHelperPtr);
      #if DEBUG
      Serial.println("Biom registered");
      #endif      
    }
    else {      
      #if DEBUG
      Serial.print(arg);
      Serial.println(" module not found");
      #endif      
    }
  } 
}

// Set the index for a specified helper array
// 2 args - first is which array(given to HelperSelector, second is the index of that array
void ChangeIdx() {
  uint8_t * idxVar;
  
  char * arg = SerialCom::sCmd.next();
  if(arg != NULL) {
    uint8_t helperIdx = (uint8_t)atoi(arg);
    arg = SerialCom::sCmd.next();
    if(arg != NULL) {
      uint8_t idx = (uint8_t)atoi(arg);
      if(!Features::SetHelperIndex(helperIdx, idx)) {
        eriscommon::printText("Index not valid");
      }
    }
  }
}

void FeaturesInfo(){
  
  FeaturesHelperPtr featuresHelperPtr;

  // Retrieve the featureHelper pointer 
  char *arg=SerialCom::sCmd.next();
  if (arg==NULL){
    Error::RaiseError(COMMAND,(char *)"helper not found");
    return;
  }
  
  // First argument is the helper index
  uint8_t helperIdx=atoi(arg);
  bool isgood=Features::HelperSelector(featuresHelperPtr,helperIdx);
    
  if (!isgood){
    Error::RaiseError(COMMAND,(char *)"helper not found");
    return;
  }
  
  Serial.print("Features helper info: Array ");
  Serial.print(helperIdx);
  Serial.print(" Index ");
  Serial.println((helperIdx == 0) ? Features::regIdx : Features::classIdx);
  Serial.println("--------------------");
  // Display number of channels
  uint8_t numchannels=featuresHelperPtr->getNumChannels();
  uint8_t numfeats=featuresHelperPtr->maskSize();
  
  for (uint8_t i=0;i<numchannels;i++){
    bool * mask=featuresHelperPtr->getMask(i);
    Serial.print("chan(");
    Serial.print(i);
    Serial.print(") ");
    Serial.print("[");
    Serial.print(featuresHelperPtr->getChannelName(i));
    Serial.print("]:");
    Serial.print(featuresHelperPtr->getWindow(i));
    Serial.print(" ");
    for(uint8_t j=0;j<numfeats;j++){
       Serial.print(mask[j],DEC);       
    }
    Serial.println("");
  }
  Serial.print("Total features: ");
  Serial.print(featuresHelperPtr->numFeatures());
  Serial.println("");
  Serial.println("--------------------");    
}

//This handles a classification query. When this function is called, the teensy will send a a packet containing features for classification. 
//Takes 1 arg to specificy gait location (ClassificationHelper index)
void ClassifQuery() {
  //Send features packet
  char* arg = SerialCom::sCmd.next();
  uint8_t idx;
  if (arg != NULL) {
    idx = (uint8_t)atoi(arg);
    Features::classIdx = idx; //set gait location index
    Features::ExtractClassification();//extract features for specified window size, type 1 (classification)
  }
}

//Set a single mask at a time
// Takes 3 arguments: F_MASK <helperIdx> <channelIdx> <boolmask>
// Example: "F_MASK 0 3 1000000000" sets the feature mask for helper 0 (e.g. regression at initial bin), channel 3 to  "true false false false false false false false false"
// Include ambulation mode?
void ChangeMask(){
  FeaturesHelperPtr featuresHelperPtr;
  uint8_t channelIdx;
  bool mask[10];
  bool isgood=ChangeMask_parser(featuresHelperPtr,channelIdx,mask);
  if (isgood){    
    featuresHelperPtr->setMask(channelIdx,mask);
  }
  else{
    Error::RaiseError(COMMAND,(char *)"F_MASK");
  }
}
  
bool ChangeMask_parser(FeaturesHelperPtr &featuresHelperPtr,uint8_t & channelIdx, bool * mask) {
  // Read the serial command to obtain the helper pointer the channelidx and the mask
  char* args[3];     
  uint8_t index = 0; 

  bool validArgs=false;
  
  do {
    args[index] = SerialCom::sCmd.next(); 
    index++; 
  } while (index < 3 && *args[index-1]!=NULL);

  // This can be improved but for now just hardcode the helper to the index
  uint8_t helperIdx=atoi(args[0]);
  bool isgood=Features::HelperSelector(featuresHelperPtr,helperIdx);
  if (!isgood){    
    return false;
  }   

  channelIdx=atoi(args[1]);
  
  if (args[2] != NULL && strlen(args[2]) == featuresHelperPtr->maskSize()) {           
    for (uint8_t i = 0; i < featuresHelperPtr->maskSize(); i++) {  
      mask[i]=(bool) ((args[2][i] - '0')!=0);     
    }
  } else {
    Error::RaiseError(COMMAND,(char *)"check mask size");   
    return validArgs;
    }
    validArgs=true;
  return validArgs;
}

void ChangeWindow() {
  //Takes in the helper index and a window value
  // If the helper is value, sets that helper's 
  char * arg = SerialCom::sCmd.next();
  if(arg != NULL) {
    uint8_t helperIdx = (uint8_t)atoi(arg);
    
    FeaturesHelperPtr featuresHelperPtr;

    if(Features::HelperSelector(featuresHelperPtr,helperIdx)) {
      arg = SerialCom::sCmd.next();
      if(arg != NULL) {
        uint16_t win = (uint16_t)atoi(arg);
        featuresHelperPtr->setAllWindows(win);
      }
    } else {
      Serial.println('Helper array not found');
    }
  }
}
