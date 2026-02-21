#include "Eris.h"
#include "features.h"

#include "serialcommand.h"

namespace Features{

  float features[FEATURESSIZE]={0,0,0,0,0}; //Content of the feature data vector
  float lastFeatures[FEATURESSIZE]={NAN,NAN,NAN,NAN,NAN}; //Content of the feature data vector
  static bool isReady[FEATURESSIZE]={false,false,false,false,false}; //To check if every feature component is ready

  const float max[FEATURESSIZE]={1,1,1,1,1}; // Features normalized as (x-min)/(max-min) Leave as 1 to avoid normalization
  const float min[FEATURESSIZE]={0,0,0,0,0};  // Features normalized as (x-min)/(max-min) Leave as 0 to avoid normalization

 
  static Packet packet;
  
  void newFeature(float value,uint8_t index){
    //Add a component in the feature vector
    bool allReady=true;
    features[index]=(value-min[index])/(max[index]-min[index]); //Features are normalized
    isReady[index]=true;
    for (uint8_t i=0;i<FEATURESSIZE;i++){
      allReady=(allReady&isReady[i]);
    }
    if (allReady==true){
      send();
    }
  }

  
  void send(){
    //Send the features in packet form

    if (SerialCom::stream_en==true){
    //Start a packet as a feature type
    packet.start(Packet::PacketType::FEAT);
    
    //Fill in the packet
    packet.append((uint8_t *)features,FEATURESSIZE*sizeof(float));   

    //TODO Not sure if I should include a mutex here?
    packet.send();
    }
    
    for (uint8_t i=0;i<FEATURESSIZE;i++){
      lastFeatures[i]=features[i];
      isReady[i]=false;
    }
  }



  
}
