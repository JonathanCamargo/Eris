#include "FeatureExtractor.h"

FeatureExtractor extractor;

float f;
int counter = 0;

void setup(){
  while (!Serial){  
  }
}


void loop() {
  //int a=analogRead(14); 
  //Serial.println(counter);
  int a=counter;
  counter++;  
  
  if (counter>99)  
  {
     counter= 0;
  }
  
 // Serial.println(a);
  bool ready=extractor.newSample((float) a);
  if (ready){
   f=extractor.WL();
   Serial.print("feature value: ");
   Serial.println(f);
   extractor.clear(); //Hi Breanna! Please don't forget to clear the extractor after you are done computing the features
  }


  delay(10);
}
