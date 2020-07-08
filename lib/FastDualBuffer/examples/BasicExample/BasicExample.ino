#include <fastdualbuffer.h>


FastDualBuffer<float,100> b ;

void setup(){
  Serial.begin(9600);
  uint8_t algo=b.size();
  delay(1000);
  Serial.println("hola");
  Serial.println(algo);
  
}


volatile float i=0;
void loop(){  
  i=i+1;
  if (b.add(i)){
    float *ptr=b.read();
    uint8_t idx;
    for (idx=0;idx<10;idx++){
         Serial.print(ptr[idx]);
         Serial.print(","); 
    }
    Serial.println(ptr[idx]);
  }
  
  delay(10);
}
