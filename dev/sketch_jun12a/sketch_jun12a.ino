#include "fastdualbuffer.h"
#include <ADC.h>
#include <IntervalTimer.h>

FastDualBuffer buf;
IntervalTimer timer0;

ADC adc;

char buffer[128];

volatile bool flag=false;
volatile long i=0;
void ISR_NewSample(){
  i++;
  if (i>10000){
    i=1;
  }
  int full=buf.add((float)i);
  if (full>0){
    flag=true;
  }
  else if(full<0){
    timer0.end();
    Serial.println("ERROR");
  }
  else{
    return;
  }

}

void setup() {
  // put your setup code here, to run once:
  adc.setAveraging(4);
  adc.setResolution(10);
  Serial.begin(9600);
  delay(100);
  Serial.println("start");
  Serial.send_now();
  timer0.begin(ISR_NewSample,1000);
}



void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  if (flag==true){
    float *x=buf.read();
    for (int i=0;i<FASTDUALBUFFERSIZE;i++){
      int num=sprintf(buffer,"%1.5f\n",x[i]);
      Serial.print(buffer);
    }
    flag=false;
    buf.clear();
  }
  
}
