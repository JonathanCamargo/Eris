#include <ChRt.h>

#include "fastdualbuffer.h"
#include <ADC.h>
#include <Arduino.h>
#include <IntervalTimer.h>

FastDualBuffer buf;
IntervalTimer timer0;

thread_t *thread1 = NULL;

ADC adc;

char buffer[128];

volatile bool flag=false;
volatile long i=0;

static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  while (1) {
    // Sleep for 1000 milliseconds.
  chThdSleepMilliseconds(5);
    // Toggle pin to show heartbeat    
    //digitalWrite(PIN_LED,!digitalRead(PIN_LED));           
  delay(1);
  if (flag==true){
    Serial.println("aja");
    float *x=buf.read();
    Serial.println((int)x);
    //int num=snprintf(buffer,128,"%1.5f\n",12.5);
    int num=snprintf(buffer,128,"hola gonorreita3 %1.2f",12.5);
    Serial.println(buffer);  
    for (int i=0;i<FASTDUALBUFFERSIZE;i++){
      int num=snprintf(buffer,128,"%1.2f\n",x[i]);
      Serial.print(buffer);
    }    
    flag=false;
    //buf.clear();
  }
  
  //Serial.println("hola");
  }
}


void ISR_NewSample(){
  i++;
  if (i>10000){
    i=1;
  }
  int full=buf.add((float)i);
  if (full>0){
    flag=true;
    Serial.println("trueflag");
  }
  else if(full<0){
    timer0.end();
    Serial.println("ERROR");
  }
  else{
    return;
  }

}

void start(){
  chThdCreateStatic(waThread1, sizeof(waThread1),
                                   NORMALPRIO, Thread1, NULL);
  int test=1;
  test=test+1;
  Serial.println(test);
  int num=snprintf(buffer,128,"hola2 %1.5f",(float)12); // Que mierda
  Serial.println(buffer);
  timer0.begin(ISR_NewSample,1000);
}

void setup() {
  // put your setup code here, to run once:
  adc.setAveraging(4);
  adc.setResolution(10);
  Serial.begin(9600);
  delay(100);
  Serial.println("start");  
  Serial.println(buffer);
  Serial.send_now();  
  chBegin(start); 

     
  while(true){
    
  }
  
}



void loop() {
  // put your main code here, to run repeatedly:
 
  
}
