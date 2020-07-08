#include <sriloadcell.h>

#define CANID 128

SRILoadcell lc(CANID);

#define SERIALPLOTTER

int delayTime = 300;

LoadcellData_t data;

void printData(LoadcellData_t *data){
  Serial.print("forcex:");
  Serial.print(data->forceX);Serial.print(" ");
  Serial.print("forcey:");
  Serial.print(data->forceY);Serial.print(" ");
  Serial.print("forcez:");
  Serial.print(data->forceZ);Serial.print(" ");
  Serial.print("momentx:");
  Serial.print(data->momentX);Serial.print(" ");
  Serial.print("momenty:");
  Serial.print(data->momentY);Serial.print(" ");
  Serial.print("momentz:");
  Serial.println(data->momentZ);
}

void printDataPlotter(LoadcellData_t *data){
  Serial.print(data->forceX);Serial.print("\t");
  Serial.print(data->forceY);Serial.print("\t");
  Serial.print(data->forceZ);Serial.print("\t");
  Serial.print(data->momentX);Serial.print("\t");
  Serial.print(data->momentY);Serial.print("\t");
  Serial.println(data->momentZ);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial){
    
  }
  Serial.println("Before sent first message");
  delay(delayTime);
  
  Can0.begin(1000000);
  Serial.println("Initialized CAN");
  delay(delayTime);

  lc.connect();Serial.println("\nConnected lc");
  delay(delayTime);
}

void loop() {
  if (lc.get_data(&data)){
    #ifdef SERIALPLOTTER
    printDataPlotter(&data);
    #else
    printData(&data);
    #endif
  }
  else{
    Serial.println("error");
  }
  delayMicroseconds(1000);
}
