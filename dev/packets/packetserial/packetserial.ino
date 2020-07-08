#include <PacketSerial.h>


PacketSerial myPacketSerial;

void setup(){
  Serial.begin(115200);
  myPacketSerial.setStream(&Serial);
}

char buffer[100]="hola carechimba";
void loop(){
  myPacketSerial.send((uint8_t *)buffer,15);
  delay(250);
}
