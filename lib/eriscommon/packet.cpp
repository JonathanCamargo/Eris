#include "packet.h"
#include <Stream.h>

uint8_t Packet::buffer[PACKET_BUFFERSIZE]; // Memory allocation for storing data

/* Packet is a class to send data in packet form encoded to avoid shifting of bits */
//Constructor
Packet::Packet(){
  this->myPacketSerial.setStream(&Serial);
}

Packet::Packet(Stream * stream){
	this->myPacketSerial.setStream(stream);
}


// Send the data stored in the packet buffer
void Packet::send(){
  this->myPacketSerial.send(this->buffer,this->size);  
  this->size=0;
  this->lock = false;  
}

void Packet::start(Packet::PacketType packetType){
  this->size=0;

  while (this->lock){	 // Make this blocking
	}
	
  this->lock=true;

  switch (packetType){
    case Packet::PacketType::CLASSIFIER:
      write('C');
    break;
    case Packet::PacketType::REGRESSION:
      write('R');
    break;
	case Packet::PacketType::DATA:
      write('D');
    break;
    case Packet::PacketType::TEXT:
      write('T');
    break;	
  } 
}

size_t Packet::write(uint8_t c){
  this->buffer[this->size++]=c;
  return 1;
}

int Packet::available(void){
  return -1;
}

int Packet::peek(void){
  return -1;
}

int Packet::read(void){
  return -1;
}

void Packet::flush(void){
  this->size=0;
}

// Append more binary data to the packet buffer
size_t Packet::append(const uint8_t *buffer, size_t size){
  size_t n = 0;
  while (size--) {
    this->buffer[this->size++]=*buffer++;
    n++;
  }  
  return n;
}
