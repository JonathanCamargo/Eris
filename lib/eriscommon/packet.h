#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include <PacketSerial.h>
#include <Stream.h>
/*
Packet is a class to help in sending stream data in the form of a packet to avoid shifting of bits
*/

#define PACKET_BUFFERSIZE 1024
/* Packet is a class to send data in packet form encoded to avoid shifting of bits */
class Packet:public Stream
{

  public:

    enum class PacketType{
      DATA,
      CLASSIFIER, 
      REGRESSION, 
      TEXT
    };
    
	Packet(); //Default constructor
	Packet(Stream * stream); //Construct passing a stream (useful for accessing other serial ports
	
	uint8_t lock=false; // Locks the class from creating new packets :)
	
    size_t append(const uint8_t *buffer, size_t size);    
    void test();
    void send();
    void start(Packet::PacketType  packetType);
    //using Print::write;
    virtual size_t write(uint8_t);
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);    
    virtual void flush(void);
        
  private:
    PacketSerial myPacketSerial;
    static uint8_t buffer[PACKET_BUFFERSIZE];
    long size=0;      
};




#endif
