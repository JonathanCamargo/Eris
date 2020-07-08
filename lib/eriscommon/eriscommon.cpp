#include "eriscommon.h"
#include "packet.h"
#include <Arduino.h>

PrintHelper p; // global print helper instance
Packet packet(&Serial); // Use regular serial port for communication

void eriscommon::setPrintPacketMode(bool enable){		
	p.packetMode(enable);
}
void eriscommon::printText(const char* text) { 
//Kept for compatibility with old versions of eris
#warning "this function will be deprecated in the next release"
	eriscommon::print(text);
}

void  eriscommon::printText(const __FlashStringHelper *text)
{
	#warning "this function will be deprecated in the next release"
	
}

void eriscommon::print(const __FlashStringHelper *text){
	p.Print(text);
}

void eriscommon::print(const char * text){
	    p.Print(text);
}

void eriscommon::print(double f, int dec){
	    p.Print(f,dec);
}

void eriscommon::println(const __FlashStringHelper *text){
	p.Println(text);
}

void eriscommon::println(const char * text){
	    p.Println(text);
}

void eriscommon::println(double f, int dec){
	    p.Println(f,dec);
}


void PrintHelper::packetMode(bool enable) {
	sendTPacket = enable; 
}

bool PrintHelper::asPacket() {
	return sendTPacket; 
}


size_t PrintHelper::write(uint8_t a){
	if(!sendTPacket){ 
		Serial.write(a);
	}
	else{			
		packet.append(&a,sizeof(uint8_t));
	}	
	return 	0;
}	


void PrintHelper::Print(const __FlashStringHelper * ptr){
	    if(sendTPacket){ 
			packet.start(Packet::PacketType::TEXT);
		}
		this->print(ptr);	
		if(sendTPacket){ 
			packet.send();
		}		
}


void PrintHelper::Print(const char* s){
	    if(sendTPacket){ 
			packet.start(Packet::PacketType::TEXT);
		}
		this->print(s);			
		if(sendTPacket){ 
			packet.send();
		}	
}


void PrintHelper::Print(double f, int dec){
	    if(sendTPacket){ 
			packet.start(Packet::PacketType::TEXT);
		}
		this->print(f,dec);			
		if(sendTPacket){ 
			packet.send();
		}	
}


void PrintHelper::Println(const char* s){
	    if(sendTPacket){ 
			packet.start(Packet::PacketType::TEXT);
		}
		this->println(s);			
		if(sendTPacket){ 
			packet.send();
		}	
}

void PrintHelper::Println(double f, int dec){
	    if(sendTPacket){ 
			packet.start(Packet::PacketType::TEXT);
		}
		this->println(f,dec);			
		if(sendTPacket){ 
			packet.send();
		}	
}

void PrintHelper::Println(const __FlashStringHelper *text){
	    if(sendTPacket){ 
			packet.start(Packet::PacketType::TEXT);
		}
		this->println(text);			
		if(sendTPacket){ 
			packet.send();
		}	
}


