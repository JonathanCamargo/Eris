#ifndef ERISCOMMON_H
#define ERISCOMMON_H

#include <Arduino.h>

#define MEMBUFFERSIZE 64 // Size for internal data buffers
#define ERRORBUFFERSIZE 10 // Size for internal data buffers

#include "error.h"
#include "buffers.h"
#include "packet.h" 
#include "Print.h"

extern Packet packet;  // Global packet object for handling communication in packets 
// To avoid problems there should only be a single instance of packet?

// Create a global print helper instance


class eriscommon {	
	
	private:
    		
	public:
	static void setPrintPacketMode(bool enable); // Enable or disable packet mode for print function
	
//	void print(const char[]);
	static void printText(const char* text); 		
	static void printText(const __FlashStringHelper *text);		
			
	static void print(const char[]);
    static void println(const char[]);
	
	static void print(const __FlashStringHelper *text);		
	static void println(const __FlashStringHelper *text);		

	static void print(double, int = 2);
	static void println(double, int = 2);



	
};




class PrintHelper : public Print {
	/// Print helper class to simplify printing data in both packet form and standard
	/// form in arduino.
	
	
	private:	
		bool sendTPacket=false; //Global flag to determine if we should packet the prints	
		
	public:
	
		PrintHelper (){};		
		void packetMode(bool enable);  //Set this helper in packet mode (1) or regular text mode (0)
		bool asPacket(); 	           //Check if it is on packet mode (1) or regular text mode(1)    
			
		size_t write(uint8_t a);
		
		void Print(const __FlashStringHelper *);
        //static void print(const String &);
        void Print(const char[]);
        /*
		size_t print(char);
        size_t print(unsigned char, int = DEC);
        size_t print(int, int = DEC);
        size_t print(unsigned int, int = DEC);
        size_t print(long, int = DEC);
        size_t print(unsigned long, int = DEC);
        */
		void Print(double, int = 2);
		
        //size_t print(const Printable&);

        void Println(const __FlashStringHelper *);
        void Println(const char[]);
		/*
        size_t println(const String &s);
        size_t println(const char[]);
        size_t println(char);
        size_t println(unsigned char, int = DEC);
        size_t println(int, int = DEC);
        size_t println(unsigned int, int = DEC);
        size_t println(long, int = DEC);
        size_t println(unsigned long, int = DEC);
        */
		void Println(double, int = 2);
        /*
		size_t println(const Printable&);
        size_t println(void);
		*/
};


#endif




