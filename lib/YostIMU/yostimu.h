// YostIMU Teensy Library for use with NextFlex sensor
/*
To use this library, put include path and include "YostIMU.h". The
class "YostIMU" contains all functions necessary to opperate the YostIMU with
Teensy. To use the object, Initialize an YostIMU object with the Teensy CS pin
connected to the imu. 

To start collecting data, call the "startCollecting" function for the object to
put the yost chip in collecting mode call "stopCollecting" to turn the YostIMU back to off mode.
*/

#ifndef YostIMU_teensy_lib_v2_h
#define YostIMU_teensy_lib_v2_h

#include "Arduino.h"
#include <SPI.h>
#include <string.h>

#define USE_DMA true
#define DEBUG true
#define DEBUG_YostIMU

#if USE_DMA
#include <DmaSpi.h>
#endif





#define YostIMU_BUFFERSIZE 10

#define SPISETTINGS SPISettings(100000, MSBFIRST, SPI_MODE0)

#define BUFFER_SIZE 64//max number of floats returned per command

//commands
#define GET_QUAT 0x00 // x,y,z,w?
#define GET_EULER 0x01 // roll,pitch,yaw ?
#define GET_ALL 0x25 // gyro(x3),accel(x3),compass(x3)

#define	RESET 0xE2
#define TARE 0x60

#define RREG 0x69
#define WREG 0xE9
#define STATUS 0x81



class YostIMU {
  public:

  YostIMU(int cs);
      
  uint8_t Read(uint8_t *buffer,uint8_t size);
  void WriteCommand(uint8_t _cmd);
  void WriteCommand(uint8_t _cmd, uint8_t *_data,uint8_t dataSize);

  void ReadFloats(uint8_t *buffer,float * values, int numFloats);
    
  void reset(void);
  void tare(void);

  bool DataReady(void);
  
  ~YostIMU();

  private: 
  
  int _cs;
  #if USE_DMA
	DmaSpi::Transfer trx;
	ActiveLowChipSelect csdma;
  #endif
  

};




#endif


