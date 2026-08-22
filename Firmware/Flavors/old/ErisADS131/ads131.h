#ifndef ADS131_H
#define ADS131_H

#include <Arduino.h>
#include "Eris.h"
#include "customtypes.h"
#include <eriscommon.h>

namespace ADS131{

extern bool configStatus;
extern bool powerStatus;
extern ErisBuffer<EMGSample_t> buffer;

bool reset_config(void);
void startCollecting(void);
void stopCollecting(void);
void start(void);

//Register addresses
#define CONFIG1_ID 0x01
#define CONFIG2_ID 0x02
#define CONFIG3_ID 0x03
#define FAULT_ID 0x04
#define CH0_ID 0x05
#define CH1_ID 0x06
#define CH2_ID 0x07
#define CH3_ID 0x08
#define CH4_ID 0x09
#define CH5_ID 0x0A
#define CH6_ID 0x0B
#define CH7_ID 0x0C
#define FAULT_STATP 0x12
#define FAULT_STATN 0x13
#define GPIO 0x14

//Register settings
#define CONFIG1_1K B10010110 //1ksps
#define CONFIG1_2K B10010101 //2ksps
#define CONFIG2 B11100010
#define CONFIG3_UNI B11100000 //unipolar supply, vref = 4V
#define CONFIG3_BI B11000000 //bipolar supply, vref = 2.4V
#define FAULT B0
#define CH0_1 B00010000 //gain = 1
#define CH0_2 B00100000 //gain = 2

//SPI commands
#define WAKEUP 0x02
#define STANDBY 0x04
#define RESET 0x06
#define START 0x08
#define STOP 0x0A
#define OFFSETCAL 0x1A
#define RDATAC 0x10
#define SDATAC 0x11
#define RDATA 0x12
#define RREG 0x02
#define WREG 0x04

}

#endif
