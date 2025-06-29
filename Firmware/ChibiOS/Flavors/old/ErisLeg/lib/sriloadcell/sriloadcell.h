
#ifndef SRILOADCELL_H
#define SRILOADCELL_H

#include <Arduino.h>

//Configurations
#define MAILBOXES 2


#define CANDEV Can0

#define MOMENT_SCALING_FACTOR 10.0


typedef struct LoadcellData {
	float forceX;
	float forceY;
	float forceZ;
	float momentX;
	float momentY;
	float momentZ;
} LoadcellData_t;

typedef struct LoadcellDataRaw {
	uint8_t loadcellvalues[8];//CANFRAME
} LoadcellDataRaw_t;


int bit2sign(bool bit);
LoadcellData_t frame2data(uint8_t *frameDataBytes);


#if defined(__MK20DX256__) || defined(__MK20DX128__) || \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include "teensy3/sriloadcell.h"
#elif defined(__SAM3X8E__) || defined(__SAM3X8H__)
#include "sam3x/sriloadcell.h"
#else
#error unknown ARM processor
#endif

#endif
