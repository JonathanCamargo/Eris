/*
	Motor.h - A library for using the Arduino Due to
	communicate with the ELMO solo gold whistle MC.
	Created by Trent Rankin, trankin8@gatech.edu
	3/30/2018

	Changes by Jonathan	
	Renamed to ELMO and refactored as an arduino library (5/06/2018)
	Fixed mailbox operation	
	Repacked to use in teensy and due
	
*/

#ifndef ELMO_H
#define ELMO_H

#include <Arduino.h>
//Configurations
#define MAILBOXES 2
#define ELMO_CANDEV Can1
#define MAX_QUERIES 5

#if defined(__MK20DX256__) || defined(__MK20DX128__) || \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include "teensy3/elmo.h"
#elif defined(__SAM3X8E__) || defined(__SAM3X8H__)
#include "sam3x/elmo.h"
#else
#error unknown ARM processor
#endif


#endif
