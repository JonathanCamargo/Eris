
#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>
#include "Eris.h"
/*
All the functions related to streaming of data
*/


namespace Streaming{

bool AddFunction(char * arg);
void ClearFunctions();

//Functions should be declared as void and send the desired data via serial


//void EMG(); 
void IMU();
void IMU_byIdx(int index);
void IMU_0();
void IMU_1();
void IMU_2();
void IMU_3();

void FSR(); 

// Sinusoidal wave
void SineWave();

void Sync();

void Stream();

}


#endif
