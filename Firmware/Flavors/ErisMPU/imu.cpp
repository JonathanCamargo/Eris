#include "Eris.h"
#include "imu.h"
#include "MPU9250.h"
#include "configuration.h"
#include "error.h"

namespace IMU{
  
static eris_thread_ref_t imuThread = NULL;

int failures = -1;

bool imu0OK=true;
bool imu1OK=false;

MPU9250 imu0(Wire,0x68);
MPU9250 imu1(Wire,0x69);
MPU9250 * imuptr;

//Buffer for readings 
ErisBuffer<IMUSample_t> buffer0;
ErisBuffer<IMUSample_t> buffer1;

void IMUGetDataHelper(MPU9250 * imu, IMUSample_t & sample ){
  sample.ax=imu->getAccelX_mss();
  sample.ay=imu->getAccelY_mss();
  sample.az=imu->getAccelZ_mss();
  sample.wx=imu->getGyroX_rads();
  sample.wy=imu->getGyroY_rads();
  sample.wz=imu->getGyroZ_rads();
}

// Indices and flags
static void ISR_NewSample(){  
  ERIS_CRITICAL_ENTER();
  float timestamp = ((float)(micros() - t0))/1000.0;  
  //Sample every channel
  IMUSample_t thisSample;
  thisSample.timestamp=timestamp; 
    
  if (imu0OK){  
  imu0.readSensor();
  IMUGetDataHelper(&imu0,thisSample);
  buffer0.append(thisSample);
  }

  if (imu1OK){
  imu1.readSensor();
  IMUGetDataHelper(&imu1,thisSample);   
  buffer1.append(thisSample);
  }
  
  ERIS_CRITICAL_EXIT();  
}

void InitIMU(void){
  eriscommon::println("Initializing IMU");
  //timer0.end();
  int status=0;
  failures = -1;

  imu0OK=true;
  imu1OK=false;

  imuptr=&imu0;
  // Start the IMU configuration 
  eriscommon::println("Initializing IMU 0");
  status = imuptr->begin();
  if (status<0){
    failures |= B1000;
    Error::RaiseError(Error::SENSOR,"IMU 0 initialization unsuccessful");
    imu0OK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0,1);
  imuptr->setAccelCalY(0,1);
  imuptr->setAccelCalZ(0,1);

  //imuptr=&imu1;
  //eriscommon::println("Initializing IMU 1");
  //status = imuptr->begin();
  //if (status<0){
  //  failures |= B0100;
  //  RaiseError(Error::SENSOR,"IMU 1 initialization unsuccessful");    
  //  imuThighOK=false;
  //}
  //imuptr->setSrd(0); // 1000Hz
  //imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  //imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  //imuptr->setAccelCalX(0.15,1);
  //imuptr->setAccelCalY(0.15,1);
  //imuptr->setAccelCalZ(0.29,0.98);

 
  eriscommon::println("Starting IMU collection");  

  // Attach interrupt function
  //timer0.begin(ISR_NewSample, IMU_PERIOD_US);    
  TimerTc3.initialize(IMU_PERIOD_US); // 500000 us = 0.5 seconds
  // Attach interrupt function
  TimerTc3.attachInterrupt(ISR_NewSample);
}

void start(void){
  Wire.begin();
  failures=0;
  // Start ErisBuffers            
  buffer0.init();   
  buffer1.init();   
  //imuThread = eris_thread_create(waIMU_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+1, IMU_T, NULL);
  InitIMU();
  Serial.println("IMU ready");
}
}
