#include "Eris.h"
#include "MPU9250.h"
#include "configuration.h"

namespace IMU{
  
thread_t *readAnalog = NULL;

IntervalTimer timer0; // Timer for triggering IMU collection

int failures = -1;

bool imuTrunkOK=true;
bool imuThighOK=true;
bool imuShankOK=true;
bool imuFootOK=true;

MPU9250 imuTrunk(SPI,PIN_IMU_TRUNK);
MPU9250 imuThigh(SPI,PIN_IMU_THIGH);
MPU9250 imuShank(SPI,PIN_IMU_SHANK);
MPU9250 imuFoot(SPI,PIN_IMU_FOOT);
MPU9250 * imuptr;

//Buffer for readings 
ErisBuffer<IMUSample_t> bufferTrunk;
ErisBuffer<IMUSample_t> bufferThigh;
ErisBuffer<IMUSample_t> bufferShank;
ErisBuffer<IMUSample_t> bufferFoot;

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
  chSysLockFromISR();
  float timestamp = ((float)(micros() - t0))/1000.0;  
  //Sample every channel
  IMUSample_t thisSample;
  thisSample.timestamp=timestamp; 
  
  //Trunk
  if (imuTrunkOK){  
  imuTrunk.readSensor();
  IMUGetDataHelper(&imuTrunk,thisSample);   
  bufferTrunk.append(thisSample);
  }

  if (imuThighOK){
  imuThigh.readSensor();
  IMUGetDataHelper(&imuThigh,thisSample);   
  bufferThigh.append(thisSample);
  }

  if (imuShankOK){
  imuShank.readSensor();
  IMUGetDataHelper(&imuShank,thisSample);   
  bufferShank.append(thisSample);
  }

  if (imuFootOK){
  imuFoot.readSensor();
  IMUGetDataHelper(&imuFoot,thisSample);   
  bufferFoot.append(thisSample);
  }
  
  chSysUnlockFromISR();  
}


void start(void){
  failures=0;
  // Start ErisBuffers            
  bufferTrunk.init();   
  bufferThigh.init();   
  bufferShank.init();   
  bufferFoot.init();            

  imuptr=&imuTrunk;
	// Start the IMU configuration 
  int status = imuptr->begin();
	if (status<0){
    failures |= B1000;
		Serial.println("Trunk IMU initialization unsuccessful");
    imuTrunkOK=false;
	}
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0,1);
  imuptr->setAccelCalY(0,1);
  imuptr->setAccelCalZ(0,1);

  imuptr=&imuThigh;
  status = imuptr->begin();
  if (status<0){
    failures |= B0100;
    Serial.println("Thigh IMU initialization unsuccessful");
    imuThighOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0.15,1);
  imuptr->setAccelCalY(0.15,1);
  imuptr->setAccelCalZ(0.29,0.98);

  imuptr=&imuShank;
  status = imuptr->begin();
  if (status<0){
    failures |= B0010;
    Serial.println("Shank IMU initialization unsuccessful");
    imuShankOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0.04,1);
  imuptr->setAccelCalY(0.04,1);
  imuptr->setAccelCalZ(0.13,0.99);

  imuptr=&imuFoot;
  status = imuptr->begin();
  if (status<0){
    failures |= B0001;
    Serial.println("Foot IMU initialization unsuccessful");
    imuFootOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(-.01,1);
  imuptr->setAccelCalY(-.04,1);
  imuptr->setAccelCalZ(-.05,.98);
   
  //Start samples semaphore for feature extraction
  //chBSemObjectInit(&xsamplesSemaphore,true);        
  // Initialize interrupt to take the samples      
  // Timer interrupt to take the IMU samples        
  //Set up ADC
  timer0.begin(ISR_NewSample, IMU_PERIOD_US);      
  }
}
