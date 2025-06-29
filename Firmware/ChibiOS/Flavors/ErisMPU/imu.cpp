#include "Eris.h"
#include "MPU9250.h"
#include "configuration.h"
#include "error.h"

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

void InitIMU(void){
  eriscommon::println("Initializing IMU");
  timer0.end();
  int status=0;
  failures = -1;

  imuTrunkOK=true;
  imuThighOK=true;
  imuShankOK=true;
  imuFootOK=true;

  imuptr=&imuTrunk;
  // Start the IMU configuration 
  eriscommon::println("Initializing IMU Trunk");
  status = imuptr->begin();
  if (status<0){
    failures |= B1000;
    Error::RaiseError(Error::SENSOR,"Trunk IMU initialization unsuccessful");
    imuTrunkOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0,1);
  imuptr->setAccelCalY(0,1);
  imuptr->setAccelCalZ(0,1);

  imuptr=&imuThigh;
  eriscommon::println("Initializing IMU Thigh");
  status = imuptr->begin();
  if (status<0){
    failures |= B0100;
    RaiseError(Error::SENSOR,"Thigh IMU initialization unsuccessful");    
    imuThighOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0.15,1);
  imuptr->setAccelCalY(0.15,1);
  imuptr->setAccelCalZ(0.29,0.98);

  imuptr=&imuShank;
  eriscommon::println("Initializing IMU Shank");
  status = imuptr->begin();
  if (status<0){
    failures |= B0010;
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
    RaiseError(Error::SENSOR,"Shank IMU initialization unsuccessful");
    
    imuShankOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0.04,1);
  imuptr->setAccelCalY(0.04,1);
  imuptr->setAccelCalZ(0.13,0.99);

  imuptr=&imuFoot;
  eriscommon::println("Initializing IMU Foot");
  status = imuptr->begin();
  if (status<0){
    failures |= B0001;
    RaiseError(Error::SENSOR,"Foot IMU initialization unsuccessful");    
    imuFootOK=false;
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(-.01,1);
  imuptr->setAccelCalY(-.04,1);
  imuptr->setAccelCalZ(-.05,.98);

  eriscommon::println("Starting IMU collection");
  timer0.begin(ISR_NewSample, IMU_PERIOD_US);    
}

void start(void){
  pinMode(PIN_IMU_TRUNK,OUTPUT);
  pinMode(PIN_IMU_THIGH,OUTPUT);
  pinMode(PIN_IMU_SHANK,OUTPUT);
  pinMode(PIN_IMU_FOOT,OUTPUT);
  digitalWrite(PIN_IMU_TRUNK,HIGH);
  digitalWrite(PIN_IMU_THIGH,HIGH);
  digitalWrite(PIN_IMU_SHANK,HIGH);
  digitalWrite(PIN_IMU_FOOT,HIGH);
  SPI.begin();
  failures=0;
  // Start ErisBuffers            
  bufferTrunk.init();   
  bufferThigh.init();   
  bufferShank.init();   
  bufferFoot.init();            

  InitIMU();
   
  //Start samples semaphore for feature extraction
  //chBSemObjectInit(&xsamplesSemaphore,true);        
  // Initialize interrupt to take the samples      
  // Timer interrupt to take the IMU samples        
  //Set up ADC   
  }
}
