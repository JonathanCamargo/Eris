#include "Eris.h"
#include "imu.h"
#include "MPU9250.h"
#include "configuration.h"
#include "error.h"

namespace IMU{
  
static eris_thread_ref_t imuThread = NULL;

// Bitmask of which IMUs failed to initialize, reported by the FAIL command.
// -1 until InitIMU() has run at least once.
int failures = -1;
static const int IMU0_FAIL_BIT = (1 << 3);   // was B1000
static const int IMU1_FAIL_BIT = (1 << 2);   // was B0100

// Set by InitIMU() from each device's actual begin() result. The sampling
// thread skips any IMU whose flag is false, so an absent chip costs nothing.
bool imu0OK=false;
bool imu1OK=false;

// imu1 needs its AD0/SDO pin pulled HIGH to answer at 0x69; left floating or
// tied LOW it collides with imu0 and neither device enumerates reliably.
MPU9250 imu0(Wire,0x68);
MPU9250 imu1(Wire,0x69);

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

// Periodic sampling thread.
// This used to be a hardware-timer ISR (TimerTc3 on SAMD21), which pinned the
// flavor to one MCU family and did blocking I2C from interrupt context inside a
// critical section. A thread with an absolute-deadline wakeup is portable across
// every board eris_rtos.h supports and lets Wire run with interrupts enabled.
static void SampleIMU(void){
  float timestamp = ((float)(micros() - t0))/1000.0;
  //Sample every channel
  IMUSample_t thisSample;
  thisSample.timestamp=timestamp;

  if (imu0OK){
  imu0.readSensor();
  IMUGetDataHelper(&imu0,thisSample);
  buffer0.append(thisSample);   // append() takes the critical section itself
  }

  if (imu1OK){
  imu1.readSensor();
  IMUGetDataHelper(&imu1,thisSample);
  buffer1.append(thisSample);
  }
}

ERIS_THREAD_WA(waIMU_T, ERIS_STACK_MEDIUM);
ERIS_THREAD_FUNC(IMU_T) {
  (void)arg;
  eris_systime_t nextTime = eris_get_time();
  while(1){
    nextTime += ERIS_MS_TO_TICKS(IMU_PERIOD_MS);
    SampleIMU();
    eris_sleep_until(&nextTime);
  }
}

// Bring one MPU9250 up and apply the common configuration.
// Returns true on success. On failure the device is left alone: the old code
// ran the whole setSrd/setRange/setCal sequence even when begin() had failed,
// which just piled more failing I2C transactions onto an absent chip.
static bool InitOne(MPU9250 * imu, const char * name){
  eriscommon::print("Initializing ");
  eriscommon::println(name);

  int status = imu->begin();
  if (status < 0){
    sprintf(strbuffer,"%s initialization unsuccessful (status %d)",name,status);
    Error::RaiseError(Error::SENSOR,strbuffer);
    return false;
  }

  imu->setSrd(0); // 1000Hz
  imu->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imu->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // Identity accel calibration. These are per-physical-device and must be
  // measured for each chip -- do not copy values between units.
  imu->setAccelCalX(0,1);
  imu->setAccelCalY(0,1);
  imu->setAccelCalZ(0,1);
  return true;
}

void InitIMU(void){
  eriscommon::println("Initializing IMU");

  // Accumulate failure bits, so start from a clean slate. This used to be
  // `failures = -1` (all bits set), which made every subsequent |= a no-op and
  // left the FAIL command reporting -1 forever. -1 now means only "InitIMU has
  // never run" -- the value the global is born with.
  failures = 0;

  imu0OK = InitOne(&imu0,"IMU 0");
  if (!imu0OK) failures |= IMU0_FAIL_BIT;

  imu1OK = InitOne(&imu1,"IMU 1");
  if (!imu1OK) failures |= IMU1_FAIL_BIT;

  eriscommon::print("IMU 0: ");   eriscommon::println(imu0OK ? "OK" : "ABSENT");
  eriscommon::print("IMU 1: ");   eriscommon::println(imu1OK ? "OK" : "ABSENT");

  eriscommon::println("Starting IMU collection");
}

void start(void){
  Wire.begin();
  // Start ErisBuffers            
  buffer0.init();   
  buffer1.init();   
  InitIMU();          // sets `failures` from each device's begin() result
  imuThread = eris_thread_create(waIMU_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+1, IMU_T, NULL);
  Serial.println("IMU ready");
}
}
