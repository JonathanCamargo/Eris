#include "Eris.h"
#include "imu.h"
#include "MPU9250.h"
#include "configuration.h"
#include "error.h"

namespace IMU{

// Bitmask of which IMUs failed to initialize, reported by the FAIL command.
// Bit (3 - ch) per channel, so IMU 0 -> bit 3, IMU 1 -> bit 2 (the historical
// B1000 / B0100 positions). -1 until InitIMU() has run at least once.
int failures = -1;

// The devices, and an index over them so the polling body can be written once.
// Pointers rather than an array of MPU9250 so nothing depends on the driver
// being copy-constructible.
//
// imu1 only answers at 0x69 with its AD0/SDO pin pulled HIGH; floating or tied
// LOW it collides with imu0 and neither enumerates reliably.
MPU9250 imu0(Wire,0x68);
MPU9250 imu1(Wire,0x69);
static MPU9250 * const dev[IMU_COUNT] = { &imu0, &imu1 };

// Set by InitIMU() from each device's actual begin() result. An IMU that is
// absent simply stays false and is skipped -- it is not an error.
static bool ok[IMU_COUNT] = { false, false };

}

// ---------------------------------------------------------------------------
// The sensor itself: 2 devices on one I2C bus, polled by one thread.
// Generates IMU::buffer[2], IMU::start(), and the 250 Hz polling thread.
// ---------------------------------------------------------------------------
ERIS_SENSOR_MULTI(IMU, IMUSample_t, IMU_FREQUENCY_HZ, IMU_COUNT) {
  if (!ok[ch]) return false;          // absent device: append nothing

  dev[ch]->readSensor();
  s.ax = dev[ch]->getAccelX_mss();
  s.ay = dev[ch]->getAccelY_mss();
  s.az = dev[ch]->getAccelZ_mss();
  s.wx = dev[ch]->getGyroX_rads();
  s.wy = dev[ch]->getGyroY_rads();
  s.wz = dev[ch]->getGyroZ_rads();
  return true;
}

namespace IMU{

// Bring one MPU9250 up and apply the common configuration.
// On failure the device is left alone: the old code ran the whole
// setSrd/setRange/setCal sequence even when begin() had failed, which just
// piled more failing I2C transactions onto an absent chip.
static bool InitOne(uint8_t ch){
  sprintf(strbuffer,"Initializing IMU %u",(unsigned)ch);
  eriscommon::println(strbuffer);

  int status = dev[ch]->begin();
  if (status < 0){
    sprintf(strbuffer,"IMU %u initialization unsuccessful (status %d)",(unsigned)ch,status);
    Error::RaiseError(Error::SENSOR,strbuffer);
    return false;
  }

  dev[ch]->setSrd(0); // 1000Hz
  dev[ch]->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  dev[ch]->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // Identity accel calibration. These are per-physical-device and must be
  // measured for each chip -- do not copy values between units.
  dev[ch]->setAccelCalX(0,1);
  dev[ch]->setAccelCalY(0,1);
  dev[ch]->setAccelCalZ(0,1);
  return true;
}

void InitIMU(void){
  eriscommon::println("Initializing IMU");

  // Accumulate failure bits, so start from a clean slate. This used to be
  // `failures = -1` (all bits set), which made every subsequent |= a no-op and
  // left the FAIL command reporting -1 forever. -1 now means only "InitIMU has
  // never run" -- the value the global is born with.
  failures = 0;

  for (uint8_t ch = 0; ch < IMU_COUNT; ch++){
    ok[ch] = InitOne(ch);
    if (!ok[ch]) failures |= (1 << (3 - ch));
    sprintf(strbuffer,"IMU %u: %s",(unsigned)ch, ok[ch] ? "OK" : "ABSENT");
    eriscommon::println(strbuffer);
  }
}

// Flavor entry point: bring the bus and the devices up, then hand over to the
// generated start(), which creates the buffers and the polling thread.
void begin(void){
  Wire.begin();
  InitIMU();
  start();
  Serial.println("IMU ready");
}

}
