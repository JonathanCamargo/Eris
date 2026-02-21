#include "Eris.h"
#include "joints.h"
#include "configuration.h"

#include <elmo.h>

namespace Joints {

//Scheduled tasks
thread_t *writeCurrent = NULL;// UpdateCurrent_T send new TC command
thread_t *readCurrent = NULL;// TODO: ReadCurrent_T (100Hz) read current in motor

//Buffer for readings
ErisBuffer<JointStateSample_t> kneebuffer;
ErisBuffer<JointStateSample_t> anklebuffer;

long counterTest = 0;

#if DEBUG_TIME
time_measurement_t time;
#endif

bool firstTime = true;

//Elmo driver connected to CANID
static Elmo motor0(ELMO_CANID0);
static Elmo motor1(ELMO_CANID1);

// Setpoints
double SetpointKnee, OutputKnee;
double SetpointAnkle, OutputAnkle;

JointStateSample_t lastKneeJointState;
JointStateSample_t lastAnkleJointState;

//Impedance Parameters
ImpedanceParameters_t ipK;
ImpedanceParameters_t ipA;

void UpdateState(JointStateSample_t newKneeState, JointStateSample_t newAnkleState ) {

  float timestamp = ((float)(micros() - t0)) / 1.0e3;
  ////eriscommon::println(timestamp);  
  newKneeState.timestamp = timestamp;
  newAnkleState.timestamp = timestamp;
  kneebuffer.append(newKneeState);
  anklebuffer.append(newAnkleState);
  // Copy data into the buffer variable:
  lastKneeJointState = newKneeState;
  lastAnkleJointState = newAnkleState;

  chSysUnlockFromISR();

}


double ImpedanceLawKnee(JointStateSample_t jointState) {
  double Tau;
  Tau = -ipK.k * (jointState.theta - ipK.theta_eq) - ipK.b * (jointState.theta_dot);
  return Tau;
}

double ImpedanceLawAnkle(JointStateSample_t jointState) {
  double Tau;
  Tau = -ipA.k * (jointState.theta - ipA.theta_eq) - ipA.b * (jointState.theta_dot);
  return Tau;
}

void SetIP(float k_Knee, float b_Knee, float theta_eq_Knee, float k_Ankle, float b_Ankle, float theta_eq_Ankle) {
  ipK.k = k_Knee;
  ipK.b = b_Knee;
  ipK.theta_eq = theta_eq_Knee;
  ipA.k = k_Ankle;
  ipA.b = b_Ankle;
  ipA.theta_eq = theta_eq_Ankle;
}

void SetIPA(float k, float b, float theta_eq) {
  ipA.k = k;
  ipA.b = b;
  ipA.theta_eq = theta_eq;
}

void SetIPK(float k, float b, float theta_eq) {
  ipK.k = k;
  ipK.b = b;
  ipK.theta_eq = theta_eq;
}

void SetIP(ImpedanceParameters_t paramsK, ImpedanceParameters_t paramsA) {
  ipK.k = paramsK.k;
  ipK.b = paramsK.b;
  ipK.theta_eq = paramsK.theta_eq;
  ipA.k = paramsA.k;
  ipA.b = paramsA.b;
  ipA.theta_eq = paramsA.theta_eq;
}


void GetIP(float &k_Knee, float &b_Knee, float &theta_eq_Knee, float &k_Ankle, float &b_Ankle, float &theta_eq_Ankle) {
  k_Knee = ipK.k;
  b_Knee = ipK.b;
  theta_eq_Knee = ipK.theta_eq;
  k_Ankle = ipA.k;
  b_Ankle = ipA.b;
  theta_eq_Ankle = ipA.theta_eq;
}

void GetIP(ImpedanceParameters_t paramsK, ImpedanceParameters_t paramsA) {
  paramsK.k = ipK.k;
  paramsK.b = ipK.b;
  paramsK.theta_eq = ipK.theta_eq;
  paramsA.k = ipA.k;
  paramsA.b = ipA.b;
  paramsA.theta_eq = ipA.theta_eq;
}

static THD_WORKING_AREA(waWriteCurrent_T, 512);
static THD_FUNCTION(WriteCurrent_T, arg) {
  // Take the last joint state reading and update the impedance law to produce the new torque command for the motor
  systime_t time = chVTGetSystemTime();
  while (!chThdShouldTerminateX()) {
#if DEBUG_TIME
    chTMStopMeasurementX(&time);
    chTMStartMeasurementX(&time);
#endif

    ReadIncrementalData();
    //Send output to motor using CAN library
    SetpointKnee = ImpedanceLawKnee(lastKneeJointState);
    SetpointAnkle = ImpedanceLawAnkle(lastAnkleJointState);
    OutputKnee = (SetpointKnee / KNEE_GEAR_RATIO) / ELMO_KTCONSTANT;
    OutputAnkle = (SetpointAnkle / ANKLE_GEAR_RATIO) / ELMO_KTCONSTANT;
    //Saturate current
    if (OutputKnee > ELMO_MAXCURRENT) {
      OutputKnee = ELMO_MAXCURRENT;
    }
    else if (OutputKnee < ELMO_MINCURRENT) {
      OutputKnee = ELMO_MINCURRENT;
    }
    if (OutputAnkle > ELMO_MAXCURRENT) {
      OutputAnkle = ELMO_MAXCURRENT;
    }
    else if (OutputAnkle < ELMO_MINCURRENT) {
      OutputAnkle = ELMO_MINCURRENT;
    }

    //Output=0; //Making sure we don't turn on the motors yet
    //Output=0.25*sin(2*M_PI*0.1*millis()/1000);
    motor0.set_current(OutputKnee);
    //eriscommon::println(OutputKnee);
    motor1.set_current(OutputAnkle);
    //eriscommon::println(OutputAnkle);

    time = time + US2ST(ELMO_RATE_US);
    chThdSleepUntil(time);
  }
  eriscommon::println("Joint thread terminated");
}

void kill() {
  if (writeCurrent != NULL) {
    chThdTerminate(writeCurrent);
  }
}

void ReadIncrementalData() {
  // Read position and velocity
  bool readingsOk = true;
  int pos0, pos1, vel0, vel1;

  //Blocking
  //readingsOk=readingsOk && motor0.get_position(&pos0);
  //readingsOk=readingsOk && motor1.get_position(&pos1);
  //readingsOk=readingsOk && motor0.get_velocity(&vel0);
  //readingsOk=readingsOk && motor1.get_velocity(&vel1);

  //Nonblocking
  readingsOk = readingsOk && motor0.query_position();
  readingsOk = readingsOk && motor1.query_position();
  chThdSleepMicroseconds(2000);
  readingsOk = readingsOk && motor0.get_reply(&pos0);
  readingsOk = readingsOk && motor1.get_reply(&pos1);
  readingsOk = readingsOk && motor0.query_velocity();
  readingsOk = readingsOk && motor1.query_velocity();
  chThdSleepMicroseconds(2000);
  readingsOk = readingsOk && motor0.get_reply(&vel0);
  readingsOk = readingsOk && motor1.get_reply(&vel1);


  if (readingsOk) {
    // Write JointState with this data if absolute encoders are disabled
    JointStateSample_t newKneeState;
    JointStateSample_t newAnkleState;

    //        counterTest=counterTest+1;
    //        if (counterTest==10000){
    //          counterTest=0;
    //        }
    //        newKneeState.theta=counterTest;

    newKneeState.theta=pos0*360.0/(ELMO_MAX_COUNTS_INCREMENTAL*KNEE_GEAR_RATIO);
    newAnkleState.theta = pos1 * 360.0 / (ELMO_MAX_COUNTS_INCREMENTAL * ANKLE_GEAR_RATIO);
    newKneeState.theta_dot = vel0 * 360.0 / (ELMO_MAX_COUNTS_INCREMENTAL * KNEE_GEAR_RATIO);
    newAnkleState.theta_dot = vel1 * 360.0 / (ELMO_MAX_COUNTS_INCREMENTAL * ANKLE_GEAR_RATIO);
    newKneeState.torque = 0;
    newAnkleState.torque = 0;
    UpdateState(newKneeState, newAnkleState);
  }
  else {
    Error::RaiseError(CANBUS, (char *)"JOINTS");
  }

}


void start(void) {

#if DEBUG_TIME
  chTMObjectInit (&time);
  chTMStartMeasurementX(&time);
#endif

  //
  if (firstTime) {
    kneebuffer.init();
    anklebuffer.init();
    firstTime = false;
  }
  //
  ipK.k = 0; ipK.b = 0; ipK.theta_eq = 0;
  ipA.k = 0; ipA.b = 0; ipA.theta_eq = 0;

  // Start CAN
  Can1.begin(1000000);
  bool motorsOk = true;

  if (motor0.connect()) {
    motor0.motor_on();
    eriscommon::println("Turning on motor 0");
  }
  else {
    motorsOk = false;
    Error::RaiseError(CANBUS, (char *)"Joint");
  }

  if (motor1.connect()) {
    motor1.motor_on();
    eriscommon::println("Turning on motor 1");
  }
  else {
    motorsOk = false;
    Error::RaiseError(CANBUS, (char*)"Joint");
  }

  if (motorsOk) {
    writeCurrent = chThdCreateStatic(waWriteCurrent_T, sizeof(waWriteCurrent_T), NORMALPRIO + 1, WriteCurrent_T, NULL);
  }
}
}
