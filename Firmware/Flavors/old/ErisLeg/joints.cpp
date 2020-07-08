#include "Eris.h"
#include "joints.h"
#include "sdcard.h"

#include "buffers.h"

#include <elmo.h>
// Closed or openloop torque
#define OPENLOOP // comment for closed loop torque using PID
#ifndef OPENLOOP
#include <PID_v1.h>
#endif

#if defined(__arm__) && defined(CORE_TEENSY)
#endif
#if defined(__SAM3X8E__) || defined(__SAM3X8H__)
  #include <due_can.h>
#endif


namespace Joints{

  //Scheduled tasks
  thread_t *writeCurrent = NULL;// UpdateCurrent_T send new TC command
  thread_t *readCurrent = NULL;// TODO: ReadCurrent_T (100Hz) read current in motor
  
  //Buffer for readings
  ErisBuffer<JointState_t> buffer;

  long counterTest=0;
  
  #if DEBUG_TIME
    time_measurement_t time;
  #endif

  bool firstTime=true;

  //Elmo driver connected to CANID
  static Elmo motor0(ELMO_CANID0);
  static Elmo motor1(ELMO_CANID1);
  
  //PID setpoints
  double SetpointKnee, InputKnee, OutputKnee;
  double SetpointAnkle, InputAnkle, OutputAnkle;

  JointState_t lastJointState;
  
  #ifndef OPENLOOP
  #error THIS IS NOT FINISHED
  double Kp=2, Ki=5, Kd=1;
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  #endif
  
  //Impedance Parameters
  ImpedanceParameters_t ip;

void UpdateState(JointState_t newState){
  //Receive a new joint state object and upload to the buffer queue
  chSysLockFromISR();
  //Check if we have free space in the mailbox
  if (chMBGetFreeCountI(&buffer)>0){
    uint8_t index=0;
    for (uint8_t i=0;i<BUFFERSIZE;i++){
      if (!bufferData[i].inuse){
           index=i;
           break;
      }
    }
    //copy newState to the data in buffer
    memcpy(&bufferData[index].data,&newState,sizeof(JointState_t));
    bufferData[index].inuse=true;
    msg_t msg=(uint32_t)&bufferData[index];
    (void)chMBPostI(&buffer,msg);
  }
  else{
    //Remove the first message from the mailbox and increment missed counter
    msg_t msg;
    chMBFetchI(&buffer,&msg);
    missed=missed+1;
    (*(buffer_t*)msg).inuse=true;
    memcpy(&((*(buffer_t*)msg).data),&newState,sizeof(JointState_t));
    chMBPostI(&buffer,msg);
  }
  #if SDCARD
  SDCard::addJoint(newState);
  #endif

  // Copy data into the buffer variable:
  lastJointState=newState;
  
  chSysUnlockFromISR();
  
}


double ImpedanceLawKnee(JointState_t jointState){
  double Tau;
  Tau=-ip.kKnee*(jointState.thetaKnee-ip.theta_eqKnee)-ip.bKnee*(jointState.thetadotKnee);
  return Tau;
}

double ImpedanceLawAnkle(JointState_t jointState){
  double Tau;
  Tau=-ip.kAnkle*(jointState.thetaAnkle-ip.theta_eqAnkle)-ip.bAnkle*(jointState.thetadotAnkle);
  return Tau;
}

void SetIP(float k_Knee,float b_Knee, float theta_eq_Knee,float k_Ankle,float b_Ankle, float theta_eq_Ankle){
  ip.kKnee=k_Knee;
  ip.bKnee=b_Knee;
  ip.theta_eqKnee=theta_eq_Knee;
  ip.kAnkle=k_Ankle;
  ip.bAnkle=b_Ankle;
  ip.theta_eqAnkle=theta_eq_Ankle;
}

void SetIPA(float k, float b, float theta_eq){
  ip.kAnkle=k;
  ip.bAnkle=b;
  ip.theta_eqAnkle=theta_eq;
}

void SetIPK(float k, float b, float theta_eq){
  ip.kKnee=k;
  ip.bKnee=b;
  ip.theta_eqKnee=theta_eq;
}

void SetIP(ImpedanceParameters_t params){
  ip.kKnee=params.kKnee;
  ip.bKnee=params.bKnee;
  ip.theta_eqKnee=params.theta_eqKnee;
  ip.kAnkle=params.kAnkle;
  ip.bAnkle=params.bAnkle;
  ip.theta_eqAnkle=params.theta_eqAnkle;
}


void GetIP(float &k_Knee,float &b_Knee, float &theta_eq_Knee,float &k_Ankle,float &b_Ankle, float &theta_eq_Ankle){
  k_Knee=ip.kKnee;
  b_Knee=ip.bKnee;
  theta_eq_Knee=ip.theta_eqKnee;
  k_Ankle=ip.kAnkle;
  b_Ankle=ip.bAnkle;
  theta_eq_Ankle=ip.theta_eqAnkle;
}

void GetIP(ImpedanceParameters_t params){
  params.kKnee=ip.kKnee;
  params.bKnee=ip.bKnee;
  params.theta_eqKnee=ip.theta_eqKnee;
  params.kAnkle=ip.kAnkle;
  params.bAnkle=ip.bAnkle;
  params.theta_eqAnkle=ip.theta_eqAnkle;
}

static THD_WORKING_AREA(waWriteCurrent_T, 512);
 static THD_FUNCTION(WriteCurrent_T, arg) {
  // Take the last joint state reading and update the impedance law to produce the new torque command for the motor
    systime_t time = chVTGetSystemTime();   
    while (!chThdShouldTerminateX()){
    #if DEBUG_TIME
      chTMStopMeasurementX(&time);
      chTMStartMeasurementX(&time);
    #endif

    ReadIncrementalData();
    //Read the last jointState
    /*USING PEEK IS DUMB (delay)
    chSysLockFromISR();
    msg_t msg;
    msg=chMBPeekI(&buffer);
    JointState_t lastJointState;
    JointState_t * src=&(*(buffer_t*)msg).data;
    memcpy(&lastJointState,src,sizeof(JointState_t));
    chSysUnlockFromISR();
    */
    #ifndef OPENLOOP
      Input=lastJointState.torqueKnee;
      Setpoint=ImpedanceLaw(lastJointState);
      //Compute pid output from current torque measurement and send to ELMO
      myPID.Compute();
     //Output contains pid output
      //Send output to motor using CAN library
      //motor0.set_current(Output);
    #else
      //Send output to motor using CAN library
      SetpointKnee=ImpedanceLawKnee(lastJointState);
      SetpointAnkle=ImpedanceLawAnkle(lastJointState);
      OutputKnee=(SetpointKnee/KNEE_GEAR_RATIO)/ELMO_KTCONSTANT;
      OutputAnkle=(SetpointAnkle/ANKLE_GEAR_RATIO)/ELMO_KTCONSTANT;
      //Saturate current
      if (OutputKnee>ELMO_MAXCURRENT){
        OutputKnee=ELMO_MAXCURRENT;
      }
      else if (OutputKnee<ELMO_MINCURRENT){
        OutputKnee=ELMO_MINCURRENT;
      }
      if (OutputAnkle>ELMO_MAXCURRENT){
        OutputAnkle=ELMO_MAXCURRENT;
      }
      else if (OutputAnkle<ELMO_MINCURRENT){
        OutputAnkle=ELMO_MINCURRENT;
      }
     
      //Output=0; //Making sure we don't turn on the motors yet
      //Output=0.25*sin(2*M_PI*0.1*millis()/1000);
      motor0.set_current(OutputKnee);
      //Serial.println(OutputKnee);
      motor1.set_current(OutputAnkle);
      //Serial.println(OutputAnkle);

    #endif
    time=time+US2ST(ELMO_RATE_US);
    chThdSleepUntil(time);
   }
   Serial.println("Joint thread terminated");
 }

void kill(){
  if (writeCurrent!=NULL){
   chThdTerminate(writeCurrent);
  }
}

void ReadIncrementalData(){
    // Read position and velocity
      bool readingsOk=true;
      int pos0,pos1,vel0,vel1;

      //Blocking
      //readingsOk=readingsOk && motor0.get_position(&pos0);
      //readingsOk=readingsOk && motor1.get_position(&pos1);
      //readingsOk=readingsOk && motor0.get_velocity(&vel0);
      //readingsOk=readingsOk && motor1.get_velocity(&vel1);

      //Nonblocking
        readingsOk=readingsOk && motor0.query_position();
        readingsOk=readingsOk && motor1.query_position();
        chThdSleepMicroseconds(2000);
        readingsOk=readingsOk && motor0.get_reply(&pos0);
        readingsOk=readingsOk && motor1.get_reply(&pos1);
        readingsOk=readingsOk && motor0.query_velocity();
        readingsOk=readingsOk && motor1.query_velocity();
        chThdSleepMicroseconds(2000);
        readingsOk=readingsOk && motor0.get_reply(&vel0);
        readingsOk=readingsOk && motor1.get_reply(&vel1);
      
        
      if (readingsOk){
        // Write JointState with this data if absolute encoders are disabled
        JointState_t newState;
        //newState.thetaKnee=pos0*360.0/(ELMO_MAX_COUNTS_INCREMENTAL*KNEE_GEAR_RATIO);
        counterTest=counterTest+1;
        if (counterTest==10000){
          counterTest=0;
        }
        newState.thetaKnee=counterTest;
        newState.thetaAnkle=pos1*360.0/(ELMO_MAX_COUNTS_INCREMENTAL*ANKLE_GEAR_RATIO);
        newState.thetadotKnee=vel0*360.0/(ELMO_MAX_COUNTS_INCREMENTAL*KNEE_GEAR_RATIO);
        newState.thetadotAnkle=vel1*360.0/(ELMO_MAX_COUNTS_INCREMENTAL*ANKLE_GEAR_RATIO);
        newState.torqueAnkle=0;
        newState.torqueKnee=0;
        UpdateState(newState);
      }
      else{
        Error::RaiseError(CANBUS,(char *)"JOINTS");
      }

}


void JointState_t2JointStateRaw_t(JointState_t &orig,JointStateRaw_t &dest){
  //Normalize to a range  and convert
  dest.thetaKnee=(uint16_t)((65536)*(orig.thetaKnee-JOINT_MINPOS)/(JOINT_MAXPOS-JOINT_MINPOS));
  dest.thetaAnkle=(uint16_t)((65536)*(orig.thetaAnkle-JOINT_MINPOS)/(JOINT_MAXPOS-JOINT_MINPOS));
  dest.thetadotKnee=(uint16_t)((65536)*(orig.thetadotKnee-JOINT_MINVEL)/(JOINT_MAXVEL-JOINT_MINVEL));
  dest.thetadotAnkle=(uint16_t)((65536)*(orig.thetadotAnkle-JOINT_MINVEL)/(JOINT_MAXVEL-JOINT_MINVEL));
  dest.torqueKnee=(uint16_t)((65536)*(orig.torqueKnee-JOINT_MINTOR)/(JOINT_MAXTOR-JOINT_MINTOR));
  dest.torqueAnkle=(uint16_t)((65536)*(orig.torqueAnkle-JOINT_MINTOR)/(JOINT_MAXTOR-JOINT_MINTOR));
}


void start(void){
    missed=0;

    #if DEBUG_TIME
    chTMObjectInit (&time);
    chTMStartMeasurementX(&time);
    #endif

    //
    if (firstTime){
      chMBObjectInit (&buffer, buffer_P, BUFFERSIZE);
      firstTime=false;
    }
    //
    ip.kKnee=0;ip.bKnee=0;ip.theta_eqKnee=0;
    ip.kAnkle=0;ip.bAnkle=0;ip.theta_eqAnkle=0;

    #ifndef OPENLOOP
    //configure the pid
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(ELMO_MINCURRENT, ELMO_MAXCURRENT);
    #endif

    // Start CAN
    Can0.begin(1000000);
    bool motorsOk=true;

    if (motor0.connect()){
      motor0.motor_on();
      Serial.println("Turning on motor 0");
    }
    else{
        motorsOk=false;
        Error::RaiseError(CANBUS,(char *)"Joint");
    }

    if (motor1.connect()){
      motor1.motor_on();
      Serial.println("Turning on motor 1");
    }
    else{
        motorsOk=false;
        Error::RaiseError(CANBUS,(char*)"Joint");
    }

    if (motorsOk){
       writeCurrent=chThdCreateStatic(waWriteCurrent_T, sizeof(waWriteCurrent_T),NORMALPRIO+1, WriteCurrent_T, NULL);
	  }
}
}
