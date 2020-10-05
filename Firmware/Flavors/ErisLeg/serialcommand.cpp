#include "Eris.h"
#include "serialcommand.h"
#include "configuration.h"
#include "fsr.h"
#include "joints.h"
#include "loadcell.h"
#include "sinewave.h"
#include "sync.h"
#include "streaming.h"

//static Packet packet;

//Static allocation to avoid moving a lot of memory in RTOS
static FSRSample_t fsrsamples[MEMBUFFERSIZE];
static uint8_tSample_t syncsamples[MEMBUFFERSIZE];
static JointStateSample_t kneevalues[MEMBUFFERSIZE];
static JointStateSample_t anklevalues[MEMBUFFERSIZE];
static LoadcellSample_t loadcellsamples[MEMBUFFERSIZE];

char strbuffer[STRBUFFERSIZE];

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;
  static bool stream_en=false;

  long startTime = 0;


  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 1024);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(100);
    }
	}

  //To process //Serial commands
  static THD_WORKING_AREA(waStreamSerial_T, 1024);
  static THD_FUNCTION(StreamSerial_T, arg) {
    while(1){
      if (stream_en){
        stream();
      }
      chThdSleepMilliseconds(10);
    }
  }

  /***************************************************************/

	void start(void){

    //COMMANDS:
    sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    

    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current FSR buffer
    sCmd.addCommand("SYNC",TransmitSync); // Transmit the current SYNC buffer
    sCmd.addCommand("JOINT",TransmitJointState); // Transmit the current joint state
    sCmd.addCommand("LC",TransmitLoadcellState); // Transmit the current loadcell buffer
    sCmd.addCommand("IP",SetIP);  // Modify impedance parameters
    sCmd.addCommand("IPA",SetIPA);  // Modify impedance parameters of Ankle
    sCmd.addCommand("IPK",SetIPK);  // Modify impedance parameters of Knee
    sCmd.addCommand("IP?",GetIP);  // Modify impedance parameters

    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    eriscommon::println("Serial Commands are ready");

    // create task at priority one
    readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  sprintf(strbuffer, "Firmware: %s", FIRMWARE_INFO);
  eriscommon::printText(strbuffer);
}

void StartThreads(){
	eriscommon::println("Starting threads manually");
	Joints::start();
	Loadcell::start();
	}

void KillThreads(){
  eriscommon::println("Killing threads");
  Joints::kill();
  Loadcell::kill();
}

void StreamingStart(){
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;
  eriscommon::println("Streaming off");
}

void StreamingSetFeatures(){
  char *arg;
  chSysLockFromISR();
  Streaming::ClearFunctions();
  // Select the streaming function based on names
  arg = sCmd.next();
  while(arg!= NULL){
      bool found=Streaming::AddFunction(arg);
      if (!found){
        Error::RaiseError(COMMAND,(char *)"StreamingSetFeatures");
        Streaming::ClearFunctions();
        chSysUnlockFromISR();
        return;
      }
      arg = sCmd.next();
  }
  chSysUnlockFromISR();
  eriscommon::println("Features Ready");
}

void stream(){
  Streaming::Stream();
}

void LED_on() {
  eriscommon::println("LED on");
  digitalWrite(PIN_LED, HIGH);
}

void LED_off() {
  eriscommon::println("LED off");
  digitalWrite(PIN_LED, LOW);
}

void TransmitSineWave(){
   chSysLockFromISR();
   floatSample_t samples[MEMBUFFERSIZE];
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();
   chSysUnlockFromISR();
   eriscommon::print("SineWave:");
   // Show number of missed points
   eriscommon::print("(missed:");
   eriscommon::print(missed);
   eriscommon::print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     eriscommon::print("(@");
     eriscommon::print(samples[0].timestamp,2);
     eriscommon::print("ms)");
     for (i=0;i<num-1;i++){
        eriscommon::print(samples[i].value,2);
        eriscommon::print(",");
     }
     eriscommon::print(samples[i].value,2);
     eriscommon::print("(@");
     eriscommon::print(samples[i].timestamp,2);
     eriscommon::println("ms)");
   }
   else {
     eriscommon::println("num < 0");
   }
}

void TransmitFSR(){
  chSysLockFromISR();
  FSRSample_t *samples=&fsrsamples[0];
  int num=FSR::buffer.FetchData(samples,(char*)"FSR",MEMBUFFERSIZE);
  long missed=FSR::buffer.missed();
  chSysUnlockFromISR();
  eriscommon::print("FSR[ch0]:");
  // Show number of missed points
  eriscommon::print("(missed:");
  eriscommon::print(missed);
  eriscommon::print(") ");
  // Show the data
   if (num>0){
     uint8_t i=0;
     eriscommon::print("(@");
     eriscommon::print(samples[0].timestamp,2);
     eriscommon::print("ms)");
     for (i=0;i<num-1;i++){
        eriscommon::print(samples[i].ch[0],2);
        eriscommon::print(",");
     }
     eriscommon::print(samples[i].ch[0],2);
     eriscommon::print("(@");
     eriscommon::print(samples[i].timestamp,2);
     eriscommon::println("ms)");
   }
   else {
     eriscommon::println("num < 0");
   }
}


void TransmitSync(){
  chSysLockFromISR();
  uint8_tSample_t *samples=&syncsamples[0];
  int num=Sync::buffer.FetchData(samples,(char*)"SYNC",MEMBUFFERSIZE);
  long missed=Sync::buffer.missed();
  chSysUnlockFromISR();
  eriscommon::print("Sync:");
  // Show number of missed points
  eriscommon::print("(missed:");
  eriscommon::print(missed);
  eriscommon::print(") ");
  // Show the data
   if (num>0){
     uint8_t i=0;
     eriscommon::print("(@");
     eriscommon::print(samples[0].timestamp,2);
     eriscommon::print("ms)");
     for (i=0;i<num-1;i++){
        eriscommon::print(samples[i].value,2);
        eriscommon::print(",");
     }
     eriscommon::print(samples[i].value,2);
     eriscommon::print("(@");
     eriscommon::print(samples[i].timestamp,2);
     eriscommon::println("ms)");
   }
   else {
     eriscommon::println("num < 0");
   }
}

void TransmitJointState(){
   #if DEBUG_TIME
      chTMStartMeasurementX(&time);
   #endif
   chSysLockFromISR();
   int numKnee=Joints::kneebuffer.FetchData(kneevalues,(char*)"JOINTS_RAW",MEMBUFFERSIZE);   
   int numAnkle=Joints::anklebuffer.FetchData(anklevalues,(char*)"JOINTS_RAW",MEMBUFFERSIZE);
   chSysUnlockFromISR();
   #if DEBUG_TIME
      chTMStopMeasurementX(&time);
   #endif

   // Show the data
   uint8_t i=0;
   int numsamples=(numKnee < numAnkle) ? numKnee : numAnkle;
   
   eriscommon::println("(Knee)theta,theta_dot,torque,(Ankle)theta,theta_dot,torque");
      
   for (i=0;i<numsamples;i++){
       eriscommon::print(kneevalues[i].theta,2);
       eriscommon::print(",");
       eriscommon::print(kneevalues[i].theta_dot,2);
       eriscommon::print(",");
       eriscommon::print(kneevalues[i].torque,2);
       eriscommon::print(",");
       eriscommon::print(anklevalues[i].theta,2);
       eriscommon::print(",");
       eriscommon::print(anklevalues[i].theta_dot,2);
       eriscommon::print(",");
       eriscommon::println(anklevalues[i].torque,2);
   }   
   
   #if DEBUG_TIME
      chSysLockFromISR();
      time_measurement_t tmptime;
      tmptime=Joints::time;
      chSysUnlockFromISR();
      eriscommon::print("Write Current: ");
      eriscommon::print("Last:");eriscommon::print(tmptime.last/DEBUG_SYSCLK);eriscommon::print("(us) ");
      eriscommon::print("Best:");eriscommon::print(tmptime.best/DEBUG_SYSCLK);eriscommon::print("(us) ");
      eriscommon::print("Worst:");eriscommon::print(tmptime.worst/DEBUG_SYSCLK);eriscommon::println("(us) ");
      eriscommon::print("n=");eriscommon::print(tmptime.n);eriscommon::println("samples");
      chSysLockFromISR();
      tmptime=time;
      chSysUnlockFromISR();
      eriscommon::print("READING: ");
      eriscommon::print("Last:");eriscommon::print(tmptime.last/DEBUG_SYSCLK);eriscommon::print("(us) ");
      eriscommon::print("Best:");eriscommon::print(tmptime.best/DEBUG_SYSCLK);eriscommon::print("(us) ");
      eriscommon::print("Worst:");eriscommon::print(tmptime.worst/DEBUG_SYSCLK);eriscommon::print("(us) ");
      eriscommon::print("n=");eriscommon::print(tmptime.n);eriscommon::println("samples");
   #endif
}

void TransmitLoadcellState(){
   chSysLockFromISR();   
   int num=Loadcell::buffer.FetchData(loadcellsamples,(char*)"LOADCELL_RAW",MEMBUFFERSIZE);
   chSysUnlockFromISR();
   // Show the data
   uint8_t i=0;
   for (i=0;i<num;i++){
       LoadcellSample_t * data=&loadcellsamples[i];
       eriscommon::print("forceX:");
       eriscommon::print(data->forceX,2);
       eriscommon::print(", forceY:");
       eriscommon::print(data->forceY,2);
       eriscommon::print(", forceZ:");
       eriscommon::print(data->forceZ,2);
       eriscommon::print(", momentX:");
       eriscommon::print(data->momentX,2);
       eriscommon::print(", momentY:");
       eriscommon::print(data->momentY,2);
       eriscommon::print(", momentZ:");
       eriscommon::println(data->momentZ,2);
   }
}

void SayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    eriscommon::print("Hello ");
    eriscommon::println(arg);
  }
  else {
    eriscommon::println("Hello!");
  }
}




void SetIP() {
  float kKnee,bKnee,theta_eqKnee;
  float kAnkle,bAnkle,theta_eqAnkle;
  char *arg;

  //Knee
  arg = sCmd.next();
  if (arg != NULL) {
    kKnee = atof(arg);    // Converts a char string to an integer
  }
  else {
    eriscommon::println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    bKnee = atof(arg);
  }
  else {
    eriscommon::println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eqKnee = atof(arg);
  }
  else {
    eriscommon::println("No third argument");
    return;
  }

    arg = sCmd.next();
  if (arg != NULL) {
    kAnkle = atof(arg);    // Converts a char string to an integer
  }
  else {
    eriscommon::println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    bAnkle = atof(arg);
  }
  else {
    eriscommon::println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eqAnkle = atof(arg);
  }
  else {
    eriscommon::println("No third argument");
    return;
  }

  Joints::SetIP(kKnee,bKnee,theta_eqKnee,kAnkle,bAnkle,theta_eqAnkle);
  eriscommon::print("Updated impedance parameters:");
  eriscommon::print("kKnee=");eriscommon::print(kKnee,2);
  eriscommon::print(", bKnee=");eriscommon::print(bKnee,2);
  eriscommon::print(", theta_eqKnee=");eriscommon::print(theta_eqKnee,2);
  eriscommon::print(" kAnkle=");eriscommon::print(kAnkle,2);
  eriscommon::print(", bAnkle=");eriscommon::print(bAnkle,2);
  eriscommon::print(", theta_eqAnkle=");eriscommon::println(theta_eqAnkle,2);
}

void SetIPA() {
  float k,b,theta_eq;
  char *arg;

  //Knee
  arg = sCmd.next();
  if (arg != NULL) {
    k = atof(arg);    // Converts a char string to an integer
  }
  else {
    eriscommon::println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    b = atof(arg);
  }
  else {
    eriscommon::println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eq = atof(arg);
  }
  else {
    eriscommon::println("No third argument");
    return;
  }

  Joints::SetIPA(k,b,theta_eq);
  eriscommon::print("Updated impedance parameters:");
  eriscommon::print(" kAnkle=");eriscommon::print(k,2);
  eriscommon::print(", bAnkle=");eriscommon::print(b,2);
  eriscommon::print(", theta_eqAnkle=");eriscommon::println(theta_eq,2);
}


void SetIPK() {
  float k,b,theta_eq;
  char *arg;

  //Knee
  arg = sCmd.next();
  if (arg != NULL) {
    k = atof(arg);    // Converts a char string to an integer
  }
  else {
    eriscommon::println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    b = atof(arg);
  }
  else {
    eriscommon::println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eq = atof(arg);
  }
  else {
    eriscommon::println("No third argument");
    return;
  }
  Joints::SetIPK(k,b,theta_eq);
  eriscommon::print("Updated impedance parameters:");
  eriscommon::print(" kKnee=");eriscommon::print(k,2);
  eriscommon::print(", bKnee=");eriscommon::print(b,2);
  eriscommon::print(", theta_eqKnee=");eriscommon::println(theta_eq,2);
}

void GetIP() {
  float kKnee=0,bKnee=0,theta_eqKnee=0;
  float kAnkle=0,bAnkle=0,theta_eqAnkle=0;
  Joints::GetIP(kKnee,bKnee,theta_eqKnee,kAnkle,bAnkle,theta_eqAnkle);
  eriscommon::print("kKnee=");eriscommon::print(kKnee,2);
  eriscommon::print(", bKnee=");eriscommon::print(bKnee,2);
  eriscommon::print(", theta_eqKnee=");eriscommon::print(theta_eqKnee,2);
  eriscommon::print(" kAnkle=");eriscommon::print(kAnkle,2);
  eriscommon::print(", bAnkle=");eriscommon::print(bAnkle,2);
  eriscommon::print(", theta_eqAnkle=");eriscommon::println(theta_eqAnkle,2);
}

void GetID() {
  eriscommon::print("Firmware:");
  eriscommon::println(FIRMWARE_INFO);

}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Error::RaiseError(COMMAND);
  eriscommon::println("What?");
}


}
