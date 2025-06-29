#include "Eris.h"
#include "serialcommand.h"

#include "sinewave.h"
#include "joints.h"
#include "loadcell.h"

#include "sdcard.h"
#include "streaming.h"


namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;

  static bool stream_en=false;

  
  /********************** Threads *********************************/
  //To process //Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 4096);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
		  sCmd.readSerial();
		  chThdSleepMilliseconds(100);
    }
	}

  //To process //Serial commands
  static THD_WORKING_AREA(waStreamSerial_T, 4096);
  static THD_FUNCTION(StreamSerial_T, arg) {    
    while(1){
      if (stream_en){
        stream();  
      }                        
      chThdSleepMilliseconds(40);
    }
  }
 
  /***************************************************************/

	void start(void){ 

    //COMMANDS:
    sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off
    
    sCmd.addCommand("ID?",GetID); // Get the ID of this device and additional info

    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("JOINT",TransmitJointState); // Transmit the current joint state
    sCmd.addCommand("LC",TransmitLoadcellState); // Transmit the current joint state
    sCmd.addCommand("IP",SetIP);  // Modify impedance parameters
    sCmd.addCommand("IPA",SetIPA);  // Modify impedance parameters of Ankle
    sCmd.addCommand("IPK",SetIPK);  // Modify impedance parameters of Knee
    sCmd.addCommand("IP?",GetIP);  // Modify impedance parameters

    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.addCommand("SD_REC",StartRecording); //Save to sd card
    sCmd.addCommand("SD_NREC",StopRecording); //Save to sd card
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

  	// create task at priority one
		readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO, StreamSerial_T, NULL);

	}

 void INFO() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);
}


void StartRecording(){
  Serial.println("Start record on SDCard");
  SDCard::StartRecording();
}

void StopRecording(){
  SDCard::StopRecording();
  Serial.println("Stop record on SDCard");
}


void StartThreads(){
  Serial.println("Starting threads manually");
  KillThreads();
  Joints::start(); 
  Loadcell::start(); 
}

void KillThreads(){
  Serial.println("Killing threads");  
  Joints::kill();
  Loadcell::kill();
}

void StreamingStart(){  
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;
  Serial.println("Streaming off");
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
  Serial.println("Features Ready");    
}

void stream(){
  Streaming::Stream();
}


void LED_on() {
  Serial.println("LED on");
  digitalWrite(PIN_LED, HIGH);
}

void LED_off() {
  Serial.println("LED off");
  digitalWrite(PIN_LED, LOW);
}

void TransmitSineWave(){
   chSysLockFromISR();
   float values[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(values,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   chSysUnlockFromISR();
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num<=0){
    return;
   }
   uint8_t i=0;
   for (i=0;i<num-1;i++){
      Serial.print(values[i],2);
      Serial.print(",");
   }
   Serial.println(values[i],2);
}

void TransmitJointStateRaw(){
   #if DEBUG_TIME
      chTMStartMeasurementX(&time);
   #endif
   chSysLockFromISR();
   JointState_t values[BUFFERSIZE];
   int num=Joints::buffer.FetchData(values,(char*)"JOINTS_RAW",MEMBUFFERSIZE);
   long missed=Joints::buffer.missed();
   chSysUnlockFromISR();
   // Show number of missed points
   Serial.println(num);
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");

   //Convert to reduced resolution
   JointStateRaw_t valueraw[BUFFERSIZE];
   uint8_t i=0;
   for (i=0;i<num;i++){
      Joints::JointState_t2JointStateRaw_t(values[i],valueraw[i]);
   }

   for (i=0;i<num;i++){
       Serial.print("thetaKnee:");
       Serial.print(valueraw[i].thetaKnee);
       Serial.print(", thetadotKnee:");
       Serial.print(valueraw[i].thetadotKnee);
       Serial.print(", torqueKnee:");
       Serial.print(valueraw[i].torqueKnee);
       Serial.print(", thetaAnkle:");
       Serial.print(valueraw[i].thetaAnkle);
       Serial.print(", thetadotAnkle:");
       Serial.print(valueraw[i].thetadotAnkle);
       Serial.print(", torqueAnkle:");
       Serial.println(valueraw[i].torqueAnkle);
   }

}

void TransmitJointState(){
   #if DEBUG_TIME
      chTMStartMeasurementX(&time);
   #endif
   chSysLockFromISR();
   JointState_t values[MEMBUFFERSIZE];
   int num=Joints::buffer.FetchData(values,(char*)"JOINTS_RAW",MEMBUFFERSIZE);
   long missed=Joints::buffer.missed();
   chSysUnlockFromISR();
   #if DEBUG_TIME
      chTMStopMeasurementX(&time);
   #endif
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   uint8_t i=0;
   for (i=0;i<num;i++){
       Serial.print("thetaKnee:");
       Serial.print(values[i].thetaKnee,2);
       Serial.print(", thetadotKnee:");
       Serial.print(values[i].thetadotKnee,2);
       Serial.print(", torqueKnee:");
       Serial.print(values[i].torqueKnee,2);
       Serial.print(", thetaAnkle:");
       Serial.print(values[i].thetaAnkle,2);
       Serial.print(", thetadotAnkle:");
       Serial.print(values[i].thetadotAnkle,2);
       Serial.print(", torqueAnkle:");
       Serial.println(values[i].torqueAnkle,2);
   }
   #if DEBUG_TIME
      chSysLockFromISR();
      time_measurement_t tmptime;
      tmptime=Joints::time;
      chSysUnlockFromISR();
      Serial.print("Write Current: ");
      Serial.print("Last:");Serial.print(tmptime.last/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("Best:");Serial.print(tmptime.best/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("Worst:");Serial.print(tmptime.worst/DEBUG_SYSCLK);Serial.println("(us) ");
      Serial.print("n=");Serial.print(tmptime.n);Serial.println("samples");
      chSysLockFromISR();
      tmptime=time;
      chSysUnlockFromISR();
      Serial.print("READING: ");
      Serial.print("Last:");Serial.print(tmptime.last/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("Best:");Serial.print(tmptime.best/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("Worst:");Serial.print(tmptime.worst/DEBUG_SYSCLK);Serial.print("(us) ");
      Serial.print("n=");Serial.print(tmptime.n);Serial.println("samples");
   #endif
}

void TransmitLoadcellState(){
   chSysLockFromISR();
   LoadcellDataRaw_t values[MEMBUFFERSIZE];
   int num=Loadcell::buffer.FetchData(values,(char*)"LOADCELL_RAW",MEMBUFFERSIZE);
   long missed=Loadcell::buffer.missed();
   chSysUnlockFromISR();
   // Show number of missed points
   Serial.print("(");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   uint8_t i=0;
   for (i=0;i<num;i++){
       LoadcellData_t data=frame2data((uint8_t *)&values[i]);
       Serial.print("forceX:");
       Serial.print(data.forceX,2);
       Serial.print(", forceY:");
       Serial.print(data.forceY,2);
       Serial.print(", forceZ:");
       Serial.print(data.forceZ,2);
       Serial.print(", momentX:");
       Serial.print(data.momentX,2);
       Serial.print(", momentY:");
       Serial.print(data.momentY,2);
       Serial.print(", momentZ:");
       Serial.println(data.momentZ,2);
   }
}


void SayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the //SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    Serial.print("Hello ");
    Serial.println(arg);
  }
  else {
    Serial.println("Hello!");
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
    Serial.println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    bKnee = atof(arg);
  }
  else {
    Serial.println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eqKnee = atof(arg);
  }
  else {
    Serial.println("No third argument");
    return;
  }

    arg = sCmd.next();
  if (arg != NULL) {
    kAnkle = atof(arg);    // Converts a char string to an integer
  }
  else {
    Serial.println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    bAnkle = atof(arg);
  }
  else {
    Serial.println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eqAnkle = atof(arg);
  }
  else {
    Serial.println("No third argument");
    return;
  }

  Joints::SetIP(kKnee,bKnee,theta_eqKnee,kAnkle,bAnkle,theta_eqAnkle);
  Serial.print("Updated impedance parameters:");
  Serial.print("kKnee=");Serial.print(kKnee,2);
  Serial.print(", bKnee=");Serial.print(bKnee,2);
  Serial.print(", theta_eqKnee=");Serial.print(theta_eqKnee,2);
  Serial.print(" kAnkle=");Serial.print(kAnkle,2);
  Serial.print(", bAnkle=");Serial.print(bAnkle,2);
  Serial.print(", theta_eqAnkle=");Serial.println(theta_eqAnkle,2);
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
    Serial.println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    b = atof(arg);
  }
  else {
    Serial.println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eq = atof(arg);
  }
  else {
    Serial.println("No third argument");
    return;
  }

  Joints::SetIPA(k,b,theta_eq);
  Serial.print("Updated impedance parameters:");
  Serial.print(" kAnkle=");Serial.print(k,2);
  Serial.print(", bAnkle=");Serial.print(b,2);
  Serial.print(", theta_eqAnkle=");Serial.println(theta_eq,2);
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
    Serial.println("No arguments");
    return;
  }
  arg = sCmd.next();
  if (arg != NULL) {
    b = atof(arg);
  }
  else {
    Serial.println("No second argument");
    return;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    theta_eq = atof(arg);
  }
  else {
    Serial.println("No third argument");
    return;
  }
	Joints::SetIPK(k,b,theta_eq);
Serial.print("Updated impedance parameters:");
Serial.print(" kKnee=");Serial.print(k,2);
Serial.print(", bKnee=");Serial.print(b,2);
Serial.print(", theta_eqKnee=");Serial.println(theta_eq,2);
}

void GetIP() {
  float kKnee=0,bKnee=0,theta_eqKnee=0;
  float kAnkle=0,bAnkle=0,theta_eqAnkle=0;
  Joints::GetIP(kKnee,bKnee,theta_eqKnee,kAnkle,bAnkle,theta_eqAnkle);
  Serial.print("kKnee=");Serial.print(kKnee,2);
  Serial.print(", bKnee=");Serial.print(bKnee,2);
  Serial.print(", theta_eqKnee=");Serial.print(theta_eqKnee,2);
  Serial.print(" kAnkle=");Serial.print(kAnkle,2);
  Serial.print(", bAnkle=");Serial.print(bAnkle,2);
  Serial.print(", theta_eqAnkle=");Serial.println(theta_eqAnkle,2);
}


void GetID() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);

}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Error::RaiseError(COMMAND);
  Serial.println("What?");
}


}
