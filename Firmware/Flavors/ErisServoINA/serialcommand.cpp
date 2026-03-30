#include "Eris.h"
#include "serialcommand.h"

#include "sinewave.h"
#include "ina219.h"
#include "servos.h"

#include "streaming.h"

char strbuffer[STRBUFFERSIZE];

namespace SerialCom{

  SerialCommand sCmd;

  eris_thread_ref_t readSerial = NULL;
  eris_thread_ref_t streamSerial = NULL;
  static bool stream_en=false;

  long startTime = 0;


  /********************** Threads *********************************/
  //To process Serial commands
  ERIS_THREAD_WA(waReadSerial_T, ERIS_STACK_LARGE);
	ERIS_THREAD_FUNC(ReadSerial_T) {
    while(1){
		  sCmd.readSerial();
		  eris_sleep_ms(10);
    }
	}

  //To stream data
  ERIS_THREAD_WA(waStreamSerial_T, ERIS_STACK_LARGE);
  ERIS_THREAD_FUNC(StreamSerial_T) {
    while(1){
      if (stream_en){
        stream();
      }
      eris_sleep_ms(10);
    }
  }

  /***************************************************************/

	void start(void){

    //COMMANDS:
    sCmd.addCommand("INFO",INFO);  // Display information about the firmware
    sCmd.addCommand("ON",LED_on);       // Turns LED on
    sCmd.addCommand("OFF",LED_off);        // Turns LED off

    sCmd.addCommand("SINE",TransmitSineWave); // Transmit the current sinewave buffer
    sCmd.addCommand("INA",TransmitINA219); // Transmit the current INA219 buffer

    sCmd.addCommand("X",ServoMove); // Move servo(s) immediately
    sCmd.addCommand("Y",ServoSmoothMove); // Move servo(s) smoothly

    sCmd.addCommand("S_TIME",SynchronizeTime); //Synchronize time

    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*

    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

    // create task at priority one
    readSerial=eris_thread_create(waReadSerial_T, ERIS_STACK_LARGE, ERIS_NORMAL_PRIORITY, ReadSerial_T, NULL);
    streamSerial=eris_thread_create(waStreamSerial_T, ERIS_STACK_LARGE, ERIS_NORMAL_PRIORITY+3, StreamSerial_T, NULL);

	}


void INFO() {
  packet.start(Packet::PacketType::TEXT);
  char temp[50];
  int num = sprintf(temp, "Firmware: %s", FIRMWARE_INFO);
  packet.append((uint8_t *)temp, num);
  packet.send();
}


void StartThreads(){
  Serial.println("Starting threads manually");
  KillThreads();
}

void StreamingStart(){
  stream_en=true;
}

void StreamingStop(){
  stream_en=false;
  Serial.println("Streaming off");
}

void SynchronizeTime(){
  // Reset the start time
  ERIS_CRITICAL_ENTER();
  t0=micros();
  ERIS_CRITICAL_EXIT();
}

void StreamingSetFeatures(){
  char *arg;
  ERIS_CRITICAL_ENTER();
  Streaming::ClearFunctions();
  // Select the streaming function based on names
  arg = sCmd.next();
  while(arg!= NULL){
      bool found=Streaming::AddFunction(arg);
      if (!found){
        Error::RaiseError(Error::COMMAND,(char *)"StreamingSetFeatures");
        Streaming::ClearFunctions();
        ERIS_CRITICAL_EXIT();
        return;
      }
      arg = sCmd.next();
  }
  ERIS_CRITICAL_EXIT();
  Serial.println("Features Ready");
}

void KillThreads(){
  Serial.println("Killing threads");
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
   ERIS_CRITICAL_ENTER();
   floatSample_t samples[MEMBUFFERSIZE];
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();
   ERIS_CRITICAL_EXIT();
   Serial.print("SineWave:");
   // Show number of missed points
   Serial.print("(missed:");
   Serial.print(missed);
   Serial.print(") ");
   // Show the data
   if (num>0){
     uint8_t i=0;
     Serial.print("(@");
     Serial.print(samples[0].timestamp,2);
     Serial.print("ms)");
     for (i=0;i<num-1;i++){
        Serial.print(samples[i].value,2);
        Serial.print(",");
     }
     Serial.print(samples[i].value,2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");
   }
   else {
     Serial.println();
   }
}

void TransmitINA219(){
   ERIS_CRITICAL_ENTER();
   inaSample_t samples[MEMBUFFERSIZE];
   int num=INA219::buffer.FetchData(samples,(char*)"INA219",MEMBUFFERSIZE);
   long missed=INA219::buffer.missed();
   ERIS_CRITICAL_EXIT();
   Serial.print("INA219:");
   Serial.print("(missed:");
   Serial.print(missed);
   Serial.print(") ");
   if (num>0){
     uint8_t i=0;
     Serial.print("(@");
     Serial.print(samples[0].timestamp,2);
     Serial.print("ms) ");
     for (i=0;i<num-1;i++){
        Serial.print(samples[i].current_mA,2);
        Serial.print("mA,");
     }
     Serial.print(samples[i].current_mA,2);
     Serial.print("mA (@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");
   }
   else {
     Serial.println();
   }
}

void ServoMove(){
  // Parse arguments: X <ch> <angle> or X <a0> <a1> ... <a15>
  char *arg;
  float args[NUM_SERVOS];
  int nargs = 0;

  arg = sCmd.next();
  while (arg != NULL && nargs < NUM_SERVOS){
    args[nargs] = atof(arg);
    nargs++;
    arg = sCmd.next();
  }

  if (nargs == 2){
    // Single servo: X <channel> <angle>
    uint8_t channel = (uint8_t)args[0];
    float angle = args[1];
    Servos::move(channel, angle);
    Serial.print("Servo ");
    Serial.print(channel);
    Serial.print(" -> ");
    Serial.println(angle);
  }
  else if (nargs == NUM_SERVOS){
    // All servos: X <a0> <a1> ... <a15>
    Servos::moveAll(args);
    Serial.println("All servos moved");
  }
  else {
    Serial.print("Error: expected 2 or ");
    Serial.print(NUM_SERVOS);
    Serial.println(" arguments");
  }
}

void ServoSmoothMove(){
  // Parse arguments: Y <ch> <angle> or Y <a0> <a1> ... <a15>
  char *arg;
  float args[NUM_SERVOS];
  int nargs = 0;

  arg = sCmd.next();
  while (arg != NULL && nargs < NUM_SERVOS){
    args[nargs] = atof(arg);
    nargs++;
    arg = sCmd.next();
  }

  if (nargs == 2){
    // Single servo: Y <channel> <angle>
    uint8_t channel = (uint8_t)args[0];
    float angle = args[1];
    Servos::smoothMove(channel, angle);
    Serial.print("Smooth servo ");
    Serial.print(channel);
    Serial.print(" -> ");
    Serial.println(angle);
  }
  else if (nargs == NUM_SERVOS){
    // All servos: Y <a0> <a1> ... <a15>
    Servos::smoothMoveAll(args);
    Serial.println("All servos smooth move");
  }
  else {
    Serial.print("Error: expected 2 or ");
    Serial.print(NUM_SERVOS);
    Serial.println(" arguments");
  }
}

void SayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    Serial.print("Hello ");
    Serial.println(arg);
  }
  else {
    Serial.println("Hello!");
  }
}


void GetID() {
  Serial.print("Firmware:");
  Serial.println(FIRMWARE_INFO);

}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Error::RaiseError(Error::COMMAND);
  Serial.println("What?");
}


}
