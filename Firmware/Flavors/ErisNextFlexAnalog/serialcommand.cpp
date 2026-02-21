#include "Eris.h"
#include "serialcommand.h"
#include "emg.h"
#include "fsr.h"
#include "sinewave.h"
#include "serialeti.h"
#include "streaming.h"

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;
  thread_t *streamSerial = NULL;
  static bool stream_en=false;

  long startTime = 0;

  //Static allocation to avoid moving a lot of memory in RTOS
   TISample_t tisamples[MEMBUFFERSIZE];      
   EMGSample_t emgsamples[MEMBUFFERSIZE];      


  
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
    sCmd.addCommand("EMG",TransmitEMG); // Transmit the current EMG buffer      
    sCmd.addCommand("ETI",TransmitETI); // Transmit the current ETI buffer
    sCmd.addCommand("FSR",TransmitFSR); // Transmit the current ETI buffer

    sCmd.addCommand("TIME0",SynchronizeTime); // Synchronize time                 
    sCmd.addCommand("S_F",StreamingSetFeatures); // Configure the streaming functions
    sCmd.addCommand("S_ON",StreamingStart); // Stream the buffers' data
    sCmd.addCommand("S_OFF",StreamingStop); // Stop streaming

    sCmd.addCommand("KILL",KillThreads); //Kill threads *Experimental*
    sCmd.addCommand("START",StartThreads);//Start threads *Experimental*
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

    // create task at priority one
    readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);
    streamSerial=chThdCreateStatic(waStreamSerial_T, sizeof(waStreamSerial_T),NORMALPRIO+3, StreamSerial_T, NULL);

	}


void INFO() {
  packet.start(Packet::PacketType::TEXT); 
  char temp[50]; 
  int num = sprintf(temp, "Firmware: %s", FIRMWARE_INFO); 
  //Serial.print(temp); 
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
  chSysLockFromISR();
  t0=micros();
  chSysUnlockFromISR();
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

void TransmitFSR(){
   chSysLockFromISR();
   FSRSample_t samples[MEMBUFFERSIZE];   
   int num=FSR::buffer.FetchData(samples,(char*)"FSR",MEMBUFFERSIZE);
   long missed=FSR::buffer.missed();   
   chSysUnlockFromISR();   
   Serial.print("FSR:");   
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
        Serial.print(samples[i].ch[0],2);
        Serial.print(",");
     }     
     Serial.print(samples[i].ch[0],2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
   }         
}
void TransmitSineWave(){
   chSysLockFromISR();
   floatSample_t samples[MEMBUFFERSIZE];   
   int num=SineWave::buffer.FetchData(samples,(char*)"SINEWAVE",MEMBUFFERSIZE);
   long missed=SineWave::buffer.missed();   
   chSysUnlockFromISR();   
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

void TransmitETI(){

   chSysLockFromISR();
   
   TISample_t * samples=&tisamples[0];      
   int num=SerialETI::buffer.FetchData(samples,(char*)"ETI",MEMBUFFERSIZE);
   //Serial.println(SerialETI::temperatureBuffer[0] == NULL);
   long missed=SerialETI::buffer.missed();      
   
   chSysUnlockFromISR();
 
   // Show number of missed points
   Serial.print("Temperature[ch0]:");
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
        Serial.print(samples[i].temperature[0],2);
        Serial.print(",");
     }     
     Serial.print(samples[i].temperature[0],2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
   }      

   
   // Show number of missed points
   Serial.print("Impedance[ch0]:");
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
        Serial.print(samples[i].impedance[0],2);
        Serial.print(",");
     }     
     Serial.print(samples[i].impedance[0],2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
   }      
   
}


void TransmitEMG(){
   chSysLockFromISR();  
   EMGSample_t * samples=emgsamples;      
   int num=EMG::buffer.FetchData(samples,(char*)"EMG",MEMBUFFERSIZE);
   long missed=EMG::buffer.missed();   
   chSysUnlockFromISR();

   // Show number of missed points
   Serial.print("EMG[ch0]:");
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
        Serial.print(samples[i].ch[0],2);
        Serial.print(",");
     }     
     Serial.print(samples[i].ch[0],2);
     Serial.print("(@");
     Serial.print(samples[i].timestamp,2);
     Serial.println("ms)");   
   } 
   else {
     Serial.println();
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
