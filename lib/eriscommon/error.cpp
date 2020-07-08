#include <ChRt.h>
#include "eriscommon.h"
#include "error.h"



namespace Error {


thread_t *processErrors = NULL;

bool enableErrorHandles;

void RaiseError(ErrorCode_t errorCode) {
    if (enableErrorHandles){
    //Resume the error handle task passing the error code
    chThdResumeI(&processErrors,(msg_t)errorCode);
    }
}

void RaiseError(ErrorCode_t errorCode,char* text) {
    if (enableErrorHandles){
    //Resume the error handle task passing the error code
    Serial.print(text);
    chThdResumeI(&processErrors,(msg_t)errorCode);
    }
}

static THD_WORKING_AREA(waProcessErrors_T, 32);
static THD_FUNCTION(ProcessErrors_T, arg) {  
  while(1){
    msg_t msg = chThdSuspendS(&processErrors);
    //Maybe save or log errors?
    switch ((ErrorCode_t)msg){
      case LOWBAT:
        //Serial.println("<<ERROR>>Low battery<<ERROR>>");
	eriscommon::printText("<<ERROR>>Low battery<<ERROR>>");
        break;
      case BUFFER:
        eriscommon::printText("<<ERROR>>Buffer<<ERROR>>");
        break;
      case CANBUS:
        eriscommon::printText("<<ERROR>>CANBUS<<ERROR>>");
        break;
      case COMMAND:
        eriscommon::printText("<<ERROR>>Unsupported command<<ERROR>>");
        break;  
      default:
        eriscommon::printText("<<ERROR>>");
        break;
    }
  }
}

void ToggleErrorHandles(){
  if (enableErrorHandles==true){
    enableErrorHandles=false;
    eriscommon::printText(F("Errors disabled"));
  }
  else{
    enableErrorHandles=true;
    eriscommon::printText(F("Errors enabled"));
  }
}

void EnableErrorHandles(){
  enableErrorHandles=true;
  eriscommon::printText(F("Enable error handle"));
}
void DisableErrorHandles(){
  enableErrorHandles=false;
  eriscommon::printText(F("Disable error handle"));
}


void start(void) {
 enableErrorHandles=true;
 processErrors=chThdCreateStatic(waProcessErrors_T, sizeof(waProcessErrors_T),NORMALPRIO, ProcessErrors_T, NULL);
}







}
