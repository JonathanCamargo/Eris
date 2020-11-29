#include "Eris.h"
#include "btnpwm.h"

namespace ButtonPWM{
	
thread_t *generatePWM = NULL;
thread_t *transitions = NULL;

static const int PERIOD_LOW_MS=(int) 1000/FREQ_LOW_HZ;
static const int PERIOD_HIGH_MS=(int) 1000/FREQ_HIGH_HZ;

typedef enum btnstates_t{  
  BTN_OFF, // Button is released 
  BTN_PRESSED, // Button is pressed  
  BTN_HOLD // Button is hold
};

typedef enum btnactions_t{  
  ACTION_PRESS=0b0001, // Button is press
  ACTION_RELEASE=0b0010 // Button is release  
};

static btnstates_t btnstate=BTN_OFF;                 

typedef enum pwmstates_t{
  PWM_ZERO,
  PWM_LOW,
  PWM_HIGH  
};
static pwmstates_t pwmstate=PWM_ZERO; // Current state of the PWM
static pwmstates_t pwmvel=PWM_LOW; // Current velocity of the PWM

static THD_WORKING_AREA(waGeneratePWM_T, 16);
static THD_FUNCTION(GeneratePWM_T, arg) {        
  while(1){      
    // Update the pulse    
    switch (pwmstate){
      case PWM_ZERO:
          digitalWrite(PIN_LED,HIGH);        
          chThdSleepMilliseconds(PERIOD_LOW_MS);       
          break;
      case PWM_LOW:        
          digitalWrite(PIN_LED,!digitalRead(PIN_LED));                                
          chThdSleepMilliseconds(PERIOD_LOW_MS);                       
          break;
      case PWM_HIGH:
          digitalWrite(PIN_LED,!digitalRead(PIN_LED));                                
          chThdSleepMilliseconds(PERIOD_HIGH_MS);       
          break;                
    }        
}
}

void PrintState(){
  switch (btnstate){
    case (BTN_OFF):
      Serial.println(F("BTN_OFF"));
      break;
    case (BTN_PRESSED):
      Serial.println(F("BTN_PRESSED"));
      break;
    case (BTN_HOLD):
      Serial.println(F("BTN_HOLD"));
      break;     
  }
}

event_source_t action_event_source;
static THD_WORKING_AREA(waTransitions_T, 16);
static THD_FUNCTION(Transitions_T, arg) {  
  long t0=millis();
  long elapsed=0;    
  // Check the button acctions and transition the state of the system
  event_listener_t action_event_listener;  
  chEvtRegisterMaskWithFlags(&action_event_source,
                    &action_event_listener,
                    EVENT_MASK(0),
                    ACTION_PRESS | ACTION_RELEASE); 
  while(1){ 
    PrintState();
    eventmask_t evt = chEvtWaitOneTimeout(EVENT_MASK(0),TIME_IMMEDIATE);
    if (evt & EVENT_MASK(0)){                    
      eventflags_t flags=chEvtGetAndClearFlags(&action_event_listener);
      switch (btnstate){
        case (BTN_OFF):
          if (flags | ACTION_PRESS){
            btnstate=BTN_PRESSED;
            t0=millis();
            elapsed=0;
            // Chill
          }
          break;
        case (BTN_PRESSED):
          if (flags | ACTION_RELEASE){
            btnstate=BTN_OFF;            
            // Change velocity
            Serial.println(F("Toggle ON/OFF"));
            pwmstate==PWM_ZERO ? pwmstate=pwmvel : pwmstate=PWM_ZERO;
          }
          break;
        case (BTN_HOLD):
          if (flags | ACTION_RELEASE){
            btnstate=BTN_OFF;
            // Chill
          }
          break;        
      }   
    }
    else{
      // If no event registered the button could be still pressed or on hold
      switch (btnstate){
        case (BTN_PRESSED):        
        case (BTN_HOLD):
          elapsed=millis()-t0;
          if (elapsed>TIME_MIN_HOLD_MS && !digitalRead(PIN_BTN) ){
            btnstate=BTN_HOLD;
            pwmvel==PWM_LOW ? pwmvel=PWM_HIGH : pwmvel=PWM_LOW;
            pwmstate=pwmvel;
            t0=millis();
            Serial.println(F("Change speed"));
          }
          else if (elapsed>TIME_MIN_HOLD_MS){
            btnstate=BTN_OFF;
          }
        break;                 
      }     
    }
    
    
           
    chThdSleepMilliseconds(20);
  }
}

static long prevTime=0;
static void ISR_PIN_BTN(){  
    chSysLockFromISR();    
    long time=millis();    
    if ((time-prevTime)<20){ //Prevent bouncing
      chSysUnlockFromISR();
      return;
    }
    prevTime=time;
    switch (btnstate){
      case BTN_OFF:        
        // Button pressed :)        
        eriscommon::println(F("press")); 
        chEvtBroadcastFlagsI(&action_event_source,ACTION_PRESS);       
        break;
      case BTN_PRESSED:          
        eriscommon::println(F("release"));      
        chEvtBroadcastFlagsI(&action_event_source,ACTION_RELEASE);       
        break;       
      case BTN_HOLD:        
        eriscommon::println(F("release"));
        chEvtBroadcastFlagsI(&action_event_source,ACTION_RELEASE);               
        break;        
    }             
    chSysUnlockFromISR();
}


	void start(void){         
    prevTime=millis();
    chEvtObjectInit(&action_event_source);   
    pinMode(PIN_BTN,INPUT_PULLUP);
    digitalWrite(PIN_LED,LOW);
    /*eriscommon::print(F("Low ms:"));
    eriscommon::println(PERIOD_LOW_MS);
    eriscommon::print(F("High ms:"));
    eriscommon::println(PERIOD_HIGH_MS);
    */    
    attachInterrupt(digitalPinToInterrupt(PIN_BTN), ISR_PIN_BTN,CHANGE);    
    // create tasks at priority lowest priority
    generatePWM=chThdCreateStatic(waGeneratePWM_T, sizeof(waGeneratePWM_T),NORMALPRIO+1, GeneratePWM_T, NULL);
    transitions=chThdCreateStatic(waTransitions_T, sizeof(waTransitions_T),NORMALPRIO+1, Transitions_T, NULL);
	}
	
	
}
