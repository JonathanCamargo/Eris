#include "Eris.h"

#include "serialeti.h"
#include "emg.h"
#include "serialcommand.h"



namespace SerialETI{
  thread_t *readSerial = NULL;  

  ErisBuffer<TISample_t> buffer;
    
  static char charbuffer0[64]; //Buffer for string chars  
  
  static uint8_t bufPos[ETI_NUMCHANNELS];
  static HardwareSerial* serialArr[ETI_NUMCHANNELS] = {&ETI_SERIAL0}; //{&ETI_SERIAL0, &ETI_SERIAL1};   
  typedef char * charPtr;
  static charPtr charbufferPtr[ETI_NUMCHANNELS]={charbuffer0};//{charbuffer0,charbuffer1};
  
  static HardwareSerial* currSerial; 
  static charPtr currBuffer;

  static uint8_t returnFlag = 0; 
  static float lastImpedance[ETI_NUMCHANNELS]={NAN};
  static float lastTemperature[ETI_NUMCHANNELS] = {NAN};

  static TISample_t thisSample;
    
  /********************** Threads *********************************/
  	   
	static THD_WORKING_AREA(waReadSerial_T, 128);
	static THD_FUNCTION(ReadSerial_T, arg) {  	   	  	
	  while(1){		    
		//Check serial buffer and see if there is anything there 					
      for (int i = 0; i < ETI_NUMCHANNELS; i++) {
        uint8_t newData=ReadSerial(i);
        if (newData){                  
          thisSample.temperature[i]=lastTemperature[i];       
          thisSample.impedance[i]=lastImpedance[i];
          if (i==0){
            thisSample.timestamp=((float)(micros() - t0))/1.0e3;  
          }
          if (i==ETI_NUMCHANNELS-1){
            buffer.append(thisSample);
            // Write data to sdcard (implemented in sdcard.cpp)            
          }
        }                          
        }  					  
		chThdSleepMilliseconds(100);
	  }
	}

  /***************************************************************/
   void clearBuffer(uint8_t idx){
	     charbufferPtr[idx][0] = '\0';
		   bufPos[idx] = 0;
   }
   
   bool ReadSerial(uint8_t idx){
     returnFlag = 0; 
     static const char term='\r';
     float temperature, impedance;  
     currSerial = serialArr[idx]; 
     currBuffer = charbufferPtr[idx];     
       if (currSerial->available()){ 
          while (currSerial->available() > 0) {
            char inChar = currSerial->read();
            uint8_t pos=bufPos[idx];
            pos++;
            bufPos[idx]=pos;
            currBuffer[pos] = inChar;  // Put character into buffer          
            currBuffer[pos+1]='\0';    // Move term char        
            //Serial.print(inChar);
            if (inChar==term){
              //float test=Serial.parseFloat(&buffer);
              impedance=atof(&currBuffer[11]);
              //Serial.println(impedance);
              temperature=atof(&currBuffer[bufPos[idx]-6]);
              lastTemperature[idx]=temperature;
              lastImpedance[idx]=impedance;
              clearBuffer(idx);  
              //Serial.print("\t1: ");
              //Serial.print(temperature);
              returnFlag = true;
            }
          }          
       }     
     return returnFlag;
   }
  
  
	void start(void){ 
   ETI_SERIAL0.begin(115200);
        
   buffer.init();
   
   while (!ETI_SERIAL0) {
      ; // wait for //Serial port to connect.
   }

   
   Serial.println("ETI ready");
   // create task at priority one
	readSerial=chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T),NORMALPRIO, ReadSerial_T, NULL);    

	}

}
