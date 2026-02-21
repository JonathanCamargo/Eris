#include "Eris.h"

#include <SPI.h>

namespace ADS131{

  //------------- Configuration
  uint8_t regs[] = {CONFIG1_ID, CONFIG2_ID, CONFIG3_ID, FAULT_ID,
                    CH0_ID, CH1_ID, CH2_ID, CH3_ID, CH4_ID, CH5_ID,
                    CH6_ID, CH7_ID};
  uint8_t sets[] = {CONFIG1_1K, CONFIG2, CONFIG3_BI, FAULT, CH0_1,
                    CH0_1, CH0_1, CH0_1, CH0_1, CH0_1,
                    CH0_1, CH0_1};
  uint8_t vref = 2.4;
  uint8_t gain = 1;
  
  //------------- GPIO pins
  int ss0 = PIN_ADC0_SS;
  int drdy0 = PIN_ADC0_DRDY;
  int start0 = PIN_ADC0_START;
  int reset0 = PIN_ADC0_RESET;
  int pwdn0 = PIN_ADC0_PWDN;
  
  //------------- SPI setup
  SPISettings SPIconfig(16000000, MSBFIRST, SPI_MODE1);
  
  //---------------- Collection vars ----------------------
  uint8_t byteCount = 0;
  uint8_t SPIbuffer[32];

  static binary_semaphore_t xConvertBytesSemaphore;
  thread_t *convertBytes;
  
  long data_temp; //uint32_t
  bool configStatus = false;
  long startTime = 0;
  
  ErisBuffer<EMGSample_t> buffer;
  
//------------- Internal helper functions
  void transferBytes(int numBytes)
  {
    //helper function to transfer data in SPIbuffer over SPI
    /*
    for(int i = 0; i < numBytes; i++)
    {
      Serial.println("Byte " + String(i) + " sent (HEX, BIN): " + String(SPIbuffer[i], HEX) + ", " + String(SPIbuffer[i], BIN)); 
    }
    */
    digitalWrite(ss0, LOW);
    delayMicroseconds(2);
    for(int i = 0; i < numBytes; i++)
    {
      SPIbuffer[i] = SPI.transfer(SPIbuffer[i]);
      delayMicroseconds(2);
    }
    delayMicroseconds(2);
    digitalWrite(ss0, HIGH);
  
    /*
    for(int i = 0; i < numBytes; i++)
    {
      Serial.println("Byte " + String(i) + " returned (BIN): " + String(SPIbuffer[i], BIN)); 
    }
    */
  }

  void startup()
  { 
    //startup sequence  
    digitalWrite(pwdn0, HIGH);
    digitalWrite(start0, HIGH);
    
    //CLKSEL pin tied high
    delayMicroseconds(100);
  
    digitalWrite(reset0, HIGH);
    delayMicroseconds(100);
    digitalWrite(start0, LOW);
    
    delay(3);//delay(ms)
    SPIbuffer[0] = SDATAC;
    transferBytes(1);
  
    configStatus = reset_config();
  }
  
  void collectData()
  {
    chSysLockFromISR();
    
    memset(SPIbuffer,0,sizeof(SPIbuffer));
    digitalWrite(ss0, LOW);
    SPI.transfer(SPIbuffer, 27);
    digitalWrite(ss0, HIGH);

    chBSemSignalI(&xConvertBytesSemaphore);
    chSysUnlockFromISR();
  }
  
  static THD_WORKING_AREA(waConvertBytes_T, 256);
  static THD_FUNCTION(ConvertBytes_T, arg) {
    //take it initially
    chBSemWait(&xConvertBytesSemaphore);
    
    while(1) {
  
      chBSemWait(&xConvertBytesSemaphore);
      
      chSysLockFromISR();

      EMGSample_t sample;
      sample.timestamp=(float)(micros() - t0)/1000.0 ;
      
      for(int chan = 0; chan < EMG_NUMCHANNELS; chan++)
      {
        //convert bytes to voltage readings
        data_temp = (((long)(signed char)(SPIbuffer[3*chan + 3])) << 16) | (((long)(SPIbuffer[3*chan + 4])) << 8) | ((long)(SPIbuffer[3*chan + 5]));
        sample.ch[chan]=2*(vref)*(data_temp)/(pow(2, 24)-1);
      }
      buffer.append(sample);  
      chSysUnlockFromISR();
    }
  }
    
//-------------- Externally callable functions
  bool reset_config()
  {
    //--------- reset sequence
    digitalWrite(reset0, LOW);
    delay(500);
    digitalWrite(reset0, HIGH);
    delay(500);
    //start/stop collecting
    SPIbuffer[0] = RDATAC;
    transferBytes(1);//enable read data continuous mode
    delay(2);
    digitalWrite(start0, HIGH);
    delay(10);
    digitalWrite(start0, LOW);
    delay(2);
    SPIbuffer[0] = SDATAC;
    transferBytes(1);//disable read data continuous mode

    //-------- configure ADC
    for(uint8_t i = 0; i < (sizeof(regs)/sizeof(regs[0])); i++)
    {
      //write
      SPIbuffer[0] = WREG << 4 | regs[i];
      SPIbuffer[1] = 0;
      SPIbuffer[2] = sets[i];
      transferBytes(3);
      
      //verify
      SPIbuffer[0] = RREG << 4 | regs[i];
      SPIbuffer[1] = 0;
      SPIbuffer[2] = 0;
      transferBytes(3);
  
      int8_t diff = SPIbuffer[2] - sets[i];//last bit on CONFIG3 register can return either 0 or 1
      if(abs(diff) > 2)//DONT DO MATH INSIDE ABS() -- incorrect
      {
         return false;
      }
    }
    return true;
  }
  
  void startCollecting()
  {
    if (configStatus) {
      SPIbuffer[0] = RDATAC;
      transferBytes(1);//enable read data continuous mode
      attachInterrupt(digitalPinToInterrupt(drdy0), collectData, FALLING);
      digitalWrite(start0, HIGH);
      startTime = micros();
    } else {
      eriscommon::printText("ADC configuration failed");
    }
  }
  
  void stopCollecting()
  {
    digitalWrite(start0, LOW);
    detachInterrupt(drdy0);
    SPIbuffer[0] = SDATAC;
    transferBytes(1);//disable read data continuous mode
  }
  
  void start()
  {
    SPI.beginTransaction(SPIconfig);

    //Set up pins
    pinMode(ss0, OUTPUT);
    pinMode(drdy0, INPUT);
    pinMode(start0, OUTPUT);
    pinMode(pwdn0, OUTPUT);
    pinMode(reset0, OUTPUT);
  
    digitalWrite(ss0, HIGH);
    digitalWrite(reset0, LOW);
    digitalWrite(pwdn0, LOW);
    digitalWrite(start0, LOW);

    //Initialize buffers
    buffer.init();
    
    startup();

    chBSemObjectInit(&xConvertBytesSemaphore, true);
    convertBytes = chThdCreateStatic(waConvertBytes_T, sizeof(waConvertBytes_T),
                                        NORMALPRIO, ConvertBytes_T, NULL);
  }
}
