#include "Eris.h"
#include "MPU9250.h"
#include "sdcard.h"
#include "configuration.h"


#include <FeatureExtractor.h>

namespace IMU{
  
thread_t *extractFeaturesIMU = NULL;
thread_t *readAnalog = NULL;

IntervalTimer timer0; // Timer for triggering IMU collection

const char * channelNames[]={"ITR","ITH","ISH","IFO"};
int failures = -1;
MPU9250 imuTrunk(SPI,PIN_IMU_TRUNK);
MPU9250 imuThigh(SPI,PIN_IMU_THIGH);
MPU9250 imuShank(SPI,PIN_IMU_SHANK);
MPU9250 imuFoot(SPI,PIN_IMU_FOOT);
MPU9250 * imuptr;
//Buffer for readings 
ErisBuffer<IMUSample_t> bufferTrunk;
ErisBuffer<IMUSample_t> bufferThigh;
ErisBuffer<IMUSample_t> bufferShank;
ErisBuffer<IMUSample_t> bufferFoot;

// Feature extractors
FeatureExtractorPtr trunkExtractors[6];
FeatureExtractorPtr thighExtractors[6];
FeatureExtractorPtr shankExtractors[6];
FeatureExtractorPtr footExtractors[6];

void ExtractorAddSampleHelper(FeatureExtractorPtr * extractorPtr, IMUSample_t sample ){
  extractorPtr[0]->newSample(sample.ax);
  extractorPtr[1]->newSample(sample.ay);
  extractorPtr[2]->newSample(sample.az);
  extractorPtr[3]->newSample(sample.wx);
  extractorPtr[4]->newSample(sample.wy);
  extractorPtr[5]->newSample(sample.wz);
}

void IMUGetDataHelper(MPU9250 * imu, IMUSample_t & sample ){
  sample.ax=imu->getAccelX_mss();
  sample.ay=imu->getAccelY_mss();
  sample.az=imu->getAccelZ_mss();
  sample.wx=imu->getGyroX_rads();
  sample.wy=imu->getGyroY_rads();
  sample.wz=imu->getGyroZ_rads();
}

// Indices and flags
static void ISR_NewSample(){  
  chSysLockFromISR();
  digitalWrite(PIN_LED,HIGH);
  float timestamp = ((float)(micros() - t0))/1000.0;  
  //Sample every channel
  IMUSample_t thisSample;
  thisSample.timestamp=timestamp; 
  
  //Trunk  
  imuTrunk.readSensor();
  IMUGetDataHelper(&imuTrunk,thisSample);   
  bufferTrunk.append(thisSample);
  #if SDCARD    
    SDCard::imutrunkbuffer.append(thisSample);    
  #endif 
  #if FEATURES    
    ExtractorAddSampleHelper(trunkExtractors, thisSample );     
  #endif 
  
  imuThigh.readSensor();
  IMUGetDataHelper(&imuThigh,thisSample);   
  bufferThigh.append(thisSample);
  #if SDCARD    
    SDCard::imuthighbuffer.append(thisSample);    
  #endif 
   #if FEATURES    
    ExtractorAddSampleHelper(thighExtractors, thisSample );     
    //Serial.println(thighExtractors[1]->features[0]);
  #endif 
   
  imuShank.readSensor();
  IMUGetDataHelper(&imuShank,thisSample);   
  bufferShank.append(thisSample);
  #if SDCARD    
    SDCard::imushankbuffer.append(thisSample);    
  #endif 
   #if FEATURES    
    ExtractorAddSampleHelper(shankExtractors, thisSample ); 
  #endif 
  
  imuFoot.readSensor();
  IMUGetDataHelper(&imuFoot,thisSample);   
  bufferFoot.append(thisSample);
  #if SDCARD    
    SDCard::imufootbuffer.append(thisSample);    
  #endif 
   #if FEATURES    
    ExtractorAddSampleHelper(footExtractors, thisSample); 
  #endif 
  
  digitalWrite(PIN_LED,LOW);
  chSysUnlockFromISR();  
}

void RegisterExtractors(FeaturesHelper * featuresHelper){
//Link the feature extractors from this module to a features helper
  FeatureExtractorPtr * imuExtractors[4]={trunkExtractors,thighExtractors,shankExtractors,footExtractors};  
  for (uint8_t imuIdx=0;imuIdx<4;imuIdx++){
    FeatureExtractorPtr * e= imuExtractors[imuIdx];
    for (uint8_t i=0;i<6;i++){          
      char chname[5]; char tmp[2];
      strcpy(chname,channelNames[imuIdx]);
      sprintf(tmp,"%d",i);
      strcat(chname,tmp);
      featuresHelper->RegisterExtractor(e[i],IMU_FREQUENCY_HZ,chname);
    }
  }
}


void start(void){   
  
  failures=0;
  // Start ErisBuffers            
  bufferTrunk.init();   
  bufferThigh.init();   
  bufferShank.init();   
  bufferFoot.init();            

  //Create extractors
  for (uint8_t i=0;i<6;i++){
    trunkExtractors[i]=new FeatureExtractor();
    thighExtractors[i]=new FeatureExtractor();
    shankExtractors[i]=new FeatureExtractor();
    footExtractors[i]=new FeatureExtractor();
  }

  imuptr=&imuTrunk;
	// Start the IMU configuration 
  int status = imuptr->begin();
	if (status<0){
    failures |= B1000;
		Serial.println("Trunk IMU initialization unsuccessful");
	}
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0,1);
  imuptr->setAccelCalY(0,1);
  imuptr->setAccelCalZ(0,1);

  imuptr=&imuThigh;
  status = imuptr->begin();
  if (status<0){
    failures |= B0100;
    Serial.println("Thigh IMU initialization unsuccessful");
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0.15,1);
  imuptr->setAccelCalY(0.15,1);
  imuptr->setAccelCalZ(0.29,0.98);

  imuptr=&imuShank;
  status = imuptr->begin();
  if (status<0){
    failures |= B0010;
    Serial.println("Shank IMU initialization unsuccessful");
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(0.04,1);
  imuptr->setAccelCalY(0.04,1);
  imuptr->setAccelCalZ(0.13,0.99);

  imuptr=&imuFoot;
  status = imuptr->begin();
  if (status<0){
    failures |= B0001;
    Serial.println("Foot IMU initialization unsuccessful");
  }
  imuptr->setSrd(0); // 1000Hz
  imuptr->setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imuptr->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imuptr->setAccelCalX(-.01,1);
  imuptr->setAccelCalY(-.04,1);
  imuptr->setAccelCalZ(-.05,.98);
   
  //Start samples semaphore for feature extraction
  //chBSemObjectInit(&xsamplesSemaphore,true);        
  // Initialize interrupt to take the samples      
  // Timer interrupt to take the ADC samples        
  //Set up ADC
  timer0.begin(ISR_NewSample, IMU_PERIOD_US);          

  }
}
