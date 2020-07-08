#include "Eris.h"
#include "emg.h"

#include "sdcard.h"

#include <SdFat.h>
#include <SPI.h>
#include <fastdualbuffer.h>

namespace SDCard {

//const int chipSelect = BUILTIN_SDCARD;
SdFatSdio SD;
thread_t *writeFiles = NULL;
static binary_semaphore_t xbufferFullSemaphore;
static bool recording = false;
static bool isSDOK = false;
static char strbuffer[128];
char * DEFAULT_TRIALNAME = (char*)"test";;
char * trialname = DEFAULT_TRIALNAME;
File dataFile;

long startTime;

/******************** Custom code should go here ****************************/
// Create file names and file objects for each type of data to store
String sineFileName;
static File sineFile;
String emgFileName;
static File emgFile;
String fsrFileName;
static File fsrFile;

File * currFile;

// Define types of buffer data to store in SDCard (each one should map to a file)
static enum BUFFERTYPE {
  SINE,
  EMG ,
  FSR
} bufferType;

// Create the buffers to temporary store data before sending to SD card
// In this case I will use FastDualBuffer, but any storage can be used.

// Buffer for sine values
static FastDualBuffer sineDataBuffer(SDBUFFERSIZE);
// Buffer for EMG values -multichannel-
static FastDualBuffer * EMGDataBuffers[NUMEMGCHANNELS];
// Buffer for FSR values
static FastDualBuffer * FSRDataBuffers[NUMFSRCHANNELS];

// Buffers for timestamps
static FastDualBuffer EMGTimeBuffer(SDBUFFERSIZE);
static FastDualBuffer FSRTimeBuffer(SDBUFFERSIZE);
static FastDualBuffer sineTimeBuffer(SDBUFFERSIZE);

time_measurement_t t;

static THD_WORKING_AREA(waWriteFiles_T, 4096);
static THD_FUNCTION(WriteFiles_T, arg) {
  while (true) {
    msg_t msg = chBSemWaitTimeout(&xbufferFullSemaphore, MS2ST(10));
    if (msg == MSG_TIMEOUT) {
      continue;
    }
    if (!recording) {
      continue;
    }
    switch (bufferType) {
      case SINE:
        {
          //Open File
          currFile = &sineFile;
          if (! *currFile) {
            Error::RaiseError(MEMORY, (char *)"sine.csv");
            continue;
          }
          //Save into SDCARD
          int buffersize = sineDataBuffer.size();
          float * data = sineDataBuffer.read();
          float * dataTime;
          dataTime = sineTimeBuffer.read();
          for (int i = 0; i < buffersize; i++) {
            int num = sprintf(strbuffer, "%1.4f,%1.4f\n", dataTime[i], data[i]);
            currFile->write(strbuffer, num);
          }
          //Flush File
          currFile->flush();
        }
        break;
      case EMG:
        {
          //Open File
          currFile = &emgFile;
          if (! *currFile) {
            Error::RaiseError(MEMORY, (char *)emgFileName.c_str());
            continue;
          }
          //Save into SDCARD
          int buffersize = EMGDataBuffers[0]->size();
          float * data[NUMEMGCHANNELS];
          float * dataTime;
          dataTime = EMGTimeBuffer.read();
          for (uint8_t emgChan = 0; emgChan < NUMEMGCHANNELS; emgChan++) {
            data[emgChan] = EMGDataBuffers[emgChan]->read();
          }
          for (int i = 0; i < buffersize; i++) {            
            //int num = sprintf(&strbuffer[0], "%1.4f,%1.4f", dataTime[i], data[0][i]);
            int num=0;
            memcpy(&strbuffer[0],&dataTime[i],sizeof(float));
            num+=sizeof(float);
            memcpy(&strbuffer[num],&data[0][i],sizeof(float));
            num+=sizeof(float);
            for (uint8_t emgChan = 1; emgChan < NUMEMGCHANNELS; emgChan++) {
              //num = num + sprintf(&strbuffer[num], ",%1.4f", data[emgChan][i]);
              memcpy(&strbuffer[num],&data[emgChan][i],sizeof(float));              
              num+=sizeof(float);
            }
            num = num + sprintf(&strbuffer[num], "\n");
            //Serial.println(((float)(micros() - SDCard::startTime)) / 1000.0);            
            currFile->write(strbuffer, num);
          }
          //Flush File          
          currFile->flush();
        }
        break;
      case FSR:
        {
          //Open File
          currFile = &fsrFile;
          if (! *currFile) {
            Error::RaiseError(MEMORY, (char *)fsrFileName.c_str());
            continue;
          }
          //Save into SDCARD
          int buffersize = FSRDataBuffers[0]->size();
          float * data[NUMFSRCHANNELS];
          float * dataTime;
          dataTime = FSRTimeBuffer.read();
          for (uint8_t fsrChan = 0; fsrChan < NUMFSRCHANNELS; fsrChan++) {
            data[fsrChan] = FSRDataBuffers[fsrChan]->read();
          }
          for (int i = 0; i < buffersize; i++) {
            int num = sprintf(&strbuffer[0], "%1.4f,%1.4f", dataTime[i], data[0][i]);
            for (uint8_t fsrChan = 1; fsrChan < NUMFSRCHANNELS; fsrChan++) {
              num = num + sprintf(&strbuffer[num], ",%1.4f", data[fsrChan][i]);
            }
            num = num + sprintf(&strbuffer[num], "\n");
            currFile->write(strbuffer, num);
          }
          //Flush File
          currFile->flush();
        }
        break;
      default:
        break;
    }
  }
}

void addSine(float currTime, float value) {
  if (!recording) {
    return;
  }
  sineTimeBuffer.add(currTime);
  if (sineDataBuffer.add(value)) {
    bufferType = SINE;
    chBSemSignalI(&xbufferFullSemaphore);
  }
}

void addEMG(float value, float currTime, uint8_t chan) {
  if (!recording) {
    return;
  }
  if (chan == 0) {
    EMGTimeBuffer.add(currTime);
  }
  if (chan == (NUMEMGCHANNELS - 1)) {
    if (EMGDataBuffers[chan]->add(value)) {
      bufferType = EMG;
      chBSemSignalI(&xbufferFullSemaphore);
    }
  }
  else if (chan < NUMEMGCHANNELS) {
    EMGDataBuffers[chan]->add(value);
  }
}

void addFSR(float value, float currTime, uint8_t chan) {
  if (!recording) {
    return;
  }
  if (chan == 0) {
    FSRTimeBuffer.add(currTime);
  }
  if (chan == (NUMFSRCHANNELS - 1)) {
    if (FSRDataBuffers[chan]->add(value)) {
      bufferType = FSR;
      chBSemSignalI(&xbufferFullSemaphore);
    }
  }
  else if (chan < NUMFSRCHANNELS) {
    FSRDataBuffers[chan]->add(value);
  }
}


bool CreateFiles(void) {
  // Create the files for sensors in subfolders for each sensor
  // Sine file
  sineFileName = String("sine/") + String(trialname) + String(".csv");
  emgFileName = String("emg/") + String(trialname) + String(".bin");
  fsrFileName = String("fsr/") + String(trialname) + String(".csv");

  if (SD.exists(emgFileName.c_str())) {
    SD.remove(emgFileName.c_str());
    SD.remove(fsrFileName.c_str());
    SD.remove(sineFileName.c_str());
    eriscommon::printText("File already exists. Overwriting..");
  }

  SD.mkdir("sine");
  sineFile = SD.open(sineFileName.c_str(), FILE_WRITE);
  if (!sineFile) {
    return false;
  }
  sineFile.println("sample\tsine(units)");
  //dataFile.close();

  // EMG File
  SD.mkdir("emg");
  emgFile = SD.open(emgFileName.c_str(), O_WRITE | O_CREAT);
  if (!emgFile) {
    return false;
  }
  emgFile.print("Time(ms)");
  for (int i = 0; i < NUMEMGCHANNELS; i++) {
    emgFile.printf(",chan%d(V)", i);
  }
  emgFile.println();
  //dataFile.close();

  // FSR File
  SD.mkdir("fsr");
  fsrFile = SD.open(fsrFileName.c_str(), FILE_WRITE);
  if (!fsrFile) {
    return false;
  }
  fsrFile.print("Time(ms)");
  for (int i = 0; i < NUMFSRCHANNELS; i++) {
    fsrFile.printf(",fsr%d(V)", i);
  }
  fsrFile.println();
  //dataFile.close();
  return true;
}

/*******************************************************************************/

bool initSD(void) {
  if (!SD.begin()) {
    eriscommon::printText("initialization failed. Things to check:");
    eriscommon::printText("* is a card inserted?");
    eriscommon::printText("* is your wiring correct?");
    eriscommon::printText("* did you change the chipSelect pin to match your shield or module?");
    return false;
  } else {
    eriscommon::printText("SD card is present.");
    isSDOK = true;
    return true;
  }
}

void StopRecording(void) {
  recording = false;

  sineFile.close();
  emgFile.close();
  fsrFile.close();
}

void StartRecording(void) {
  //Clear the buffers from any residual data from previous recordings
  sineDataBuffer.clear();

  for (uint8_t emgChan = 0; emgChan < NUMEMGCHANNELS; emgChan++) {
    EMGDataBuffers[emgChan]->clear();
  }

  for (uint8_t fsrChan = 0; fsrChan < NUMFSRCHANNELS; fsrChan++) {
    FSRDataBuffers[fsrChan]->clear();
  }
  EMGTimeBuffer.clear();
  FSRTimeBuffer.clear();

  if (!isSDOK) {
    Error::RaiseError(MEMORY, (char *)"SDCARD");
    return;
  }

  if (CreateFiles()) {
    recording = true;
    startTime = micros();
  }
  else {
    recording = false;
    Error::RaiseError(MEMORY, (char *)"SDCARD");
  }
}

void setTrialName(char * newtrialname) {
  trialname = newtrialname;
}


void start(void) {
  recording = false;
  chBSemObjectInit(&xbufferFullSemaphore, true);
  chTMObjectInit(&t);


  // Initialize SDCard
  initSD();

  for (uint8_t emgChan = 0; emgChan < NUMEMGCHANNELS; emgChan++) {
    EMGDataBuffers[emgChan] = new FastDualBuffer(SDBUFFERSIZE);
  }

  for (uint8_t fsrChan = 0; fsrChan < NUMFSRCHANNELS; fsrChan++) {
    FSRDataBuffers[fsrChan] = new FastDualBuffer(SDBUFFERSIZE);
  }

  // create tasks at priority lowest priority
  writeFiles = chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T), NORMALPRIO, WriteFiles_T, NULL);
}
}
