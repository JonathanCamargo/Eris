/* Test code for ADS1256_teensy_lib library. This code creates an ADS1256 object
defined by the libary, turn it to collecting mode and let it run for 1 second, and 
turn off collecting mode. With results in a new file saved with data from the 
one second of the ADS1256 collecting.
 */

#include <SPI.h>
#include <SD.h>

#include "ADS1256.h"
#include "string.h"

#define PIN_ADC0_SS 6
#define PIN_ADC0_DRDY 14

#define DAQTIME 1000

String filename = "EMG1.csv";
char filebuf[50];
int filenumber = 1;
File myFile;
int counter = 0;
long startTime;


// Create the object and the function that holds the interrupt
ADS1256 sensor1(PIN_ADC0_SS,PIN_ADC0_DRDY);


// Setup interrupts to collect data and to detach interrupts when the buffer is going to overflow
volatile bool bufferFlag=false; // Use as a flag to enable buffer reading from the main loop.
static void interruptHandler(void) {
 if (sensor1.collectData()){    
    bufferFlag=true;    
  } 
}

int count = 0;

void setup() {
  // Begin serial connection and wait until it is activated
  Serial.begin(115200);
  while (!Serial);
 
  // Check if an SD card is inserted into the SD card slot, quit the initialization if there is no SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return;
  }
  
  // Display that the SD is inserted and connection is found
  Serial.println("initialization done.");
  // Create a new file with the format "EMG#.csv" where # is a number that has not been used yet in the SD card
  filename.toCharArray(filebuf,50);
  while (SD.exists(filebuf)) {
    filenumber++;
    filename = "EMG" + String(filenumber) + ".csv";
    filename.toCharArray(filebuf,50);
  }
  
  // Turn the ADS1256 into Continuous read mode
  sensor1.startCollecting();
  
  // Attach the interrupt tot he DRDY pin defined by the class
  attachInterrupt(PIN_ADC0_DRDY,interruptHandler,FALLING);
  
  // Define the start of the program time
  startTime = millis();
}


uint16_t buffersize=sensor1.bufferSize(); //Get the buffer size of the library
bool wasFull=false;
void loop() {

  if ((millis()-startTime)< DAQTIME && bufferFlag){
    // You need to guarantee that this runs in less time that what it takes to fill out bufferSize at the defined sampling rate
    // In this example we use wasFull flag to make sure we do not overflow the buffer.
    float * data=sensor1.get_data();
    for (uint16_t i=0;i<buffersize;i++){
      myFile.println(data[i],3);
      }
    if (wasFull==true){
      Serial.println("Error buffer overflow");
    }
    wasFull=true;    
  }
  else if ((millis()-startTime)> DAQTIME){
    detachInterrupt(PIN_ADC0_DRDY);
    Serial.println("Done Printing");
    myFile.close();    
  }
  else{
    wasFull=false;
  }
  
}
