//-----------------------------------------------------------------------------------------
// Apnemeter v3.5
// Shanaka Liyanaarachchi
//------------------------------------------------------------------------------------------

/*
*/
//----------------------------------------------------------------------------------------
// Import libraries
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <SoftwareSerial.h>

//---------------------------------------------------------------------------------------
SoftwareSerial mySerial(10,11); //RX,TX
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
//----------------------------------------------------------------------------------------
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
//----------------------------------------------------------------------------------------------
// Define Variables
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


//------------------------------------------------------------------------------------------------------------
// Initial Setup
void setup()
{
  mySerial.begin(9600); // initialize hc-06 serial communication

  pinMode(8, OUTPUT);

  // Initialize MAX30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
  digitalWrite(8,HIGH);
    while (1);
  }

  while (mySerial.available() == 0) ; //wait until user presses a key
  mySerial.read();
//-----------------------------------------------------------------------------------------------------------
// Setup MAX30102 Sensor
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

//--------------------------------------------------------------------------------------------------------
// Loop

void loop()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps sample rate

  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) 
      particleSensor.check(); //Check the sensor for new data 

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //move to the next sample

  }

//-------------------------------------------------------------------------------------------------------------------------
  //calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
// take the sound level
  int soundLevel = analogRead(A0);

//--------------------------------------------------------------------------------------------------------------------------
// Serial communication using hc-06
  mySerial.print(heartRate, DEC);
  mySerial.print(F(","));
  mySerial.print(spo2, DEC);  
  mySerial.print(F(","));
  mySerial.println(soundLevel, DEC);
//--------------------------------------------------------------------------------------------------------------------------
  delay(1000);
  
}
