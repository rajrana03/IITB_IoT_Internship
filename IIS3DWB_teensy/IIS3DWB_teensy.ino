#include "IIS3DWB.h"
#include "SPI.h"
#include <Arduino.h>

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 2
#define CSPIN 36

String control_val = "";

//IIS3DWB definitions
// #define IIS3DWB_intPin1 34  // interrupt1 pin definitions, data ready
// #define IIS3DWB_intPin2 35  // interrupt2 pin definitions, activity detection

/* Specify sensor parameters (sample rate is same as the bandwidth 6.3 kHz by default)
 * choices are:  AFS_2G, AFS_4G, AFS_8G, AFS_16G  
*/
bool Send_Garabage_Flag = false;
int16_t Garbage_value = -5000;

// uint8_t Ascale = AFS_4G;
uint8_t Ascale = AFS_2G;
float aRes;                                 // scale resolutions per LSB for the accel
float accelBias[3] = { 0.0f, 0.0f, 0.0f };  // offset biases for the accel
int16_t IIS3DWBData[3] = { 0 };             // Stores the 16-bit signed sensor output
int16_t ax, ay, az;                         // variables to hold latest accel data values
uint8_t IIS3DWBstatus;
const float acc_mult_factor = 1000.0;  //convert to mili g

long timer1, timer2, diff;
volatile bool IIS3DWB_DataReady = false, IIS3DWB_Wakeup = false;
int count = 10;

IIS3DWB IIS3DWB(CSPIN);  // instantiate IIS3DWB class

void IIS3DWB_SETUP() {
  //Serial.println("IIS3DWB is online...");
  //Serial.println(" ");

  // reset IIS3DWB to start fresh
  IIS3DWB.reset();
  delay(1000);
  digitalWrite(myLed, LOW);  // indicate passed the ID CHECK

  // get accel sensor resolution, only need to do this once
  aRes = IIS3DWB.getAres(Ascale);

  //   IIS3DWB.selfTest();

  IIS3DWB.init(Ascale);  // configure IIS3DWB

  IIS3DWB.offsetBias(accelBias);
  delay(10);
}

uint8_t IIS3DWB_CHECK() {
  while ((IIS3DWB.getChipID() != 0x7B) && (count > 0)) {
    delayMicroseconds(10);
    count--;
  }
  uint8_t c = IIS3DWB.getChipID();
  count = 10;
  return c;
}

void setup() {
  Serial.begin(1500000);
  // Serial.begin(115200);
  //  delay(4000);
  // SPI1.setMOSI(26);
  // SPI1.setMISO(39);
  // SPI1.setSCK(27);
  SPI.begin();  // Start SPI 1 instance of teensy, for SPI default use SPI.begin()

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with led off

  //  delay(20); // wait at least 10 ms for IIS3DWB boot procedure to complete

  // Configure SPI ship select for sensor breakout
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);  // disable SPI at start

  // Configure interrupt pins
  //pinMode(IIS3DWB_intPin1, INPUT); // enable IIS3DWB interrupt1
  // pinMode(IIS3DWB_intPin2, INPUT); // enable IIS3DWB interrupt2

  // Read the IIS3DWB Chip ID register, this is a good test of communication
  //Serial.println("IIS3DWB accel...");
  uint8_t c = IIS3DWB_CHECK();  // Read CHIP_ID register for IIS3DWB
                                //  Serial.print("IIS3DWB "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x7B, HEX);
  //Serial.println(" ");

  // c = 0x11;
  delay(2000);


  if (c != 0x7B) {
    // Serial.end();
    // c = IIS3DWB.getChipID();
    Send_Garabage_Flag = true;
  } else if (c == 0x7B)  // CHECK if all SPI sensors have acknowledged
  {
    IIS3DWB_SETUP();
  }
  // else
  // {
  // // if(c != 0x6A) //Serial.println(" IIS3DWB not functioning!");
  // while(1){};
  // }

  // attachInterrupt(IIS3DWB_intPin1, myinthandler1, RISING);   // define interrupt for intPin1 output of IIS3DWB
  // attachInterrupt(IIS3DWB_intPin2, myinthandler2, FALLING);  // define interrupt for intPin2 output of IIS3DWB
}
/* End of setup */

void loop() {
while(1){
  if (!Send_Garabage_Flag) {
    for (;;) {
      IIS3DWBstatus = IIS3DWB.DRstatus();  // read data ready status
      if (IIS3DWBstatus & 0x01) {          // if new accel data is available, read it

        IIS3DWB.readAccelData(IIS3DWBData);

        // Now we'll calculate the accleration value into actual g's
        ax = acc_mult_factor * ((float)IIS3DWBData[0] * aRes - accelBias[0]);  // get actual g value in mg, this depends on scale being set
        ay = acc_mult_factor * ((float)IIS3DWBData[1] * aRes - accelBias[1]);
        az = acc_mult_factor * ((float)IIS3DWBData[2] * aRes - accelBias[2]);
        Serial.write(0xAA);  // Send the start/sync byte
        Serial.write((uint8_t*)&(ax), sizeof(ax));
        Serial.write((uint8_t*)&(ay), sizeof(ay));
        Serial.write((uint8_t*)&(az), sizeof(az));
        delayMicroseconds(25);
      } else if (IIS3DWB.getChipID() != 0x7B) {
        Send_Garabage_Flag = true;
        break;
      }
    }
  } else {
    for (;;) {
      // timer1 = micros();
      Serial.write(0xAA);  // Send the start/sync byte
      Serial.write((uint8_t*)&(Garbage_value), sizeof(Garbage_value));
      Serial.write((uint8_t*)&(Garbage_value), sizeof(Garbage_value));
      Serial.write((uint8_t*)&(Garbage_value), sizeof(Garbage_value));
      delayMicroseconds(1);
      uint8_t c = IIS3DWB.getChipID();
      if (c == 0x7B) {
        IIS3DWB_SETUP();
        Send_Garabage_Flag = false;
        break;
      }
      // timer2 = micros();
      // diff = timer2 - timer1;
      // Serial.println(diff);
      // delay(1000);
    }
  }
}
}


