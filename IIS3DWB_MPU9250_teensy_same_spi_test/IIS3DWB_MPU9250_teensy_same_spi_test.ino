#include "IIS3DWB.h"
#include "SPI.h"
#include <Arduino.h>

#define SerialDebug true  // set to true to get Serial output for debugging
#define CSPIN 36
// #define myLed 13

String control_val = "";

const int8_t SYNC_BYTE = 0xAA;
// int16_t Ax{ 0 };
// int16_t Ay{ 0 };
// int16_t Az{ 0 };

int16_t Gx{ 0 };
int16_t Gy{ 0 };
int16_t Gz{ 0 };


//IIS3DWB definitions
// #define IIS3DWB_intPin1 34  // interrupt1 pin definitions, data ready
// #define IIS3DWB_intPin2 35  // interrupt2 pin definitions, activity detection

/* Specify sensor parameters (sample rate is same as the bandwidth 6.3 kHz by default)
 * choices are:  AFS_2G, AFS_4G, AFS_8G, AFS_16G  
*/
bool Send_Garabage_Flag = false;
int16_t Garbage_value = -5000;
unsigned long lastMicros_1, lastMicros_2, timer1, timer2;

uint8_t Ascale = AFS_8G;
// uint8_t Ascale = AFS_2G;
float aRes;                                 // scale resolutions per LSB for the accel
float accelBias[3] = { 0.0f, 0.0f, 0.0f };  // offset biases for the accel
int16_t IIS3DWBData[3] = { 0 };             // Stores the 16-bit signed sensor output
int16_t ax, ay, az;                         // variables to hold latest accel data values
uint8_t IIS3DWBstatus;
const float acc_mult_factor = 1000.0;  //convert to mili g

// long timer1, timer2, diff;
volatile bool IIS3DWB_DataReady = false, IIS3DWB_Wakeup = false;
int count = 10;
uint8_t c = 0;

IIS3DWB IIS3DWB(CSPIN);  // instantiate IIS3DWB class

// MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);

void IIS3DWB_SETUP() {
  //Serial.println("IIS3DWB is online...");
  //Serial.println(" ");

  // reset IIS3DWB to start fresh
  IIS3DWB.reset();
  delay(1000);
  // digitalWrite(myLed, LOW);  // indicate passed the ID CHECK

  // get accel sensor resolution, only need to do this once
  aRes = IIS3DWB.getAres(Ascale);

  //   IIS3DWB.selfTest();

  IIS3DWB.init(Ascale);  // configure IIS3DWB

  IIS3DWB.offsetBias(accelBias);
  delay(10);
}

uint8_t IIS3DWB_CHECK() {
  while ((IIS3DWB.getChipID() != 0x7B) && (count > 0)) {
    delayMicroseconds(100);
    count--;
  }
  uint8_t d = IIS3DWB.getChipID();
  count = 10;
  return d;
}

void setup() {
  Serial.begin(1500000);
  // Serial.begin(115200);
  //  delay(4000);
  // SPI1.setMOSI(26);
  // SPI1.setMISO(39);
  // SPI1.setSCK(27);
  // SPI1.begin();  // Start SPI 1 instance of teensy, for SPI default use SPI.begin()
  // SPI.begin(26,39,27,36);

  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  SPI.begin();
  // myMPU9250.init();

  // Configure led
  // pinMode(myLed, OUTPUT);
  // digitalWrite(myLed, HIGH);  // start with led off

  //  delay(20); // wait at least 10 ms for IIS3DWB boot procedure to complete

  // Configure SPI ship select for sensor breakout
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);  // disable SPI at start

  // Configure interrupt pins
  //pinMode(IIS3DWB_intPin1, INPUT); // enable IIS3DWB interrupt1
  // pinMode(IIS3DWB_intPin2, INPUT); // enable IIS3DWB interrupt2

  // Read the IIS3DWB Chip ID register, this is a good test of communication
  //Serial.println("IIS3DWB accel...");
  // uint8_t c = IIS3DWB_CHECK();  // Read CHIP_ID register for IIS3DWB
  //  Serial.print("IIS3DWB "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x7B, HEX);
  //Serial.println(" ");
  c = IIS3DWB_CHECK();
  // c = 0x11;
  delay(2000);
  lastMicros_1 = 0;
  lastMicros_2 = 0;
  timer1 = 0;
  timer2 = 0;

  // myMPU9250.autoOffsets();
  // myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);  // bandwdith without DLPF
  // // myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  // myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  // myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  // myMPU9250.enableAccDLPF(false);
  // myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  // myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);

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
  // xyzFloat gyr = myMPU9250.getGyrValues();
  while (1) {

    // if ( micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec) {
    //   lastMicros = micros();
    //   //
    // }
    if (micros() >= lastMicros_1 + 40)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec
    {
      lastMicros_1 = micros();
    // delayMicroseconds(40);
      // uint8_t c = IIS3DWB.getChipID();
      uint8_t IIS3DWBstatus = (IIS3DWB.DRstatus() & 0x01);

      // Serial.write(0xAA);
      // Serial.write(c);
      if (IIS3DWBstatus) {
        IIS3DWB.readAccelData(IIS3DWBData);
        // acc_mult_factor *
        // Now we'll calculate the accleration value into actual g's
        ax = acc_mult_factor * ((float)IIS3DWBData[0] * aRes - accelBias[0]);  // get actual g value in mg, this depends on scale being set
        ay = acc_mult_factor * ((float)IIS3DWBData[1] * aRes - accelBias[1]);
        az = acc_mult_factor * ((float)IIS3DWBData[2] * aRes - accelBias[2]);

        Serial.write(0xAA);  // Send the start/sync byte
        Serial.write((uint8_t*)&(ax), sizeof(ax));
        Serial.write((uint8_t*)&(ay), sizeof(ay));
        Serial.write((uint8_t*)&(az), sizeof(az));
        // Serial.write(IIS3DWBstatus);
      }
    }
  }
}
