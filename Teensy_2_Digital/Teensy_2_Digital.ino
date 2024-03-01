#include "IIS3DWB.h"
#include "SPI.h"
#include <MPU9250_WE.h>
#include <Arduino.h>

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 4
#define CSPIN 38
#define MPU_CS_PIN 10

String control_val = "";

const int8_t SYNC_BYTE = 0xAA;
const int8_t SYNC_BYTE_2 = 0xAB;
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

// uint8_t Ascale = AFS_4G;
uint8_t Ascale = AFS_8G;
float aRes;                                 // scale resolutions per LSB for the accel
float accelBias[3] = { 0.0f, 0.0f, 0.0f };  // offset biases for the accel
int16_t IIS3DWBData[3] = { 0 };             // Stores the 16-bit signed sensor output
int16_t ax, ay, az;                         // variables to hold latest accel data values
uint8_t IIS3DWBstatus;
const float acc_mult_factor = 1000.0;  //convert to mili g

// long timer1, timer2, diff;
volatile bool IIS3DWB_DataReady = false, IIS3DWB_Wakeup = false;
int count = 10;

IIS3DWB IIS3DWB(CSPIN);  // instantiate IIS3DWB class

MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);

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
  Serial2.begin(115200);

  // SerialUSB1.begin(115200);

  //  delay(4000);
  SPI1.setMOSI(26);
  SPI1.setMISO(39);
  SPI1.setSCK(27);
  SPI1.begin();  // Start SPI 1 instance of teensy, for SPI default use SPI.begin()


  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  SPI.begin();
  myMPU9250.init();

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
  lastMicros_1 = 0;
  lastMicros_2 = 0;
  timer1 = 0;
  timer2 = 0;

  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);  // bandwdith without DLPF
  // myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
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
  xyzFloat gyr = myMPU9250.getGyrValues();
  while (1) {

    // if ( micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec) {
    //   lastMicros = micros();
    //   //
    // }

    if (micros() >= lastMicros_1 + 40)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec
    {
      lastMicros_1 = micros();

      if (!Send_Garabage_Flag) {
        IIS3DWBstatus = IIS3DWB.DRstatus();  // read data ready status
        if (IIS3DWBstatus & 0x01) {          // if new accel data is available, read it

          IIS3DWB.readAccelData(IIS3DWBData);

          // Now we'll calculate the accleration value into actual g's
          ax = ((float)IIS3DWBData[0] * aRes - accelBias[0]) * acc_mult_factor;  // get actual g value in mg, this depends on scale being set
          ay = ((float)IIS3DWBData[1] * aRes - accelBias[1]) * acc_mult_factor;
          az = ((float)IIS3DWBData[2] * aRes - accelBias[2]) * acc_mult_factor;

        } else if (IIS3DWB.getChipID() != 0x7B) {
          Send_Garabage_Flag = true;
          break;
        }
      }

      gyr = myMPU9250.getGyrValues();
      Gx = gyr.x * 100;
      Gy = gyr.y * 100;
      Gz = gyr.z * 100;
      Serial.write(0xAA);  // Send the start/sync byte
      Serial.write(SYNC_BYTE_2);
      Serial.write((uint8_t *)&(lastMicros_1), sizeof(lastMicros_1));  //Timestamp
      if (!Send_Garabage_Flag) {

        Serial.write((uint8_t *)&(ax), sizeof(ax));
        Serial.write((uint8_t *)&(ay), sizeof(ay));
        Serial.write((uint8_t *)&(az), sizeof(az));
      } else {
        Serial.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));
        Serial.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));
        Serial.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));
        uint8_t c = IIS3DWB.getChipID();
        if (c == 0x7B) {
          IIS3DWB_SETUP();
          Send_Garabage_Flag = false;
          // break;
        }
      }
      Serial.write((uint8_t *)&(Gx), sizeof(Gx));
      Serial.write((uint8_t *)&(Gy), sizeof(Gy));
      Serial.write((uint8_t *)&(Gz), sizeof(Gz));
    }

    // SerialUSB1.write(0XAA);
    // SerialUSB1.write(0XBB);
    // if (SerialUSB1.available() > 0) {
    //   byte rxVal = SerialUSB1.read();
    //   // rxVal.trim();
    //   // SerialUSB1.println(rxVal);
    //   delay(3000);
    //   if (rxVal == 0xAB) {  // 0xAA
    //     Serial2.write(0XAA);
    //     // delay(10000);
    //     while (Serial2.available() > 0) {
    //       byte rxVal = Serial2.read();
    //       // rxVal.trim();
    //       // SerialUSB1.println(rxVal);
    //       delay(100);
    //       if (rxVal == 0xBB) {  // 0xAA
    //         SerialUSB1.write(0XCC);
    //         delay(10000);
    //         break;
    //       }
    //     }
    //   }
    // }
  }
}
