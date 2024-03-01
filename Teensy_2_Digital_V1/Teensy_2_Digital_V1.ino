#include "IIS3DWB.h"     // Library for IIS3DWB sensor
#include "SPI.h"         // Library for SPI Protocol
#include <MPU9250_WE.h>  // Library for MPU9250 sensor

#define CSPIN 38       // CS pin for IIS3DWB
#define MPU_CS_PIN 10  //CS pin for MPU9250

const int8_t SYNC_BYTE = 0xAA;    //Initial sync byte to detect start of data
const int8_t SYNC_BYTE_1 = 0xAB;  //Another sync byte to avoid any overlapping of data and sync byte

float Gyro_bias[3]; // Declaration of Gyroscope Bias Variable
const int16_t NUM_OF_CALIBRATION_SAMPLES = 5000; // Taking 5000 mpu samples for averaging 

bool Send_Garabage_Flag = false; // Flag which turns true when IIS3DWB board disconnects
int16_t Garbage_value = -5000; // Value to send when IIS3DWB disconnects so that Process doesn't stop  

/* Specify sensor parameters (sample rate is same as the bandwidth 6.3 kHz by default)
   choices are:  AFS_2G, AFS_4G, AFS_8G, AFS_16G
*/

uint8_t Ascale = AFS_8G;
float aRes;                                 // scale resolutions per LSB for the accel
float accelBias[3] = { 0.0f, 0.0f, 0.0f };  // offset biases for the accel
const float acc_mult_factor = 1000.0;  //convert to mili g 
const float gyr_mult_factor = 10.0; // Get more floating point value in Gyro for better precision output
IIS3DWB IIS3DWB(CSPIN);  // instantiate IIS3DWB class - SPI1

MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);  // instantiate MPU9250 class - SPI

void IIS3DWB_SETUP() {
  // reset IIS3DWB to start fresh
  IIS3DWB.reset(); // Resets IIS3DWB to go to default settings of Registers //CHECK WHETHER THIS LEADS TO FAULT
  delay(1000); 

  // get accel sensor resolution, only need to do this once
  aRes = IIS3DWB.getAres(Ascale); // Returns conversion factor for given Ascale 

  IIS3DWB.init(Ascale);  // configure IIS3DWB

  IIS3DWB.offsetBias(accelBias); // Calculate IIS3DWB offsets to remove Bias due to initial positioning
  delay(10);
}

uint8_t IIS3DWB_CHECK() {
  uint8_t count = 10; // Tries 10 times to detect IIS3DWB Chip(~100us) 
  while ((IIS3DWB.getChipID() != 0x7B) && (count > 0)) {
    delayMicroseconds(10);
    count--;
  }
  uint8_t c = IIS3DWB.getChipID(); // WHOAMI checks whether IIS3DWB board is connected 
  return c;
}

void setup() {
  Serial.begin(1500000);  //serial port to get connected to PC at 15,00,000 baud rate
  Serial2.begin(115200);  //serial uart to communicate with Teensy_1 (Analog)

  SerialUSB1.begin(115200);  //dual serial port to get reset command from software

  //using SPI1 for IIS3DWB sensor
  SPI1.setMOSI(26);
  SPI1.setMISO(39);
  SPI1.setSCK(27);
  SPI1.begin();  // Start SPI 1 instance of teensy, for SPI default use SPI.begin()


  //using SPI for MPU9250 sensor
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  SPI.begin(); //  Start SPI instance of teensy
  bool mpu_success = myMPU9250.init();  // Adding check whether init is successful
  // while(!mpu_success){
  //   myMPU9250.init();
  //   delay(10);
  // }
  // Configure SPI ship select for sensor breakout
  pinMode(CSPIN, OUTPUT); 
  digitalWrite(CSPIN, HIGH);  // disable SPI at start


  // Read the IIS3DWB Chip ID register, this is a good test of communication
  //Serial.println("IIS3DWB accel...");
  uint8_t c = IIS3DWB_CHECK();  // Read CHIP_ID register for IIS3DWB

  delay(2000);

  // myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);  // bandwdith without DLPF
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);      //change back to 500
  calibrate_MPU(Gyro_bias); // Calibrate Gyroscope bias and store in Gyro_bias
  delay(1000);

  if (c != 0x7B) {
    Send_Garabage_Flag = true;
  } else if (c == 0x7B)  // CHECK if IIS3DWB has been acknowledged
  {
    IIS3DWB_SETUP(); // Loads Sensor Settings into IIS3DWB
  }
}
/* End of setup */

void loop() {
  unsigned long lastMicros_1 = 0;
  int16_t Gx=0,Gy=0,Gz=0;// Initialization to 0 of output Gyroscope Variable
  int16_t IIS3DWBData[3] = { 0 };  // Stores the 16-bit signed sensor output
  int16_t ax, ay, az;          // variables to hold latest accel data values
  xyzFloat gyr;  // = myMPU9250.getGyrValues();
  while (1) {

    if (micros() >= lastMicros_1 + 40)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec     here it is 25 KHz
    {
      lastMicros_1 = micros();

      if (!Send_Garabage_Flag) {
        uint8_t IIS3DWBstatus = IIS3DWB.DRstatus();  // read data ready status
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

      // gyr = myMPU9250.getGyrRawValues();
      gyr = myMPU9250.getGyrValues(); // Get Gyr Values in rad/s
      Gx = (gyr.x - Gyro_bias[0]) * gyr_mult_factor; // Get actual Gyr Value removing bias
      Gy = (gyr.y - Gyro_bias[1]) * gyr_mult_factor;
      Gz = (gyr.z - Gyro_bias[2]) * gyr_mult_factor;
      // Gx = gyr.x;
      if (!Send_Garabage_Flag) {
        Serial.write(SYNC_BYTE);
        Serial.write(SYNC_BYTE_1);
        Serial.write((uint8_t *)&(lastMicros_1), sizeof(lastMicros_1));  //Timestamp
        Serial.write((uint8_t *)&(ax), sizeof(ax));                      //Ax
        Serial.write((uint8_t *)&(ay), sizeof(ay));                      //Ay
        Serial.write((uint8_t *)&(az), sizeof(az));                      //Az
      } else {
        Serial.write(SYNC_BYTE);
        Serial.write(SYNC_BYTE_1);
        Serial.write((uint8_t *)&(lastMicros_1), sizeof(lastMicros_1));    //Timestamp
        Serial.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));  //Ax (-5000)
        Serial.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));  //Ay (-5000)
        Serial.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));  //Az (-5000)
        uint8_t c = IIS3DWB.getChipID(); // Check if IIS3DWB is reconnected
        if (c == 0x7B) {
          IIS3DWB_SETUP();
          Send_Garabage_Flag = false;
        }
      }
      Serial.write((uint8_t *)&(Gx), sizeof(Gx));  //Gx
      Serial.write((uint8_t *)&(Gy), sizeof(Gy));  //Gy
      Serial.write((uint8_t *)&(Gz), sizeof(Gz));  //Gz
    }

    // Dual Serial Mode for Resetting purpose
    SerialUSB1.write(0XBB); // For software to recognise COM port thread used for reset 
    if (SerialUSB1.available() > 0) {
      byte rxVal = SerialUSB1.read(); // Read Reset command sent by software

      if (rxVal == 0xAB) {  // Validation (0xAA)
        Serial2.write(0XAA); // Command to reset Teensy_1(Analog) 
        delay(1000); // Nescessary delay to wait for Teensy_1 ACK
        while (Serial2.available() > 0) {
          byte rxVal = Serial2.read(); 
          delay(100);

          if (rxVal == 0xBB) {  // Validation of reset ACK of Teensy_1(0xAA)
            SerialUSB1.write(0XCC); // Send ACK to software before reset of teensy_2
            delay(1000);
            SCB_AIRCR = 0x05FA0004;  // Magic value to initiate a software reset
          }
        }
      }
    }
  }
}

void calibrate_MPU(float gyro_bias[]) {  //pass array by ref

  const int MPU9250_MINIMUM_SAMPLING_DELAY_uSEC = 50;  // Delay of 50us between sample taken to calibrate 

  for (int i = 0; i < 3; i++) {
    gyro_bias[i] = 0.0; // Initialize of bias to 0
  }

  for (int i = 0; i < NUM_OF_CALIBRATION_SAMPLES; i++) {
    xyzFloat gyr = myMPU9250.getGyrValues();
    gyro_bias[0] = gyro_bias[0] + (gyr.x);
    gyro_bias[1] = gyro_bias[1] + (gyr.y);
    gyro_bias[2] = gyro_bias[2] + (gyr.z);
    delayMicroseconds(MPU9250_MINIMUM_SAMPLING_DELAY_uSEC);  //delay because max sampling rate of accelerometer is 4 khz
  }

  for (int i = 0; i < 3; i++) {
    gyro_bias[i] = gyro_bias[i] / NUM_OF_CALIBRATION_SAMPLES; // Actual Bias (Avg by 5000)
  }
}
