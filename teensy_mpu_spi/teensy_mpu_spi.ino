// #include "esp_task_wdt.h"
// #include <MPU9250_WE.h>
// #include <SPI.h>

// /////////////MPU Setup/////////////
// //VSPI
// #define MPU_CS_PIN 5

// const int8_t SYNC_BYTE = 0xAA;
// int16_t Ax{ 0 };
// int16_t Ay{ 0 };
// int16_t Az{ 0 };

// int16_t Gx{ 0 };
// int16_t Gy{ 0 };
// int16_t Gz{ 0 };

// //long timer1, timer2;
// const int16_t gyr_factor = 10;
// const int16_t acc_factor = 1000;

// ///////////////CONSTRUCTORS//////////////
// MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);  //VSPI
// /////////////FREE-RTOS Setup/////////////
// TaskHandle_t MPU_GET_h = NULL;

// void MPU_GET(void* parameter) {
//   esp_task_wdt_delete(NULL);

//   // vTaskSuspend(NULL);

//   myMPU9250.init();
//   delay(1000);
//   myMPU9250.autoOffsets();
//   myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);  // bandwdith without DLPF
//   // myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
//   myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
//   myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
//   myMPU9250.enableAccDLPF(false);
//   myMPU9250.setAccDLPF(MPU9250_DLPF_6);
//   myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
//   delay(1000);

//   for (;;) {

//     esp_task_wdt_init(10, false);
//     //    timer1 = micros();
//     xyzFloat gValue = myMPU9250.getGValues();
//     xyzFloat gyr = myMPU9250.getGyrValues();

//     Ax = gValue.x * acc_factor;
//     Ay = gValue.y * acc_factor;
//     Az = gValue.z * acc_factor;

//     Gx = gyr.x * gyr_factor;
//     Gy = gyr.y * gyr_factor;
//     Gz = gyr.z * gyr_factor;

//     //  timer2 = micros();
//     //  Serial.println(timer2 - timer1);

//     Serial.write(SYNC_BYTE);  // Send the start/sync byte

//     Serial.write((uint8_t*)&(Ax), sizeof(Ax));
//     Serial.write((uint8_t*)&(Ay), sizeof(Ay));
//     Serial.write((uint8_t*)&(Az), sizeof(Az));

//     Serial.write((uint8_t*)&(Gx), sizeof(Gx));
//     Serial.write((uint8_t*)&(Gy), sizeof(Gy));
//     Serial.write((uint8_t*)&(Gz), sizeof(Gz));
//   }
// }
// void setup() {
//   Serial.begin(1500000);
//   //  Serial.begin(115200);

//   xTaskCreatePinnedToCore(
//     MPU_GET,    //TASK
//     "MPU_GET",  //Task id
//     8000,       //Stack Size
//     NULL,
//     1,
//     &MPU_GET_h,  //TASK HANDLE
//     0);
// }

// void loop() {
//   for (;;) {
//     delay(1000);
//   }
// }



// #include "esp_task_wdt.h"
#include <MPU9250_WE.h>
#include <SPI.h>

/////////////MPU Setup/////////////
//VSPI
#define MPU_CS_PIN 10

const int8_t SYNC_BYTE = 0xAA;
int16_t Ax{ 0 };
int16_t Ay{ 0 };
int16_t Az{ 0 };

int16_t Gx{ 0 };
int16_t Gy{ 0 };
int16_t Gz{ 0 };

//long timer1, timer2;

///////////////CONSTRUCTORS//////////////
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);  //VSPI

void setup() {
  Serial.begin(1500000);
  //  Serial.begin(115200);
  // esp_task_wdt_delete(NULL);

  // vTaskSuspend(NULL);

  myMPU9250.init();
  delay(1000);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);  // bandwdith without DLPF
  // myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);
}

void loop() {
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  for (;;) {
    // esp_task_wdt_init(10, false);
    //    timer1 = micros();

    gValue = myMPU9250.getGValues();
    gyr = myMPU9250.getGyrValues();
    Ax = gValue.x * 1000;
    Ay = gValue.y * 1000;
    Az = gValue.z * 1000;

    Gx = gyr.x * 100;
    Gy = gyr.y * 100;
    Gz = gyr.z * 100;

    //  timer2 = micros();
    //  Serial.println(timer2 - timer1);

    Serial.write(SYNC_BYTE);  // Send the start/sync byte

    Serial.write((uint8_t*)&(Ax), sizeof(Ax));
    Serial.write((uint8_t*)&(Ay), sizeof(Ay));
    Serial.write((uint8_t*)&(Az), sizeof(Az));

    Serial.write((uint8_t*)&(Gx), sizeof(Gx));
    Serial.write((uint8_t*)&(Gy), sizeof(Gy));
    Serial.write((uint8_t*)&(Gz), sizeof(Gz));

    // delay(1);
    // delayMicroseconds(1);
  }
}