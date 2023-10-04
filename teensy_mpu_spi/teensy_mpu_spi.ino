#include <SPI.h>
#include <MPU9250_WE.h>

///////////// MPU Setup ///////////////
// SPI
#define MPU_CS_PIN 5 // Define the chip select pin for the MPU9250

const int8_t SYNC_BYTE = 0xAA;
int16_t Ax = 0;
int16_t Ay = 0;
int16_t Az = 0;

int16_t Gx = 0;
int16_t Gy = 0;
int16_t Gz = 0;

float Acc_bias[3];
float Gyro_bias[3];

long timer1, timer2;
const int16_t gyr_factor = 10;
const int16_t acc_factor = 1000;
const int16_t NUM_OF_CALIBRATION_SAMPLES = 5000;

MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);

/////////////// Function Prototypes //////////////
void calibrate_MPU(float acc_bias[], float gyro_bias[]);

void setup() {
  // Serial.begin(1500000);
  Serial.begin(115200);
  // Initialize SPI communication
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); // Configure SPI settings
  Serial.println("i am here");
  // // Initialize chip select pin
  pinMode(MPU_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH); // Deselect the MPU9250
  Serial.println("i am here 1");
  delay(1000);

  myMPU9250.init();
  delay(1000);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);
  calibrate_MPU(Acc_bias, Gyro_bias);
    Serial.println("i am here 2");
}

void loop() {
    Serial.println("i am here 3");
  timer1 = micros();
  for (uint32_t i = 0; i < 100000; i=i+1)
  {
  xyzFloat gValue = myMPU9250.getGValues();
  // xyzFloat gyr = myMPU9250.getGyrValues();
    Serial.println("i am here 4");
  Ax = (gValue.x - Acc_bias[0]) * acc_factor;
  Ay = (gValue.y - Acc_bias[1]) * acc_factor;
  Az = (gValue.z - Acc_bias[2]) * acc_factor;

  // Gx = (gyr.x - Gyro_bias[0]) * gyr_factor;
  // Gy = (gyr.y - Gyro_bias[1]) * gyr_factor;
  // Gz = (gyr.z - Gyro_bias[2]) * gyr_factor;

  // Serial.println("Acceleration (x,y,z): " + String(Ax) + " " + String(Ay) + " " + String(Az));
  // Serial.println("Gyro (x,y,z): " + String(Gx) + " " + String(Gy) + " " + String(Gz));
  // Serial.println();

    Serial.write(SYNC_BYTE); // Send the start/sync byte

    Serial.write((uint8_t*) & (Ax), sizeof(Ax));
    Serial.write((uint8_t*) & (Ay), sizeof(Ay));
    Serial.write((uint8_t*) & (Az), sizeof(Az));

    // Serial.write((uint8_t*) & (Gx), sizeof(Gx));
    // Serial.write((uint8_t*) & (Gy), sizeof(Gy));
    // Serial.write((uint8_t*) & (Gz), sizeof(Gz));

  }
  timer2 = micros();
  delay(5000);
  long diff = timer2 - timer1;
 Serial.write(SYNC_BYTE);
 Serial.write((uint8_t*) & (diff), sizeof(diff));
 while(1);
}

void calibrate_MPU(float acc_bias[], float gyro_bias[]) {
  const int MPU9250_MINIMUM_SAMPLING_DELAY_uSEC = 125;  //freq 8kHz

  for (int i = 0; i < 3; i++) {
    acc_bias[i] = 0.0;
    gyro_bias[i] = 0.0;
  }

  for (int i = 0; i < NUM_OF_CALIBRATION_SAMPLES; i++) {
    xyzFloat gValue = myMPU9250.getGValues();
    xyzFloat gyr = myMPU9250.getGyrValues();
    acc_bias[0] = acc_bias[0] + (gValue.x);
    acc_bias[1] = acc_bias[1] + (gValue.y);
    acc_bias[2] = acc_bias[2] + (gValue.z);
    gyro_bias[0] = gyro_bias[0] + (gyr.x);
    gyro_bias[1] = gyro_bias[1] + (gyr.y);
    gyro_bias[2] = gyro_bias[2] + (gyr.z);
    delayMicroseconds(MPU9250_MINIMUM_SAMPLING_DELAY_uSEC); //delay because max sampling rate of accelerometer is 4 khz

  }

  for (int i = 0; i < 3; i++) {
    acc_bias[i] = acc_bias[i] / NUM_OF_CALIBRATION_SAMPLES;
    gyro_bias[i] = gyro_bias[i] / NUM_OF_CALIBRATION_SAMPLES;
  }
  acc_bias[2] = acc_bias[2] - 1; //z axis
}
