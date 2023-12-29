#include "esp_task_wdt.h"
#include <MPU9250_WE.h>
#include <SPI.h>
#include <SparkFun_KX13X.h>

/////////////MPU Setup/////////////
// VSPI for MPU9250
#define MPU_CS_PIN 5
SPIClass *mpuSpi = &SPI; // Use the default SPI bus

const int8_t SYNC_BYTE = 0xAA;
int16_t Ax{0};
int16_t Ay{0};
int16_t Az{0};

int16_t Gx{0};
int16_t Gy{0};
int16_t Gz{0};

///////////////CONSTRUCTORS//////////////
MPU9250_WE myMPU9250 = MPU9250_WE(mpuSpi, MPU_CS_PIN, true); // VSPI
SparkFun_KX132_SPI kxAccel;

rawOutputData myData;   // Struct for the accelerometer's data
byte chipSelect_KX = 15; // Change to fit your project.

void setup()
{
  Serial.begin(1500000);
  // Wait for the Serial monitor to be opened.
  while (!Serial)
    delay(50);

  esp_task_wdt_delete(NULL);

  // MPU9250 setup
  myMPU9250.init();
  delay(1000);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  delay(1000);

  // KX132 setup
  pinMode(chipSelect_KX, OUTPUT);
  SPIClass *kxSpi = new SPIClass(HSPI); // Use HSPI for KX132
  kxSpi->begin(14, 27, 13, 15);
  SPISettings kxSettings = SPISettings(10000000, MSBFIRST, SPI_MODE0);
  if (!kxAccel.begin(*kxSpi, kxSettings, chipSelect_KX))
  {
    Serial.println("Could not communicate with the KX13X. Freezing.");
    while (1)
      ;
  }
  delay(5);
  kxAccel.enableAccel(false);
  kxAccel.setRange(SFE_KX132_RANGE8G);
  kxAccel.enableDataEngine();
  kxAccel.setOutputDataRate(15);
  kxAccel.enableAccel();
}

void loop()
{
  xyzFloat accValue = myMPU9250.getAccRawValues();
  xyzFloat gyr = myMPU9250.getGyrRawValues();

  for (;;)
  {
    esp_task_wdt_init(10, false);

    // digitalWrite(chipSelect_KX, LOW);
    kxAccel.getRawAccelRegisterData(&myData);
    // digitalWrite(chipSelect_KX, HIGH);

    // digitalWrite(MPU_CS_PIN, LOW);
    gyr = myMPU9250.getGyrRawValues();
    // digitalWrite(MPU_CS_PIN, HIGH);

    // Gx = gyr.x;
    // Gy = gyr.y;
    // Gz = gyr.z;

    Serial.write(SYNC_BYTE);

    Serial.write((uint8_t *)&myData.xData, sizeof(myData.xData));
    Serial.write((uint8_t *)&myData.yData, sizeof(myData.yData));
    Serial.write((uint8_t *)&myData.zData, sizeof(myData.zData));

    Serial.write((uint8_t *)&Gx, sizeof(Gx));
    Serial.write((uint8_t *)&Gy, sizeof(Gy));
    Serial.write((uint8_t *)&Gz, sizeof(Gz));
  }
}
