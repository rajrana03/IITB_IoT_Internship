#include <ADC.h>
#include <ADC_util.h>
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

// const int8_t SYNC_BYTE = 0xAA;
uint16_t raw1, raw2, raw3, raw4, raw5, raw6, raw7, raw8, raw9, raw10, diff;
unsigned long lastMicros_1, lastMicros_2, lastMicros_3, timer1, timer2;

#define reset 2
unsigned long startTime = 0;  // Variable to store the start time
boolean pinWasHigh = false;   // Flag to track if the pin was high

int pinState;

int c = 0;

ADC *adc = new ADC();  // adc object;

void check_for_reset() {
  pinState = digitalRead(reset);

  // If the pin is high, record the start time
  if (pinState == HIGH) {
    if (!pinWasHigh) {
      startTime = millis();
      pinWasHigh = true;
    }
  } else {
    pinWasHigh = false;
  }

  if (pinState == HIGH) {
    // Check if the pin has been high for 1 second
    if (millis() - startTime >= 1000) {
      SCB_AIRCR = 0x05FA0004;  // Magic value to initiate a software reset
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1500000);
  Serial1.begin(1500000);
  //  Serial.begin(2000000);
  //  CCM_CSCDR1=105450240;
  adc->adc0->setAveraging(1);    // set number of averages
  adc->adc0->setResolution(16);  // set bits of resolution

  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // change the sampling speed

  // adc->adc0->enableCompare(1.0 / 3.3 * adc->adc0->getMaxValue(), 0);                                                // measurement will be ready if value < 1.0V
  // adc->adc0->enableCompareRange(1.0 * adc->adc0->getMaxValue() / 3.3, 2.0 * adc->adc0->getMaxValue() / 3.3, 0, 1);  // ready if value lies out of [1.0,2.0] V
  lastMicros_1 = 0;
  lastMicros_2 = 0;
  lastMicros_3 = 0;
  timer1 = 0;
  timer2 = 0;

  
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

  pinMode(reset, INPUT);
  digitalWrite(reset, LOW);
  delay(5000);
  timer1 = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  if (micros() >= lastMicros_1 + 20)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec
  {
    lastMicros_1 = micros();

    // raw1 = adc->adc0->analogRead(A1);    //vt1
    // raw2 = adc->adc0->analogRead(A2);    //vt2
    // raw3 = adc->adc0->analogRead(A3);    //vt3
    raw4 = adc->adc0->analogRead(A0);    //ct1
    raw5 = adc->adc0->analogRead(A8);    //ct2
    // raw6 = adc->adc0->analogRead(A9);    //ct3
    // raw7 = adc->adc0->analogRead(A7);    //ct4


    Serial.write(SYNC_BYTE);
    Serial.write((uint8_t *)&(raw1), sizeof(raw1));
    Serial.write((uint8_t *)&(raw2), sizeof(raw2));
    // Serial.write((uint8_t *)&(raw3), sizeof(raw3));
    // Serial.write((uint8_t *)&(raw4), sizeof(raw4));
    // Serial.write((uint8_t *)&(raw5), sizeof(raw5));
    // Serial.write((uint8_t *)&(raw6), sizeof(raw6));
    // Serial.write((uint8_t *)&(raw7), sizeof(raw7));
  }
  if (micros() >= lastMicros_2 + 1000 - 20)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec
  {
    lastMicros_2 = micros();

    raw7 = adc->adc0->analogRead(A7);    //ct4
  //   raw8 = adc->adc0->analogRead(A8);    //pressure
  //   c++;
  //   Serial1.write(SYNC_BYTE);
  //   Serial1.write((uint8_t *)&(raw7), sizeof(raw7));
  //   Serial1.write((uint8_t *)&(raw8), sizeof(raw8));
  //   // Serial1.println(raw7);
  //   // Serial1.println(raw8);
  }

  if (micros() >= lastMicros_3 + 100)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec
  {
    lastMicros_3 = micros();

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

    // Serial.write(SYNC_BYTE);  // Send the start/sync byte

    // Serial.write((uint8_t*)&(Ax), sizeof(Ax));
    // Serial.write((uint8_t*)&(Ay), sizeof(Ay));
    // Serial.write((uint8_t*)&(Az), sizeof(Az));

    // Serial.write((uint8_t*)&(Gx), sizeof(Gx));
    // Serial.write((uint8_t*)&(Gy), sizeof(Gy));
    // Serial.write((uint8_t*)&(Gz), sizeof(Gz));
  }
  // check_for_reset();
  // if(c>=10000)
  // {
  //   timer2 = micros();
  //   diff = timer2 - timer1;
  //   Serial1.println(diff);
  //   // Serial1.write(SYNC_BYTE);
  //   // Serial1.write((uint8_t *)&(diff), sizeof(diff));
  //   // Serial1.write((uint8_t *)&(diff), sizeof(diff));
  //   while(1);
  // }
}
