#include <ADC.h>
#include <ADC_util.h>  //Necessary header files for teensy's ADC

const int8_t SYNC_BYTE = 0xAA;    //Initial sync byte to detect start of data
const int8_t SYNC_BYTE_1 = 0xAB;  //Another sync byte to avoid any overlapping of data and sync byte

uint16_t raw0, raw1, raw2, raw3, raw4, raw5, raw6, raw7, raw8, raw9, raw17, diff;  //defining all variables
unsigned long lastMicros_1;

ADC *adc = new ADC();  // adc object;

void setup() {
  Serial.begin(1500000);  //serial port to get connected to PC at 15,00,000 baud rate
  Serial2.begin(115200);  //serial uart to communicate with Teensy_2 (Digital)

  adc->adc0->setAveraging(1);    // set number of averages
  adc->adc0->setResolution(16);  // set bits of resolution

  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // change the sampling speed


  lastMicros_1 = 0;

  delay(5000);
}

void loop() {

  if (micros() >= lastMicros_1 + 20)  // micros() >= lastMicros + DESIRED_LOOP_DELAY_uSec       here it is 50 KHz
  {
    lastMicros_1 = micros();

    raw0 = adc->adc0->analogRead(A0);  //magnetic
    raw1 = adc->adc0->analogRead(A1);  //vt1
    raw2 = adc->adc0->analogRead(A2);  //vt2
    raw3 = adc->adc0->analogRead(A3);  //vt3
    raw4 = adc->adc0->analogRead(A4);  //ct1
    raw5 = adc->adc0->analogRead(A5);  //ct2
    raw6 = adc->adc0->analogRead(A6);  //ct3
    raw7 = adc->adc0->analogRead(A7);  //ct4
    raw8 = adc->adc0->analogRead(A8);  //pressure
    raw9 = adc->adc0->analogRead(A9);  //Waterflow
    // raw17 = adc->adc0->analogRead(A17);  //rpm


    Serial.write(SYNC_BYTE);
    Serial.write(SYNC_BYTE_1);
    Serial.write((uint8_t *)&(lastMicros_1), sizeof(lastMicros_1));  //Timestamp
    Serial.write((uint8_t *)&(raw0), sizeof(raw0));                  //magnetic
    Serial.write((uint8_t *)&(raw1), sizeof(raw1));                  //vt1
    Serial.write((uint8_t *)&(raw2), sizeof(raw2));                  //vt2
    Serial.write((uint8_t *)&(raw3), sizeof(raw3));                  //vt3
    Serial.write((uint8_t *)&(raw4), sizeof(raw4));                  //ct1
    Serial.write((uint8_t *)&(raw5), sizeof(raw5));                  //ct2
    Serial.write((uint8_t *)&(raw6), sizeof(raw6));                  //ct3
    // Serial.write((uint8_t *)&(raw7), sizeof(raw7));                  //ct4
    // Serial.write((uint8_t *)&(raw8), sizeof(raw8));                  //pressure
    // Serial.write((uint8_t *)&(raw9), sizeof(raw9));                  //waterflow
    // Serial.write((uint8_t *)&(raw17), sizeof(raw17));             //rpm
  }


  if (Serial2.available() > 0) { // Checking for Reset command from Teensy_2(Digital)
    byte rxVal = Serial2.read();
    delay(300);
    if (rxVal == 0xAA) {  // Validation of Command received (0xAA)
      Serial2.write(0XBB); // Sending ACK to Teensy_2
      delay(1000);
      SCB_AIRCR = 0x05FA0004;  // Magic value to initiate a software reset
    }
  }
  
}
