

#include <ADC.h>
#include <ADC_util.h>

long timer1, timer2;
const int8_t SYNC_BYTE = 0xAA;
char text[10];
uint16_t raw = 1234;
uint16_t raw1, raw2, raw3, raw4, raw5, raw6, raw7, raw8, raw9, raw10;

ADC *adc = new ADC(); // adc object;
void setup() {
  // put your setup code here, to run once:
//  Serial.begin(1500000);
 Serial.begin(2000000);
//  CCM_CSCDR1=105450240; 
    adc->adc0->setAveraging(1); // set number of averages
    adc->adc0->setResolution(12); // set bits of resolution

    // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
    // see the documentation for more information
    // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
    // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
    // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
    
    adc->adc0->enableCompare(1.0/3.3*adc->adc0->getMaxValue(), 0); // measurement will be ready if value < 1.0V
    adc->adc0->enableCompareRange(1.0*adc->adc0->getMaxValue()/3.3, 2.0*adc->adc0->getMaxValue()/3.3, 0, 1); // ready if value lies out of [1.0,2.0] V


    
    ////// ADC1 /////
    // #ifdef ADC_DUAL_ADCS
    adc->adc1->setAveraging(1); // set number of averages
    adc->adc1->setResolution(16); // set bits of resolution
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

    //adc->adc1->setReference(ADC_REFERENCE::REF_1V2);

    // always call the compare functions after changing the resolution!
    adc->adc1->enableCompare(1.0/3.3*adc->adc1->getMaxValue(), 0); // measurement will be ready if value < 1.0V
    adc->adc1->enableCompareRange(1.0*adc->adc1->getMaxValue()/3.3, 2.0*adc->adc1->getMaxValue()/3.3, 0, 1); // ready if value lies out of [1.0,2.0] V


    // If you enable interrupts, note that the isr will read the result, so that isComplete() will return false (most of the time)
    //adc->adc1->enableInterrupts(adc1_isr);

    // #endif
  // Serial.begin(115200);
    delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // timer1 = micros();
  // for (uint32_t i = 0; i < 1000; i=i+1)
  // {
    raw1 = adc->adc0->analogRead(A9);
    // raw2 = adc->adc1->analogRead(A2);
    // raw3 = adc->adc0->analogRead(A3);
    // raw4 = adc->adc0->analogRead(A4);
    // raw5 = adc->adc0->analogRead(A5);
    // raw6 = adc->adc0->analogRead(A6);
    // raw7 = adc->adc0->analogRead(A7);
    // raw8 = adc->adc0->analogRead(A8);
    // raw9 = adc->adc0->analogRead(A9);
    // raw10 = adc->adc0->analogRead(A10);

   Serial.write(SYNC_BYTE);
   Serial.write((uint8_t*) & (raw1), sizeof(raw1));
   
   
    // #ifdef ADC_DUAL_ADCS
    // raw2 = adc->adc1->analogRead(A2);
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw2), sizeof(raw2));

  //  #endif
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw3), sizeof(raw3));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw4), sizeof(raw4));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw5), sizeof(raw5));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw6), sizeof(raw6));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw7), sizeof(raw7));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw8), sizeof(raw8));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw9), sizeof(raw9));
  //  Serial.write(SYNC_BYTE);
  //  Serial.write((uint8_t*) & (raw10), sizeof(raw10));
  //  raw=raw+1;
    // snprintf(text, sizeof(text), "%d", raw);
//     Serial.println(text);
//   }
//   timer2 = micros();
//   delay(5000);
//   long diff = timer2 - timer1;
//  Serial.write(SYNC_BYTE);
//  Serial.write((uint8_t*) & (diff), sizeof(diff));

  // snprintf(text, sizeof(text), "%d", diff);
  // Serial.println(text);
//  snprintf(text, sizeof(text), "%d", diff);
//  Serial.println(text);
  // while(1);
}
