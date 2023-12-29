/*
 * Copyright (c) 2020 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 * 
 * The IIS3DWB is a system-in-package featuring a 3-axis digital vibration sensor with low noise over an ultra-wide and 
 * flat frequency range. The wide bandwidth, low noise, very stable and repeatable sensitivity, together with the capability 
 * of operating over an extended temperature range (up to +105 °C), make the device particularly suitable for vibration 
 * monitoring in industrial applications.
 * 
 * Uses the SPI interface for fast data transfer.
 * 
 * Intended to be run on a Dragonfly development board:
 * 
 * https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/
 *  
 * 
 * Library may be used freely and without limit with attribution.
 * Github : https://github.com/kriswiner/IIS3DWB/tree/master/IIS3DWB_Basic_Dragonfly
 */

#include "IIS3DWB.h"
#include <SPI.h>

#define IIS3DWB_SPI_MODE SPI_MODE0
#define IIS3DWB_SPI_CLOCK 10000000

IIS3DWB::IIS3DWB(uint8_t cspin)
  : _cs(cspin)
{ }


uint8_t IIS3DWB::getChipID()
{
  uint8_t temp = readByte(IIS3DWB_WHO_AM_I);
  return temp;
}


void IIS3DWB::sleep()
{
  uint8_t temp = readByte(IIS3DWB_CTRL1_XL);
  writeByte(IIS3DWB_CTRL1_XL, temp & ~(0xA0));  // clear top three bits
}


void IIS3DWB::wake()
{
  uint8_t temp = readByte(IIS3DWB_CTRL1_XL);
  writeByte(IIS3DWB_CTRL1_XL, temp | 0xA0 );  // enable accelerometer
}


float IIS3DWB::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
    default :
      return 0;
  }
}


void IIS3DWB::reset()
{
  writeByte(IIS3DWB_CTRL1_XL, 0x00); // set accel to power down mode
  uint8_t temp = readByte(IIS3DWB_CTRL3_C);
  writeByte(IIS3DWB_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset IIS3DWB
  delay(1); // Wait for all registers to reset
}


uint8_t IIS3DWB::DRstatus()
{
  uint8_t temp = readByte(IIS3DWB_STATUS_REG); // read status register
  return temp;
}


uint8_t IIS3DWB::ACTstatus()
{
  uint8_t temp = readByte(IIS3DWB_ALL_INT_SRC); // read status register
  return temp;
}

 
void IIS3DWB::init(uint8_t Ascale)
{
  //writeByte(IIS3DWB_INT1_CTRL, 0x01);          // enable data ready interrupt on INT1
  writeByte(IIS3DWB_COUNTER_BDR_REG1, 0x80);   // enable pulsed (not latched) data ready interrupt  

  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  writeByte(IIS3DWB_CTRL3_C,0x04);
  // by default, interrupts active HIGH, push pull 
  // (can be changed by writing to bits 5 and 4, resp to above register)

  //  mask data ready until filter settle complete (bit 3 == 1), disable I2C (bit 2 == 1)
  writeByte(IIS3DWB_CTRL4_C, 0x08 | 0x04);

  writeByte(IIS3DWB_CTRL1_XL, 0xA0 | Ascale << 2); // set accel full scale and enable accel

  // High pass filter selection, comment out for full 6.3 kHz bandwidth
  // Set HPF to ODR/800 (bits 5 - 7 == 1), set HP Ref Mode (bit 4 == 1)
  // Set HP fast settle mode (bit 3 == 1), set filter select to 1 (bit 2 == 1)
//  writeByte(IIS3DWB_CTRL8_XL, 0xFC);  
  writeByte(IIS3DWB_CTRL8_XL, 0x00);  // Don't set HP reference mode // changed ODR 6.3kHz

  // activity interrupt handling
  //writeByte(IIS3DWB_WAKE_UP_DUR, 0x08);        // set inactivity duration at 1 LSB = 512/26.667 kHz ODR (so about 0.15 seconds)
  //writeByte(IIS3DWB_WAKE_UP_THS, 0x02);        // set wake threshold to 62.5 mg at 4 G FS, 4G/2^6 = 0.0625G
  // (change SLOPE_EN to 0x00 to drive activity change to INT)
  //writeByte(IIS3DWB_SLOPE_EN, 0x20);           // drive activity status to interrupt      
  //writeByte(IIS3DWB_INTERRUPTS_EN, 0x80);      // enable wakeup and activity/inactivity logic
  //writeByte(IIS3DWB_MD2_CFG, 0x80);            // route activity change event to INT2
  //writeByte(IIS3DWB_INT1_CTRL, 0x01);            // enable accel data ready on INT1 pin
}


void IIS3DWB::initFIFO(uint16_t fifo_size, uint8_t fifo_mode)
{
// FIFO size if 9 bits here stored in a 16-bit uint16_t, maximum is 0x01FF = 511
  writeByte(IIS3DWB_FIFO_CTRL1, (fifo_size & 0x00FF));        // write lowest 8 bits
  writeByte(IIS3DWB_FIFO_CTRL2, 0x80 | (fifo_size & 0x0100) >> 8); // write highest of 9 bits to bit 0, enable stop on watermark (bit 7 == 1)
  writeByte(IIS3DWB_FIFO_CTRL3, 0x0A);                       // write to FIFO at 26667 Hz
// time stamp (bits 6-7 == 0) and temperature (bits 4 - 5 == 0) not stored in FIFO 
  writeByte(IIS3DWB_FIFO_CTRL4, fifo_mode);                   // select FIFO mode
  writeByte(IIS3DWB_INT1_CTRL, 0x08);                         // enable FIFO threshold interrupt on INT1
}


uint16_t IIS3DWB::FIFOstatus()
{
  uint8_t temp1 = readByte(IIS3DWB_FIFO_STATUS1); // read FIFO status 1 register
  uint8_t temp2 = readByte(IIS3DWB_FIFO_STATUS2); // read FIFO status 2 register
  if(temp2 & 0x80) // if watermark full bit asserted, then FIFO ready to be read
  {
    uint16_t FIFOcount = (uint16_t) ((temp2 & 0x03) << 8) | temp1; // overwrite count with FIFO size if watermark interrupt asserted
    return FIFOcount;
  }
  return 0;
}


void IIS3DWB::selfTest()
{
  int16_t temp[3] = {0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0};

  writeByte(IIS3DWB_CTRL1_XL, 0xA0 | AFS_4G << 2); // set accel full scale and enable accel
  writeByte(IIS3DWB_CTRL3_C, 0x40 | 0x04);         // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  delay(100); // wait 100 ms for stable function

  readAccelData(temp); // discard first sample
  delay(100);
  readAccelData(temp); // should average these five times, but once is good enough
  accelNom[0] = temp[0];
  accelNom[1] = temp[1];
  accelNom[2] = temp[2];

  writeByte(IIS3DWB_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readAccelData(temp); // discard first sample
  delay(100);
  readAccelData(temp);
  accelPTest[0] = temp[0];
  accelPTest[1] = temp[1];
  accelPTest[2] = temp[2];

  writeByte(IIS3DWB_CTRL5_C, 0x02); // negative accel self test
  delay(100); // let accel respond
  readAccelData(temp); // discard first sample
  delay(100);
  readAccelData(temp);
  accelNTest[0] = temp[0];
  accelNTest[1] = temp[1];
  accelNTest[2] = temp[1];

  writeByte(IIS3DWB_CTRL5_C, 0x00); // normal mode
  writeByte(IIS3DWB_CTRL1_XL, 0x00); // power down accel and report results  

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");
  Serial.println(" ");
  delay(2000);
}


void IIS3DWB::offsetBias(float * destination)
{
  int16_t temp[3] = {0, 0, 0};
  int32_t sum[3] = {0, 0, 0};

//  Serial.println("Calculate accel offset biases: keep sensor flat and motionless!");
//  delay(10000);
  const int NSamples = 128; //5000 
  for (uint8_t ii = 0; ii < NSamples; ii++)
  {
    readAccelData(temp);
    sum[0] += temp[0];
    sum[1] += temp[1];
    sum[2] += temp[2];
    delay(1);
  }

  destination[0] = sum[0] * _aRes / float(NSamples);
  destination[1] = sum[1] * _aRes / float(NSamples);
  destination[2] = sum[2] * _aRes / float(NSamples);

  if (destination[0] > 0.75f)  {
    destination[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (destination[0] < -0.75f) {
    destination[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (destination[1] > 0.75f)  {
    destination[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (destination[1] < -0.75f) {
    destination[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (destination[2] > 0.75f)  {
    destination[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (destination[2] < -0.75f) {
    destination[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
}


void IIS3DWB::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(IIS3DWB_OUTX_L_XL, 6, &rawData[0]);  // Read the 6 raw accel data registers into data array
  destination[0] = (int16_t)(((int16_t)rawData[1] << 8)  | rawData[0]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[3] << 8)  | rawData[2]) ;
  destination[2] = (int16_t)(((int16_t)rawData[5] << 8)  | rawData[4]) ;
}


int16_t IIS3DWB::readTempData()
{
  uint8_t rawData[2];  // x/y/z accel register data stored here
  readBytes(IIS3DWB_OUT_TEMP_L, 2, &rawData[0]);  // Read the 2 raw temperature data registers into data array
  int16_t temp = (int16_t)((int16_t)rawData[1] << 8)  | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  return temp;
}


__attribute__((optimize("O3"))) void IIS3DWB::readFIFOData(uint16_t fifo_count, int16_t * xyzData)
{
  static uint8_t rawData[1 + 512*7]; // maximum FIFO size
  rawData[0] = IIS3DWB_FIFO_DATA_OUT_TAG | 0x80;
  SPI.beginTransaction(SPISettings(IIS3DWB_SPI_CLOCK, MSBFIRST, IIS3DWB_SPI_MODE));
  digitalWrite(_cs, LOW);
  SPI.transfer(rawData, (1 + (fifo_count * 7)));
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  for(uint32_t ii = 1; ii < (uint32_t)(1 + (fifo_count * 7)); ii += 7)
  {
  *xyzData++ = (rawData[ii+2] << 8) | rawData[ii+1];
  *xyzData++ = (rawData[ii+4] << 8) | rawData[ii+3];
  *xyzData++ = (rawData[ii+6] << 8) | rawData[ii+5];
  }
}


uint8_t IIS3DWB::readByte(uint8_t reg) 
{
  SPI.beginTransaction(SPISettings(IIS3DWB_SPI_CLOCK, MSBFIRST, IIS3DWB_SPI_MODE));
  digitalWrite(_cs, LOW); 
  SPI.transfer((reg & 0x7F) | 0x80);
  
  uint8_t temp = SPI.transfer(0);
  
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  return temp;
}


void IIS3DWB::readBytes(uint8_t reg, uint8_t count, uint8_t * dest) 
{
  SPI.beginTransaction(SPISettings(IIS3DWB_SPI_CLOCK, MSBFIRST, IIS3DWB_SPI_MODE));
  digitalWrite(_cs, LOW);
  SPI.transfer((reg & 0x7F) | 0x80);

  SPI.transfer(dest, count);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}


void IIS3DWB::writeByte(uint8_t reg, uint8_t value) 
{
  SPI.beginTransaction(SPISettings(IIS3DWB_SPI_CLOCK, MSBFIRST, IIS3DWB_SPI_MODE));
  digitalWrite(_cs, LOW); 
  SPI.transfer(reg & 0x7F);

  SPI.transfer(value);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}
