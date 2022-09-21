/* 11/11/2020 deknyff@lirmm.fr */

#include "LSM303AGR_Mag.h"
#include "I2CDev.h"

LSM303AGR_Mag::LSM303AGR_Mag(I2Cdev* i2c_bus)
{
 _i2c_bus = i2c_bus;   
}


uint8_t LSM303AGR_Mag::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_WHO_AM_I_REG);
  return c;
}


uint8_t LSM303AGR_Mag::getStatus()
{
  uint8_t c = _i2c_bus->readByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_STATUS_REG);
  return c;
}

void LSM303AGR_Mag::reset()
{
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_A, 0x03);  // idle
}


void LSM303AGR_Mag::init(uint8_t MODR)
{
  uint8_t MODR_mag;
  
     switch (MODR) 
     {
      case 0x02:
           MODR_mag = 0x00; //10Hz
           break;
      case 0x04:
           MODR_mag = 0x02; //50Hz
           break;
      case 0x05:
           MODR_mag = 0x03; //100Hz
           break;
      default:
           MODR_mag = 0x00; //10Hz
           break;     
     }    

  /*_i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_A, AODR_mag << 2 | 0x80);    // temp comp, high res, AODR, continuous
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_B, 0x02);				            // offset cancel enable
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_C, 0x10);                    // BDU */

   // enable temperature compensation (bit 7 == 1), continuous mode (bits 0:01 == 00)
 // for low power mode, set bit 4 to 1
 //_i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_A, 0x80 | AODR_mag <<2);  // high-resolution mode
 _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_A,  MODR_mag <<2 | 0x10);  // low power mode

 // enable low pass filter (bit 0 == 1), set to ODR/4, enable offset-cancellation bit 1
 _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_B, 0x00);  //0x02

 // enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1)
 _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_C, 0x01 | 0x10); 
}


void LSM303AGR_Mag::offsetBias(float * dest1, float * dest2)
{  
  
 int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  float _mRes = 0.0015f;
  
  Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!");
  delay(4000);

  for (int ii = 0; ii < 5000; ii++)
  {
    readMagData(mag_temp);
       for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(12);
  }

  _mRes = 0.0015f; // fixed sensitivity
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0] * _mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * _mRes;   
    dest1[2] = (float) mag_bias[2] * _mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

   Serial.print("dest2 0 : "); Serial.println(dest2[0]);
   Serial.print("dest2 1 : "); Serial.println(dest2[1]);
   Serial.print("dest2 2 : "); Serial.println(dest2[2]);
   Serial.println("Mag Calibration done!");
}


void LSM303AGR_Mag::selfTest()
{
  uint8_t i;
  int16_t temp[3] = {0, 0, 0};  
  int16_t temp_norm[3] = {0, 0, 0};
  int16_t temp_test[3] = {0, 0, 0};
  int32_t temp_cumul[3] = {0, 0, 0};
  
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_A, 0x8C);
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_B, 0x02); 
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_C, 0x10);
  delay(100);  

  while (!((getStatus() >> 3) & 0x01));  // attente data mag
  readMagData(temp);  //discard

  for (i = 0; i < 50; i++)
  {
    while (!((getStatus() >> 3) & 0x01));  // attente data mag   
    readMagData(temp);
    temp_cumul[0] +=  (int32_t)temp[0];
    temp_cumul[1] +=  (int32_t)temp[1];
    temp_cumul[2] +=  (int32_t)temp[2];
  }

  temp_norm[0] = (int16_t)(temp_cumul[0] / 50);
  temp_norm[1] = (int16_t)(temp_cumul[1] / 50);
  temp_norm[2] = (int16_t)(temp_cumul[2] / 50);

  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_C, 0x12);  //self test
  delay(100);  
  
  while (!((getStatus() >> 3) & 0x01));  // attente data mag
  readMagData(temp);  //discard

  temp_cumul[0] = 0;
  temp_cumul[1] = 0;
  temp_cumul[2] = 0;
  
  for (i = 0; i < 50; i++)
  {
    while (!((getStatus() >> 3) & 0x01));  // attente data mag   
    readMagData(temp);
    temp_cumul[0] +=  (int32_t)temp[0];
    temp_cumul[1] +=  (int32_t)temp[1];
    temp_cumul[2] +=  (int32_t)temp[2];
  }

  temp_test[0] = (int16_t)(temp_cumul[0] / 50);
  temp_test[1] = (int16_t)(temp_cumul[1] / 50);
  temp_test[2] = (int16_t)(temp_cumul[2] / 50);  

  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_C, 0x10);  //disable self test
  _i2c_bus->writeByte(LSM303AGR_Mag_ADDRESS, LSM303AGR_MAG_CFG_REG_A, 0x83);  //idle

  Serial.println("\r\nMag Self Test:");
  Serial.print("Mx results: "); Serial.print((temp_test[0] - temp_norm[0]) * 1.5f); Serial.println(" mG");
  Serial.print("My results: "); Serial.println((temp_test[1] - temp_norm[1]) * 1.5f);
  Serial.print("Mz results: "); Serial.println((temp_test[2] - temp_norm[2]) * 1.5f);
  Serial.println("Absolute values should be less that 500 mG");

  delay(1000);  
}


void LSM303AGR_Mag::readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z mag register data stored here
  _i2c_bus->readBytes(LSM303AGR_Mag_ADDRESS, 0x80 | LSM303AGR_MAG_OUTX_L_REG, 6, &rawData[0]);  // Read the 6 raw data registers into data array
  //hi-res mode
  destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value in normal mode
  destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 4;  
  destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 4; 
}
