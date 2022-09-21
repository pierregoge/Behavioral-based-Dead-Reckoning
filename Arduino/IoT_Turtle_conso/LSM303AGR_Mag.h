/* 11/11/2020 deknyff@lirmm.fr */

#ifndef LSM303AGR_Mag_h
#define LSM303AGR_Mag_h

#include "Arduino.h"
#include <Wire.h>
#include "I2CDev.h"

//Register map for LSM303AGR Accelero'
//https://www.st.com/content/ccc/resource/technical/document/datasheet/74/c4/19/54/62/c5/46/13/DM00177685.pdf/files/DM00177685.pdf/jcr:content/translations/en.DM00177685.pdf

#define LSM303AGR_MAG_OFFSET_X_REG_L  	0x45
#define LSM303AGR_MAG_OFFSET_X_REG_H  	0x46
#define LSM303AGR_MAG_OFFSET_Y_REG_L  	0x47
#define LSM303AGR_MAG_OFFSET_Y_REG_H  	0x48
#define LSM303AGR_MAG_OFFSET_Z_REG_L  	0x49
#define LSM303AGR_MAG_OFFSET_Z_REG_H  	0x4A
#define LSM303AGR_MAG_WHO_AM_I_REG  	0x4F	// should be 0x40
#define LSM303AGR_MAG_CFG_REG_A  		0x60
#define LSM303AGR_MAG_CFG_REG_B  		0x61
#define LSM303AGR_MAG_CFG_REG_C  		0x62
#define LSM303AGR_MAG_INT_CTRL_REG  	0x63
#define LSM303AGR_MAG_INT_SOURCE_REG  	0x64
#define LSM303AGR_MAG_INT_THS_L_REG  	0x65
#define LSM303AGR_MAG_INT_THS_H_REG  	0x66
#define LSM303AGR_MAG_STATUS_REG  		0x67
#define LSM303AGR_MAG_OUTX_L_REG  		0x68
#define LSM303AGR_MAG_OUTX_H_REG  		0x69
#define LSM303AGR_MAG_OUTY_L_REG  		0x6A
#define LSM303AGR_MAG_OUTY_H_REG  		0x6B
#define LSM303AGR_MAG_OUTZ_L_REG  		0x6C
#define LSM303AGR_MAG_OUTZ_H_REG  		0x6D

#define LSM303AGR_Mag_ADDRESS           0x1E  // Address of LSM303AGR Magneto

class LSM303AGR_Mag
{
  public:
  LSM303AGR_Mag(I2Cdev* i2c_bus);
  void reset();
  uint8_t getChipID();
  uint8_t getStatus();
  void init(uint8_t AODR);
  void selfTest();
  void offsetBias(float * dest1, float * dest2);
  void readMagData(int16_t * destination);
  private:
  I2Cdev* _i2c_bus;
};

#endif
