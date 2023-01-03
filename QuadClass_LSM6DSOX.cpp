/*!
 *  @file QuadClass_LSM6DSOX.cpp
 *
 * 	Extended library for LSM6DSOX IMU Sensor
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit LSM6DS library
 *  Please review inherited classes Adafruit_LSM6DSOX.h and Adafruit_LSM6DS.h for more call methods and information.
 *
 *  UCSD
 *  Author: jgarzagu@eng.ucsd.edu
 *   
 *
 * ######################################################################################################## //
 *                                                                                                          //
 *                                 ~LSM6DSOX Filters structure and call methods~                            //
 *                                                                                                          //
 * Accelerometer Filters                                                                                    //
 * ---------------------                                                                                    //
 * [Analog AA LP Filter] -> [ADC] -> [Digital LP Filter (ODR)] -> [Composite Filter]         ->   Output    //
 *                                   setAccelDataRate()           setAccelCompositeFilter()                 //
 *                                   (Table 44.)                  (Table 65.)                               //
 *                                                                                                          //
 *                                                                                                          //  
 * Gyroscope Filters                                                                                        //
 * ---------------------                                                                                    //
 * [ADC]   =>  [HPF]         =>  [LPF1]         =>  [LPF2 (ODR)]       ->   Output                          //
 *             setGyroHPF()      setGyroLPF1()      setGyroDataRate()                                       //
 *             (Table 62.)       (Table 60.)        (Table 18.)                                             //
 *                                                                                                          //  
 * ######################################################################################################## //
 *
 */
#include "QuadClass_LSM6DSOX.h"


/*!
 *    @brief Extended library for LSM6DSOX IMU Sensor                                                                     
 */
QuadClass_LSM6DSOX::QuadClass_LSM6DSOX(void) {}

/*!
 *    @brief Class destructor                                                                     
 */
QuadClass_LSM6DSOX::~QuadClass_LSM6DSOX(void) {}

/*!
 *    @brief  Raw write to LSM6DSOX registers
 *    @param  reg
 *            LSM6DSOX register to write.
 *    @param  reg_bits
 *            Number of register bits to write (e.g. 3 register bits in case of HPCF_XL_[0,1,2] of register LSM6DS_CTRL8_XL).
 *    @param  bits_shift
 *            Location of the register bits starting from 0 (right) to left (7) (e.g., 5 bit shifts for HPCF_XL_[0,1,2] of register LSM6DS_CTRL8_XL).
 *    @param  data
 *            Data to write to the register bits (e.g., true or false in case of 1 register bit, 0-3 in case of 2 register bits).
 *    @return none
 */
void QuadClass_LSM6DSOX::write(uint16_t reg, uint8_t reg_bits, uint8_t bits_shift, uint32_t data) {

  if (i2c_dev == NULL) {
    Serial.println("Quad_LSM6DS write: Initialize I2C first");
    return;
  }

  Adafruit_BusIO_Register LSM6DSOX_register = Adafruit_BusIO_Register(
    i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, reg);

  Adafruit_BusIO_RegisterBits LSM6DSOX_register_bits =
    Adafruit_BusIO_RegisterBits(&LSM6DSOX_register, reg_bits, bits_shift);

  LSM6DSOX_register_bits.write(data);
}

/*!
 *    @brief  Raw read to LSM6DSOX registers
 *    @param  reg
 *            LSM6DSOX register to read.
 *    @return Register data in 8 bit format
 */
uint8_t QuadClass_LSM6DSOX::read(uint16_t reg) {

  if (i2c_dev == NULL) {
    Serial.println("Quad_LSM6DS read: Initialize I2C first");
    return 0;
  }

  Adafruit_BusIO_Register LSM6DSOX_register = Adafruit_BusIO_Register(
    i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, reg);

  return LSM6DSOX_register.read();
}

/*!
 *    @brief  Sets accelerometer composite filter parameters
 *    @param  filter
 *            Type of filter to enable: None, high-pass, or low-pass 2 filter.
 *    @param  filter_range
 *            High-pass or low-pass 2 filter range (Datasheet Table 65.)
 *    @return none
 */
void QuadClass_LSM6DSOX::setAccelCompositeFilter(accelCompositeFilter_t filter, accelCompositeFilter_range_t filter_range) {
  switch (filter) {
    case LSM6DS_CompositeFilter_NONE:             // No filter
      this->write(LSM6DS_CTRL1_XL, 1, 1, false);  // LPF2_XL_EN = 0
      this->write(LSM6DS_CTRL8_XL, 1, 2, false);  // HP_SLOPE_XL_EN = 0
      this->write(LSM6DS_CTRL8_XL, 3, 5, 0);      // HPCF_XL_[0,1,2] = 0
      break;
    case LSM6DS_CompositeFilter_LPF2:                    // Low pass filter 2
      this->write(LSM6DS_CTRL1_XL, 1, 1, true);          // LPF2_XL_EN = 1
      this->write(LSM6DS_CTRL8_XL, 1, 2, false);         // HP_SLOPE_XL_EN = 0
      this->write(LSM6DS_CTRL8_XL, 3, 5, filter_range);  // HPCF_XL_[0,1,2] = filter_range
      break;
    case LSM6DS_CompositeFilter_HPF:
      this->write(LSM6DS_CTRL1_XL, 1, 1, false);         // LPF2_XL_EN = 0
      this->write(LSM6DS_CTRL8_XL, 1, 2, true);          // HP_SLOPE_XL_EN = 1
      this->write(LSM6DS_CTRL8_XL, 3, 5, filter_range);  // HPCF_XL_[0,1,2] = filter_range
      break;
  }
}

/*!
 *    @brief  Sets gyroscope low-pass-1 filter parameters
 *    @param  enable
 *            Enable or dissable filter
 *    @param  filter_range
 *            Gyroscope low-pass-1 filter range (Datasheet Table 60.)
 *    @return none
 */
void QuadClass_LSM6DSOX::setGyroLPF1(bool enable, gyroLPF1_range_t filter_range) {
  switch (enable) {
    case true:
      this->write(LSM6DS_CTRL4_C, 1, 1, true);          // LPF1_SEL_G = 1
      this->write(LSM6DS_CTRL6_C, 3, 0, filter_range);  // FTYPE_[0,1,2] = filter_range
      break;
    case false:
      this->write(LSM6DS_CTRL4_C, 1, 1, false);  // LPF1_SEL_G = 0
      this->write(LSM6DS_CTRL6_C, 3, 0, 0);      // FTYPE_[0,1,2] = 0
      break;
  }
}

/*!
 *    @brief  Sets gyroscope high-pass filter parameters
 *    @param  enable
 *            Enable or dissable filter
 *    @param  filter_range
 *            Gyroscope high-pass filter range (Datasheet Table 62.)
 *    @return none
 */
void QuadClass_LSM6DSOX::setGyroHPF(bool enable, gyroHPF_range_t filter_range) {
  switch (enable) {
    case true:
      this->write(LSM6DS_CTRL7_G, 1, 6, true);          // HP_EN_G = 1
      this->write(LSM6DS_CTRL7_G, 2, 4, filter_range);  // HPM[0,1]_G = filter_range
      break;
    case false:
      this->write(LSM6DS_CTRL7_G, 1, 6, false);  // HP_EN_G = 0
      this->write(LSM6DS_CTRL7_G, 2, 4, 0);      // HPM[0,1]_G = 0
      break;
  }
}