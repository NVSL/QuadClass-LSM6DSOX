/*!
 *  @file QuadClass_LSM6DSOX.h
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
#ifndef _QUADCLASS_LSM6DSOX_H
#define _QUADCLASS_LSM6DSOX_H

#include <Adafruit_LSM6DSOX.h>

#define LSM6DS_CTRL4_C 0x13
#define LSM6DS_CTRL6_C 0x15
#define LSM6DS_CTRL7_G 0x16

/**  Accelerometer composite filter */
typedef enum accelCompositeFilter {
  LSM6DS_CompositeFilter_NONE,  // No filter
  LSM6DS_CompositeFilter_LPF2,  // Low pass filter 2
  LSM6DS_CompositeFilter_HPF,   // High Pass filter
} accelCompositeFilter_t;

/**  Accelerometer high pass or low pass 2 filter range (Table 65.)*/
typedef enum accelCompositeFilter_range {
  LSM6DS_CompositeFilter_ODR_DIV_4 = 0,
  LSM6DS_CompositeFilter_ODR_DIV_10 = 1,
  LSM6DS_CompositeFilter_ODR_DIV_20 = 2,
  LSM6DS_CompositeFilter_ODR_DIV_45 = 3,
  LSM6DS_CompositeFilter_ODR_DIV_100 = 4,
  LSM6DS_CompositeFilter_ODR_DIV_200 = 5,
  LSM6DS_CompositeFilter_ODR_DIV_400 = 6,
  LSM6DS_CompositeFilter_ODR_DIV_800 = 7,
} accelCompositeFilter_range_t;

/**  Gyro LPF1 range (Table 60.) */
typedef enum gyroLPF1_range {
  LSM6DS_gyroLPF1_0 = 0,
  LSM6DS_gyroLPF1_1 = 1,
  LSM6DS_gyroLPF1_2 = 2,
  LSM6DS_gyroLPF1_3 = 3,
  LSM6DS_gyroLPF1_4 = 4,
  LSM6DS_gyroLPF1_5 = 5,
  LSM6DS_gyroLPF1_6 = 6,
  LSM6DS_gyroLPF1_7 = 7,
} gyroLPF1_range_t;

/**  Gyro HPF range (Table 62.) */
typedef enum gyroHPF_range {
  LSM6DS_gyroHPF_0 = 0, // 16mHz
  LSM6DS_gyroHPF_1 = 1, // 65mHz
  LSM6DS_gyroHPF_2 = 2, // 260mHz
  LSM6DS_gyroHPF_3 = 3, // 1.04Hz
} gyroHPF_range_t;

/*!
 *    @brief  Extended LSM6DSOX IMU library from Adafruit_LSM6DSOX & Adafruit_LSM6DS. 
 *            Library includes raw reads and writes to registers and methods for setting IMU filters 
 */
class QuadClass_LSM6DSOX : public Adafruit_LSM6DSOX {
public:
  QuadClass_LSM6DSOX();
  ~QuadClass_LSM6DSOX();
  // Raw
  void write(uint16_t reg, uint8_t reg_bits, uint8_t bits_shift, uint32_t data);
  uint8_t read(uint16_t reg);
  // Filters
  void setAccelCompositeFilter(accelCompositeFilter_t filter, accelCompositeFilter_range_t filter_range);
  void setGyroLPF1(bool enable, gyroLPF1_range_t filter_range);
  void setGyroHPF(bool enable, gyroHPF_range_t filter_range);
};

#endif