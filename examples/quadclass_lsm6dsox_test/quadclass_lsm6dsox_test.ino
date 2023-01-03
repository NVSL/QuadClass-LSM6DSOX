#include <QuadClass_LSM6DSOX.h>
/*
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

QuadClass_LSM6DSOX sox = QuadClass_LSM6DSOX();
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation

void setup() {

  // Start Serial

  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("QuadClass LSM6DSOX test!");

  // Start LSM6DSOX IMU

  if (!sox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");

  // Basic LSM6DSOX IMU configuration

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.println("Basic Configuration Set!");

  // Advanced LSM6DSOX IMU configuration using QuadClass_LSM6DSOX extended library

  Serial.println("===== Basic Read");
  Serial.println("Read Register: WHOAMI");
  Serial.println(sox.read(LSM6DS_WHOAMI), HEX);

  Serial.println("===== Accelerometer Composite Filter Write Test");
  Serial.println("Read Register: LSM6DS_CTRL8_XL");
  Serial.println(sox.read(LSM6DS_CTRL8_XL), BIN);
  Serial.println("Read Register: LSM6DS_CTRL1_XL");
  Serial.println(sox.read(LSM6DS_CTRL1_XL), BIN);
  Serial.println("> Accelerometer Composite Filter Set");
  sox.setAccelCompositeFilter(LSM6DS_CompositeFilter_HPF, LSM6DS_CompositeFilter_ODR_DIV_800);
  Serial.println("Read Register: LSM6DS_CTRL1_XL");
  Serial.println(sox.read(LSM6DS_CTRL1_XL), BIN);
  Serial.println("Read Register: LSM6DS_CTRL8_XL");
  Serial.println(sox.read(LSM6DS_CTRL8_XL), BIN);

  Serial.println("===== Gyroscope Low-Pass-1 Filter Write Test");
  Serial.println("Read Register: LSM6DS_CTRL4_C");
  Serial.println(sox.read(LSM6DS_CTRL4_C), BIN);
  Serial.println("Read Register: LSM6DS_CTRL6_C");
  Serial.println(sox.read(LSM6DS_CTRL6_C), BIN);
  Serial.println("> Gyroscope Low-Pass-1 Filter Set");
  sox.setGyroLPF1(true, LSM6DS_gyroLPF1_3);
  Serial.println("Read Register: LSM6DS_CTRL4_C");
  Serial.println(sox.read(LSM6DS_CTRL4_C), BIN);
  Serial.println("Read Register: LSM6DS_CTRL6_C");
  Serial.println(sox.read(LSM6DS_CTRL6_C), BIN);

  Serial.println("===== Gyroscope High-Pass Filter Write Test");
  Serial.println("Read Register: LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("Read Register: LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("> Gyroscope High-Pass Filter Set");
  sox.setGyroHPF(true, LSM6DS_gyroHPF_3);
  Serial.println("Read Register: LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("Read Register: LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("===== END");


  // Get Accel and Gyro AdafruitSensor Objects

  _accel = sox.getAccelerometerSensor();
  _gyro = sox.getGyroSensor();
}


void loop() {

  sensors_event_t accel;
  sensors_event_t gyro;

  _accel->getEvent(&accel);
  _gyro->getEvent(&gyro);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(2000);
}