#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>

// TODO, test gyro filter addresses
// Separate class into a file with a header
// Upload the files into a Github repo

#define LSM6DS_CTRL4_C 0x13
#define LSM6DS_CTRL6_C 0x15
#define LSM6DS_CTRL7_G 0x16

/**  Accelerometer composite filter */
typedef enum accelCompositeFilter {
  LSM6DS_CompositeFilter_NONE,  // No filter
  LSM6DS_CompositeFilter_LPF2,  // Low pass filter 2
  LSM6DS_CompositeFilter_HPF,   // High Pass filter
} accelCompositeFilter_t;

/**  Accelerometer high pass or low pass 2 filter range */
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


// ######################################################################################################## //
//                                                                                                          //
// Accelerometer Filters                                                                                    //
// ---------------------                                                                                    //
// [Analog AA LP Filter] -> [ADC] -> [Digital LP Filter (ODR)] -> [Composite Filter]         ->   Output    //
//                                   setAccelDataRate()           setAccelCompositeFilter()                 //
//                                   (Table 44.)                  (Table 65.)                               //
//                                                                                                          //
//                                                                                                          //  
// Gyroscope Filters                                                                                        //
// ---------------------                                                                                    //
// [ADC]   =>  [HPF]         =>  [LPF1]         =>  [LPF2 (ODR)]       ->   Output                          //
//             setGyroHPF()      setGyroLPF1()      setGyroDataRate()                                       //
//             (Table 62.)       (Table 60.)        (Table 18.)                                             //
//                                                                                                          //  
// ######################################################################################################## //

// Class required for accessing protected i2c_dev and spi_dev members in Adafruit_LSM6DS inherited by Adafruit_LSM6DSOX
class Quad_LSM6DSOX : public Adafruit_LSM6DSOX {
public:
  Quad_LSM6DSOX() {}
  ~Quad_LSM6DSOX() {}

  void write(uint16_t reg, uint8_t reg_bits, uint8_t bits_shift, uint32_t data) {

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

  uint8_t read(uint16_t reg) {

    if (i2c_dev == NULL) {
      Serial.println("Quad_LSM6DS read: Initialize I2C first");
      return 0;
    }

    Adafruit_BusIO_Register LSM6DSOX_register = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, reg);

    return LSM6DSOX_register.read();
  }

  void setAccelCompositeFilter(accelCompositeFilter_t filter, accelCompositeFilter_range_t filter_range) {
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

  void setGyroLPF1(bool enable, gyroLPF1_range_t filter_range) {
    switch (enable) {
      case true:
        this->write(LSM6DS_CTRL4_C, 1, 1, true); // LPF1_SEL_G = 1
        this->write(LSM6DS_CTRL6_C, 3, 0, filter_range); // FTYPE_[0,1,2] = filter_range
        break;
      case false:
        this->write(LSM6DS_CTRL4_C, 1, 1, false); // LPF1_SEL_G = 0
        this->write(LSM6DS_CTRL6_C, 3, 0, 0); // FTYPE_[0,1,2] = 0
        break;
    }
  }

  void setGyroHPF(bool enable, gyroHPF_range_t filter_range) {
    switch (enable) {
      case true:
        this->write(LSM6DS_CTRL7_G, 1, 6, true); // HP_EN_G = 1
        this->write(LSM6DS_CTRL7_G, 2, 4, filter_range); // HPM[0,1]_G = filter_range
        break;
      case false:
        this->write(LSM6DS_CTRL7_G, 1, 6, false); // HP_EN_G = 0
        this->write(LSM6DS_CTRL7_G, 2, 4, 0); // HPM[0,1]_G = 0
        break;
    }
  }
};

Quad_LSM6DSOX sox = Quad_LSM6DSOX();
quad_data_t orientation;

Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation


void update_orientation(void) {
  ahrs->getQuadOrientation(&orientation);
  orientation.pitch_rate = -orientation.pitch_rate;
  orientation.roll_rate = -orientation.roll_rate;
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

  Serial.println("Basic Configuration Set!");

  _accel = sox.getAccelerometerSensor();
  _gyro = sox.getGyroSensor();
  ahrs = new Adafruit_Simple_AHRS(_accel, _mag, _gyro);
  ahrs->getQuadOrientation(&orientation);

  Serial.println("WHOAMI");
  Serial.println(sox.read(LSM6DS_WHOAMI), HEX);
  Serial.println("=====");
  Serial.println("LSM6DS_CTRL8_XL");
  Serial.println(sox.read(LSM6DS_CTRL8_XL), BIN);
  Serial.println("LSM6DS_CTRL1_XL");
  Serial.println(sox.read(LSM6DS_CTRL1_XL), BIN);
  sox.setAccelCompositeFilter(LSM6DS_CompositeFilter_HPF, LSM6DS_CompositeFilter_ODR_DIV_800);
  Serial.println("LSM6DS_CTRL1_XL");
  Serial.println(sox.read(LSM6DS_CTRL1_XL), BIN);
  Serial.println("LSM6DS_CTRL8_XL");
  Serial.println(sox.read(LSM6DS_CTRL8_XL), BIN);
  Serial.println("===== setGyroLPF1");
  Serial.println("LSM6DS_CTRL4_C");
  Serial.println(sox.read(LSM6DS_CTRL4_C), BIN);
  Serial.println("LSM6DS_CTRL6_C");
  Serial.println(sox.read(LSM6DS_CTRL6_C), BIN);
  sox.setGyroLPF1(true, LSM6DS_gyroLPF1_3);
  Serial.println("LSM6DS_CTRL4_C");
  Serial.println(sox.read(LSM6DS_CTRL4_C), BIN);
  Serial.println("LSM6DS_CTRL6_C");
  Serial.println(sox.read(LSM6DS_CTRL6_C), BIN);
  Serial.println("===== setGyroHPF");
  Serial.println("LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  sox.setGyroHPF(true, LSM6DS_gyroHPF_3);
  Serial.println("LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("LSM6DS_CTRL7_G");
  Serial.println(sox.read(LSM6DS_CTRL7_G), BIN);
  Serial.println("=====");
}


void loop() {

  sensors_event_t accel;
  sensors_event_t gyro;

  Serial.println(_accel == NULL);
  Serial.println(_gyro == NULL);

  _accel->getEvent(&accel);
  _gyro->getEvent(&gyro);

  //  /* Get a new normalized sensor event */
  // sensors_event_t accel;
  // sensors_event_t gyro;
  // sensors_event_t temp;
  // sox.getEvent(&accel, &gyro, &temp);

  // Serial.print("\t\tTemperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

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


  update_orientation();

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tOrentation Roll: ");
  Serial.print(orientation.roll);
  Serial.print(" \tPitch: ");
  Serial.print(orientation.pitch);
  Serial.println();

  delay(2000);
}