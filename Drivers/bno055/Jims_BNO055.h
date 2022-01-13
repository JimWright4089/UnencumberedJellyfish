/*!
 *  @file Adafruit_BNO055.h
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef __JIMS_BNO055_H__
#define __JIMS_BNO055_H__

#include "Jims_I2CDevice.h"

/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

  /** Operation mode settings **/
  typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
  } adafruit_bno055_opmode_t;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          BNO055 Sensor
 */
class Jims_BNO055 {
public:
  /** BNO055 Registers **/
  typedef enum {
    /* Page id register definition */
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_PAGE_ID_ADDR = 0X07,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    BNO055_EULER_H_MSB_ADDR = 0X1B,
    BNO055_TEMP_ADDR = 0X34,
    BNO055_CALIB_STAT_ADDR = 0X35,
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,
    BNO055_SYS_TRIGGER_ADDR = 0X3F,
  } adafruit_bno055_reg_t;

  /** BNO055 power settings */
  typedef enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
  } adafruit_bno055_powermode_t;

  Jims_BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A);

  bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
  void setExtCrystalUse(bool usextal);
  void setMode(adafruit_bno055_opmode_t mode);
  void getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel,
                      uint8_t *mag);

  double getHeading();
  double getGyroZ();
  int8_t getTemp();

  bool isFullyCalibrated();

  /* Power managments functions */
  void enterSuspendMode();
  void enterNormalMode();

private:
  uint8_t read8(adafruit_bno055_reg_t);
  bool readLen(adafruit_bno055_reg_t, uint8_t *buffer, uint8_t len);
  bool write8(adafruit_bno055_reg_t, uint8_t value);

  Jims_I2CDevice *mI2CDev = NULL; ///< Pointer to I2C bus interface

  int32_t                  mSensorID;
  adafruit_bno055_opmode_t mMode;
};

#endif