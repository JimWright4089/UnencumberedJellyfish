/*!
 * @file Adafruit_BNO055.cpp
 *
 *  @mainpage Adafruit BNO055 Orientation Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 9-DOF Breakout.
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
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */
#include <limits.h>
#include <math.h>

#include "Jims_BNO055.h"
#include <chrono>
#include <thread>

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

/*!
 *  @brief  Instantiates a new Adafruit_BNO055 class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 *  @param  theWire
 *          Wire object
 */
Jims_BNO055::Jims_BNO055(int32_t sensorID, uint8_t address) {
// BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  theWire->setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  mSensorID = sensorID;
  mI2CDev = new Jims_I2CDevice(address);
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool Jims_BNO055::begin(adafruit_bno055_opmode_t mode) {

  /* Make sure we have the right device */
  uint8_t id = read8(BNO055_CHIP_ID_ADDR);
  if (id != BNO055_ID) {
    sleep_for(milliseconds(1000)); // hold on for boot
    id = read8(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
      return false; // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  sleep_for(milliseconds(30));
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    sleep_for(milliseconds(10));
  }
  sleep_for(milliseconds(50));

  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  sleep_for(milliseconds(10));

  write8(BNO055_PAGE_ID_ADDR, 0);
  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  sleep_for(milliseconds(10));
  /* Set the requested operating mode (see section 3.3) */
  setMode(mode);
  sleep_for(milliseconds(20));

  return true;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
void Jims_BNO055::setMode(adafruit_bno055_opmode_t mode) {
  mMode = mode;
  write8(BNO055_OPR_MODE_ADDR, mMode);
  sleep_for(milliseconds(30));
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
void Jims_BNO055::setExtCrystalUse(bool usextal) {
  adafruit_bno055_opmode_t modeback = mMode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  sleep_for(milliseconds(25));
  write8(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  sleep_for(milliseconds(10));
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  sleep_for(milliseconds(20));
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void Jims_BNO055::getCalibration(uint8_t *sys, uint8_t *gyro,
                                     uint8_t *accel, uint8_t *mag) {
  uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t Jims_BNO055::getTemp() {
  int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
  return temp;
}

double Jims_BNO055::getHeading() {
  uint8_t buffer[2];
  memset(buffer, 0, 2);

  int16_t x=0;

  /* Read vector data (6 bytes) */
  readLen(BNO055_EULER_H_LSB_ADDR, buffer, 2);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);

  return ((double)x) / 16.0;
}

double Jims_BNO055::getGyroZ() {
  uint8_t buffer[2];
  memset(buffer, 0, 2);

  int16_t x=0;

  /* Read vector data (6 bytes) */
  readLen(BNO055_GYRO_DATA_Z_LSB_ADDR, buffer, 2);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);

  return ((double)x) / 16.0;
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool Jims_BNO055::isFullyCalibrated() {
  uint8_t system, gyro, accel, mag;
  getCalibration(&system, &gyro, &accel, &mag);

  switch (mMode) {
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
void Jims_BNO055::enterSuspendMode() {
  adafruit_bno055_opmode_t modeback = mMode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  sleep_for(milliseconds(25));
  write8(BNO055_PWR_MODE_ADDR, 0x02);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  sleep_for(milliseconds(20));
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
void Jims_BNO055::enterNormalMode() {
  adafruit_bno055_opmode_t modeback = mMode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  sleep_for(milliseconds(25));
  write8(BNO055_PWR_MODE_ADDR, 0x00);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  sleep_for(milliseconds(20));
}

/*!
 *  @brief  Writes an 8 bit value over I2C
 */
bool Jims_BNO055::write8(adafruit_bno055_reg_t reg, uint8_t value) {
  uint8_t buffer[2] = {(uint8_t)reg, (uint8_t)value};
  return mI2CDev->writeBuffer(buffer, 2);
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 */
uint8_t Jims_BNO055::read8(adafruit_bno055_reg_t reg) {
  uint8_t buffer[1] = {reg};
  mI2CDev->write_then_read(buffer, 1, buffer, 1);
  return (uint8_t)buffer[0];
}

/*!
 *  @brief  Reads the specified number of bytes over I2C
 */
bool Jims_BNO055::readLen(adafruit_bno055_reg_t reg, uint8_t *buffer,
                              uint8_t len) {
  uint8_t reg_buf[1] = {(uint8_t)reg};
  return mI2CDev->write_then_read(reg_buf, 1, buffer, len);
}