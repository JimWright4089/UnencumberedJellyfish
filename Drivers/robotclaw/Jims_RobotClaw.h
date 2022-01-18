#ifndef __JIMS_ROBOTCLAW_H__
#define __JIMS_ROBOTCLAW_H__

#include "Jims_Serial.h"

class Jims_RobotClaw {
public:
  Jims_RobotClaw(string portName, uint32_t baud);

  bool setLeftMotor(double speed);
  bool setRightMotor(double speed);
  bool setMotors(double speed);

  bool stopLeftMotor();
  bool stopRightMotor();
  bool stopMotors();

  int32_t getLeftEncoder();
  int32_t getRightEncoder();

  bool resetEncoders();
  bool setLeftEncoder(int32_t value);
  bool setRightEncoder(int32_t value);

  void setAddress(uint8_t address);

private:
  const uint8_t LOC_COMMAND = 1;
  const uint8_t LOC_SPEED_COMMAND = 2;

  const double MAX_SPEED = 127.0;

  const uint8_t CMD_MOTOR1_FORWARD = 0;
  const uint8_t CMD_MOTOR1_BACKWARDS = 1;
  const uint8_t CMD_MOTOR2_FORWARD = 4;
  const uint8_t CMD_MOTOR2_BACKWARDS = 5;
  const uint8_t CMD_MOTORS_FORWARD = 8;
  const uint8_t CMD_MOTORS_BACKWARDS = 9;
  const uint8_t CMD_READ_MOTOR1_ENCODER = 16;
  const uint8_t CMD_READ_MOTOR2_ENCODER = 17;

  const uint8_t CMD_RESET_ENCODERS = 20;
  const uint8_t CMD_SET_MOTOR1_ENCODER = 22;
  const uint8_t CMD_SET_MOTOR2_ENCODER = 23;

  const uint8_t CMD_RESPONCE = 0xff;

  uint16_t crc16(uint8_t *packet, uint16_t nBytes);
  bool setSpeed(double speed, uint8_t forward, uint8_t backwards);
  int32_t getEncoder(uint8_t encoder);
  bool setEncoder(uint8_t encoder, int32_t value);

  Jims_Serial mSerialPort;
  uint8_t mAddress = 128;

};

#endif