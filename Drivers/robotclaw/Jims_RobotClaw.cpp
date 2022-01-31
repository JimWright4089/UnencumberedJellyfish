#include "Jims_RobotClaw.h"

//#define ROBOTCLAW_DEBUG 1

Jims_RobotClaw::Jims_RobotClaw(string portName, uint32_t baud) :
  mSerialPort(portName,baud)
{
  mWheelCirc = mWheelDiameter * 3.14159;
}

bool Jims_RobotClaw::setLeftMotor(double speed)
{
  return(setSpeed(speed,CMD_MOTOR2_FORWARD,CMD_MOTOR2_BACKWARDS));
}

bool Jims_RobotClaw::setRightMotor(double speed)
{
  return(setSpeed(speed,CMD_MOTOR1_FORWARD,CMD_MOTOR1_BACKWARDS));
}

bool Jims_RobotClaw::stopLeftMotor()
{
  return(setLeftMotor(0));
}

bool Jims_RobotClaw::stopRightMotor()
{
  return(setRightMotor(0));
}

bool Jims_RobotClaw::setLeftSpeed(int32_t speed)
{
  return setEncoder(CMD_MOTOR2_SPEED, speed);
}

bool Jims_RobotClaw::setRightSpeed(int32_t speed)
{
  return setEncoder(CMD_MOTOR1_SPEED, speed);
}


bool Jims_RobotClaw::setSpeed(double speed, uint8_t forward, uint8_t backwards)
{
	uint8_t msg[] = { mAddress, forward, 45, 'x', 'x' };
  uint8_t readBuffer[1];

  if(speed < 0)
  {
    speed *= -1;
    msg[LOC_COMMAND] = backwards;
  }

  msg[LOC_SPEED_COMMAND] = (uint8_t)(speed*MAX_SPEED);

  unsigned int crc = crc16(msg, 3);
  msg[3] = (crc>>8)&0xff;
  msg[4] = (crc)&0xff;

  mSerialPort.flush();
  bool results = mSerialPort.write_then_read(msg, sizeof(msg),readBuffer, sizeof(readBuffer));

  if(false == results)
  {
    return results;
  }

#ifdef ROBOTCLAW_DEBUG
  printf("byte received: %02x\n",readBuffer[0]);
#endif

  return(CMD_RESPONCE == readBuffer[0]);
}

int32_t Jims_RobotClaw::getLeftEncoder()
{
  return getEncoder(CMD_READ_MOTOR2_ENCODER);
}

int32_t Jims_RobotClaw::getRightEncoder()
{
  return getEncoder(CMD_READ_MOTOR1_ENCODER);
}

double Jims_RobotClaw::getLeftSpeed()
{
  return ((double)getEncoder(CMD_READ_MOTOR2_SPEED)/mTicksPerRev)*mWheelCirc;
}

double Jims_RobotClaw::getRightSpeed()
{
  return ((double)getEncoder(CMD_READ_MOTOR1_SPEED)/mTicksPerRev)*mWheelCirc;
}

double Jims_RobotClaw::getLeftCurrent()
{
  uint32_t value = (uint32_t)getEncoder(CMD_READ_MOTORS_CURRENT);
  uint16_t currentInt = (value&0xFFFF);

  return (double)((double)currentInt/CURRENT_DIVISOR);
}

double Jims_RobotClaw::getRightCurrent()
{
  uint32_t value = (uint32_t)getEncoder(CMD_READ_MOTORS_CURRENT);
  uint16_t currentInt = ((value>>16)&0xFFFF);

  return (double)((double)currentInt/CURRENT_DIVISOR);
}


int32_t Jims_RobotClaw::getEncoder(uint8_t encoder)
{
  unsigned char msg[] = { mAddress, encoder, 'x', 'x' };
  unsigned int crc = crc16(msg, 2);
  msg[2] = (crc>>8)&0xff;
  msg[3] = (crc)&0xff;
  uint8_t readBuffer [7];

  mSerialPort.flush();
  bool results = mSerialPort.write_then_read(msg, sizeof(msg),readBuffer, sizeof(readBuffer));

  if(false == results)
  {
    return 0;
  }

#ifdef ROBOTCLAW_DEBUG
  printf("encoder:");
  for(int i=0;i<7;i++)
  {
    printf("%02x ",readBuffer[i]);
  }
  printf(" ");
#endif

  int32_t encoderValue = 0;

  encoderValue = readBuffer[0];
  encoderValue = encoderValue << 8;
  encoderValue += readBuffer[1];
  encoderValue = encoderValue << 8;
  encoderValue += readBuffer[2];
  encoderValue = encoderValue << 8;
  encoderValue += readBuffer[3];

#ifdef ROBOTCLAW_DEBUG
  printf("%d\n",encoderValue);
#endif
  return encoderValue;
}

bool Jims_RobotClaw::resetEncoders()
{
	uint8_t msg[] = { mAddress, CMD_RESET_ENCODERS, 'x', 'x' };
  uint8_t readBuffer[1];

  unsigned int crc = crc16(msg, 2);
  msg[2] = (crc>>8)&0xff;
  msg[3] = (crc)&0xff;

  mSerialPort.flush();
  bool results = mSerialPort.write_then_read(msg, sizeof(msg),readBuffer, sizeof(readBuffer));

  if(false == results)
  {
    return 0;
  }

#ifdef ROBOTCLAW_DEBUG
  printf("byte received: %02x\n",readBuffer[0]);
#endif

  return(CMD_RESPONCE == readBuffer[0]);

}

bool Jims_RobotClaw::setLeftEncoder(int32_t value)
{
  return(setEncoder(CMD_SET_MOTOR2_ENCODER,value));
}

bool Jims_RobotClaw::setRightEncoder(int32_t value)
{
  return(setEncoder(CMD_SET_MOTOR1_ENCODER,value));
}

bool Jims_RobotClaw::setEncoder(uint8_t encoder, int32_t value)
{
	uint8_t msg[] = { mAddress, encoder, '1', '2', '3', '4', 'x', 'x' };
  uint8_t readBuffer[1];

  msg[5] = (uint8_t)(value&0xff);
  value = value >> 8;
  msg[4] = (uint8_t)(value&0xff);
  value = value >> 8;
  msg[3] = (uint8_t)(value&0xff);
  value = value >> 8;
  msg[2] = (uint8_t)(value&0xff);

  unsigned int crc = crc16(msg, 6);
  msg[6] = (crc>>8)&0xff;
  msg[7] = (crc)&0xff;

  mSerialPort.flush();
  bool results = mSerialPort.write_then_read(msg, sizeof(msg),readBuffer, sizeof(readBuffer));

  if(false == results)
  {
    return 0;
  }

#ifdef ROBOTCLAW_DEBUG
  printf("byte received: %02x\n",readBuffer[0]);
#endif

  return(CMD_RESPONCE == readBuffer[0]);
}


void Jims_RobotClaw::setAddress(uint8_t address)
{
  mAddress = address;
}

uint16_t Jims_RobotClaw::crc16(uint8_t *packet, uint16_t nBytes)
{
  uint16_t crc = 0;
  for (uint16_t byte = 0; byte < nBytes; byte++)
  {
    crc = crc ^ ((uint16_t)packet[byte] << 8);
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  return crc;
}
