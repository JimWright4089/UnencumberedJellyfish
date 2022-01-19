#include "Jims_RobotClaw.h"

int main(void)
{
  Jims_RobotClaw claw = Jims_RobotClaw("/dev/ttyACM0",38400);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  bool results = false;
  int32_t value = 0;

  /////////////////////////////////////////////////////////////////////
  ////
  ////   Left Forward
  ////
  /////////////////////////////////////////////////////////////////////
  results = claw.resetEncoders();
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  results = claw.setLeftMotor(.5);
  if(false == results)
  {
    printf("Bad set Motor!!\n");
  }

  for(int i=0;i<5;i++)
  {
    value = claw.getLeftEncoder();
    printf("Left Forward 1:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setLeftEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getLeftEncoder();
    printf("Left Forward 2:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getLeftSpeed();
    double dvaluer = claw.getRightCurrent();
    double dvaluel = claw.getLeftCurrent();
    printf("Left Forward speed:%d %f %f\n",value,dvaluel,dvaluer);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopLeftMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }

  /////////////////////////////////////////////////////////////////////
  ////
  ////   Right Forward
  ////
  /////////////////////////////////////////////////////////////////////
  results = claw.resetEncoders();
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  results = claw.setRightMotor(.5);
  if(false == results)
  {
    printf("Bad set Motor!!\n");
  }

  for(int i=0;i<5;i++)
  {
    value = claw.getRightEncoder();
    printf("Right Forward 1:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setRightEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getRightEncoder();
    printf("Right Forward 2:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getRightSpeed();
    double dvaluer = claw.getRightCurrent();
    double dvaluel = claw.getLeftCurrent();
    printf("Right Forward Speed:%d %f %f\n",value,dvaluer,dvaluel);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopRightMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }

  /////////////////////////////////////////////////////////////////////
  ////
  ////   Left Reverse
  ////
  /////////////////////////////////////////////////////////////////////
  results = claw.resetEncoders();
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  results = claw.setLeftMotor(-.5);
  if(false == results)
  {
    printf("Bad set Motor!!\n");
  }

  for(int i=0;i<5;i++)
  {
    value = claw.getLeftEncoder();
    printf("Left Reverse 1:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setLeftEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getLeftEncoder();
    printf("Left Reverse 2:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getLeftSpeed();
    double dvalue = claw.getLeftCurrent();
    printf("Left Reverse Speed:%d %f\n",value,dvalue);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopLeftMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }

  /////////////////////////////////////////////////////////////////////
  ////
  ////   Right Reverse
  ////
  /////////////////////////////////////////////////////////////////////
  results = claw.resetEncoders();
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  results = claw.setRightMotor(-.5);
  if(false == results)
  {
    printf("Bad set Motor!!\n");
  }

  for(int i=0;i<5;i++)
  {
    value = claw.getRightEncoder();
    printf("Right Reverse 1:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setRightEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getRightEncoder();
    printf("Right Reverse 2:%d\n",value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  for(int i=0;i<5;i++)
  {
    value = claw.getRightSpeed();
    double dvalue = claw.getRightCurrent();
    printf("Right Reverse Speed:%d %f\n",value,dvalue);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopRightMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }


  return 0;
}
