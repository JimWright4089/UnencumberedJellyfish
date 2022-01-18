#include "Jims_RobotClaw.h"

int main(void)
{
  Jims_RobotClaw claw = Jims_RobotClaw("/dev/ttyACM0",38400);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  bool results = claw.resetEncoders();
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
    claw.getLeftEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setLeftEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    claw.getLeftEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopLeftMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }


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
    claw.getRightEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setRightEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    claw.getRightEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopRightMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }

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
    claw.getLeftEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setLeftEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    claw.getLeftEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopLeftMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }


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
    claw.getRightEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.setRightEncoder(100);
  if(false == results)
  {
    printf("Bad reset!!\n");
  }
  for(int i=0;i<5;i++)
  {
    claw.getRightEncoder();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  results = claw.stopRightMotor();
  if(false == results)
  {
    printf("Bad!!\n");
  }


  return 0;
}
