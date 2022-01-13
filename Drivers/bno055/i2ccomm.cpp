//----------------------------------------------------------------------------
//
//  $Workfile: i2cComm.cpp$
//
//  $Revision: X$
//
//  Project:    Unencumbered Jellyfish
//
//                            Copyright (c) 2022
//                               Jim Wright
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
#include <chrono>
#include <thread>
#include <iostream>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include "StopWatch.hpp"

using namespace std;

//----------------------------------------------------------------------------
//  Local typedefs
//----------------------------------------------------------------------------
typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned long  uint32;
typedef char  sint8;
typedef short sint16;
typedef long  sint32;

//----------------------------------------------------------------------------
//  Local constants
//----------------------------------------------------------------------------
const uint8 SER_START = 0xA0;
const uint8 SER_END = 0xA1;

const uint8 LOC_START = 0;
const uint8 LOC_AR_STATUS = 1;
const uint8 LOC_AR_COUNT = 2;
const uint8 LOC_AR_SENSOR_START = 4;
const uint8 LOC_AR_ANALOG_1_START = 10;
const uint8 LOC_AR_ANALOG_2_START = 12;
const uint8 LOC_END = 1;
const uint8 LOC_CHECK_BYTE = 2;
const uint8 LOC_DATA_END = 3;

const uint8 LOC_PI_STATUS = 1;
const uint8 LOC_PI_LED_STATUS = 2;
const uint8 LOC_PI_COUNT = 3;

const uint8  MAX_SEND = 7;
const uint8  MAX_RECEIVE = 16;
const int    MAX_PACKET = 2000;
const int    WAIT_TIME = 5000;  //us
const int    I2C_ADDR = 0x28;
//#define BNO055_ID (0xA0)
const int    SMALL_READ_BUFFER = 60;
const int    MAX_SENSORS = 6;
const int    BAD_VALUE = -1;
const int    STALE_TIME = 2000; // two seconds
const uint8  PI_STATUS_OK = 0;
const uint8  ROBORIO_STALE = 1;

const uint8_t BNO055_CHIP_ID_ADDR = 0x00;


//----------------------------------------------------------------------------
//  Local functions
//----------------------------------------------------------------------------
void  receiveEvent();
bool  doWeHaveAGoodMessage();
uint8 calcCheckByte(uint8* data, uint8 start, uint8 number);
uint8 findNextMessage();
void  removeDataForNextMessage(uint8 offset, bool isBad);
void  sendMessage();
void  putU16IntoU8Array(uint8* data, uint8 location, int value);
int   getU16FrombyteArray(uint8* data, int location);
void  iniI2C(int addr);

//----------------------------------------------------------------------------
//  Global vars
//----------------------------------------------------------------------------
int    gFileI2c;
uint8  gBuffer[MAX_PACKET];
uint8  gBufferLoc = 0;
uint8  gSend[MAX_SEND];
bool   gGoodPacket = false;
uint16 gCommCount = 0;
uint8  gReadBuffer[SMALL_READ_BUFFER];
int    gSensors[MAX_SENSORS];
int    gAnalog1 = BAD_VALUE;
int    gAnalog2 = BAD_VALUE;
int    gSensorFlags = BAD_VALUE;
StopWatch gStale(STALE_TIME);
uint8  gLEDStatus = 'G';
uint8  gOldLEDStatus = 'G';

void writeByte(uint8 ch);
uint8_t readBytes();

uint8_t read8(uint8_t reg) {
  writeByte(reg);
  uint8_t length = readBytes();

  printf("Len=%d:",length);
  for(int i=0;i<length;i++)
  {
    printf("%2x",gReadBuffer[i]);
  }
  printf("\n");

  return gReadBuffer[0];
}

//----------------------------------------------------------------------------
//  Purpose:
//      Main loop
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
int main(void)
{
  printf("Starting\n");

  int comCount = 0;
  int otherCommCount = 0;
  int oldOtherCommCount = 0;
  char stringBuffer[MAX_PACKET];

  std::this_thread::sleep_for(std::chrono::seconds(4));
  iniI2C(I2C_ADDR);
  printf("After Init\n");

//  uint8_t id = read8(BNO055_CHIP_ID_ADDR);

  for(int i=0;i<0x30;i++)
  {
    printf("byte %2x: %2x\n",i,read8(i));
  }

/*
  while (true)
  {
    if (true == gStale.IsExpired())
    {
      gSend[LOC_PI_STATUS] = ROBORIO_STALE;
    }
    else
    {
      gSend[LOC_PI_STATUS] = PI_STATUS_OK;
    }

    try
    {
      gLEDStatus = (uint8)nt::GetEntryValue("/toPi/LED")->GetDouble();
    }
    catch (...)
    {
      nt::StopClient();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      nt::StartClient("roborio-4089-frc.local", 1735);
      fprintf(fp, "Exception\n");
      fflush(fp);
    }

    if (gLEDStatus != gOldLEDStatus)
    {
      printf("%c\n", gLEDStatus);
      gOldLEDStatus = gLEDStatus;
    }

    gSend[LOC_PI_LED_STATUS] = gLEDStatus;
    sendMessage();
    usleep(WAIT_TIME);
    receiveEvent();
    fprintf(fp, "Loop c\n");

    if (true == gGoodPacket)
    {
      gGoodPacket = false;
      gStale.Reset();
      comCount++;
      putU16IntoU8Array(gSend, LOC_PI_COUNT, comCount);
      otherCommCount = getU16FrombyteArray(gBuffer, LOC_AR_COUNT);
      if ((0 != otherCommCount)&&(0xFFFF != otherCommCount))
      {
        printf("Good packet:%d\n", otherCommCount);
      }
      oldOtherCommCount = otherCommCount;

      for (int i = 0; i < MAX_SENSORS; i++)
      {
        gSensors[i] = gBuffer[LOC_AR_SENSOR_START+i];
      }
      gAnalog1 = getU16FrombyteArray(gBuffer, LOC_AR_ANALOG_1_START);
      gAnalog2 = getU16FrombyteArray(gBuffer, LOC_AR_ANALOG_2_START);
      gSensorFlags = BAD_VALUE;
    }

    // If the data is stale send bad values
    if (true == gStale.IsExpired())
    {
      for (int i = 0; i < MAX_SENSORS; i++)
      {
        gSensors[i] = BAD_VALUE;
      }
      gAnalog1 = BAD_VALUE;
      gAnalog2 = BAD_VALUE;
      gSensorFlags = BAD_VALUE;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
*/
  printf("End\n");

}

//----------------------------------------------------------------------------
//  Purpose:
//      Startup the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void iniI2C(int addr)
{
  //----- OPEN THE I2C BUS -----
  char *filename = (char*)"/dev/i2c-1";
  if ((gFileI2c = open(filename, O_RDWR)) < 0)
  {
    //ERROR HANDLING: you can check errno to see what went wrong
    printf("Failed to open the i2c bus");
    return;
  }

  if (ioctl(gFileI2c, I2C_SLAVE, addr) < 0)
  {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    //ERROR HANDLING; you can check errno to see what went wrong
    return;
  }

  printf("Init Good %d\n",gFileI2c);
}


///--------------------------------------------------------------------
/// Purpose:
/// <summary>
///      Pull a short out of a data buffer
/// </summary>
/// 
/// Returns:
/// <returns>
///     The UInt16 from the data
/// </returns>
/// 
/// Notes:
/// <remarks>
///     None.
/// </remarks>
///--------------------------------------------------------------------
int getU16FrombyteArray(uint8* data, int location)
{
  return (int)((data[location+1] << 8) +
    data[location]);
}

//----------------------------------------------------------------------------
//  Purpose:
//      Chop a int up and put it into a buffer
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void putU16IntoU8Array(uint8* data, uint8 location, int value)
{
  data[location + 1] = (char)((value >> 8) & 0xFF);
  data[location + 0] = (char)(value & 0xFF);
}


//----------------------------------------------------------------------------
//  Purpose:
//      Write a byte to I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void writeByte(uint8 ch)
{
  uint8 data[1];

  data[0] = ch;

  //----- WRITE BYTES -----
  uint8 length = 1;			//<<< Number of bytes to write
  if (write(gFileI2c, data, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
  {
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Read a block of bytes into a temp buffer
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint8_t readBytes()
{
  //----- READ BYTES -----
  uint8 length = 1;			//<<< Number of bytes to read
  length = read(gFileI2c, gReadBuffer, length);
  return length;
}


//----------------------------------------------------------------------------
//  Purpose:
//      Get bytes from the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void sendMessage()
{
  gSend[LOC_START] = SER_START;
  gSend[MAX_SEND - LOC_CHECK_BYTE] = calcCheckByte(gSend, LOC_PI_STATUS, MAX_SEND - LOC_DATA_END);
  gSend[MAX_SEND - LOC_END] = SER_END;

  if (write(gFileI2c, gSend, MAX_SEND) != MAX_SEND)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
  {
    printf("Problem sending\n");
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Get bytes from the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void receiveEvent()
{
  int theByte = 0;
  uint8 count = 0;
  int readCount = readBytes();
  int curReadCount = 0;

  while((curReadCount<readCount)&&(gBufferLoc < MAX_PACKET))
  {
    theByte = gReadBuffer[curReadCount++];
    // Add the byte to the buffer
    gBuffer[gBufferLoc] = theByte;
    gBufferLoc++;
  }

  // if we over read the clear everything
  if (gBufferLoc >= MAX_PACKET)
  {
    gBufferLoc = 0;
  }

  count = 0;
  // Trim the garbage from the start
  while ((SER_START != gBuffer[0]) && (count < gBufferLoc))
  {
    count++;
  }

  if (count > 0)
  {
    removeDataForNextMessage(count, true);
  }

  // find if we are good or have garbage
  int nextMessage = findNextMessage();

  if (true == doWeHaveAGoodMessage())
  {
    gGoodPacket = true;
  }
  else
  {
    // Trim garbage if there is any
    if (nextMessage > 0)
    {
      removeDataForNextMessage(nextMessage, true);
    }
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Return if the packet is well formed
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
bool doWeHaveAGoodMessage()
{
  bool returnValue = false;
  uint8 theLength = MAX_RECEIVE;

  if ((gBufferLoc >= theLength) && (gBufferLoc != 0) && (theLength != 0))
  {
    //Is the preamble where it should be
    if ((gBuffer[LOC_START] == SER_START) && (gBuffer[theLength - LOC_END] == SER_END))
    {
      uint8 checkByte = calcCheckByte(gBuffer, LOC_PI_STATUS, theLength - LOC_DATA_END);

      if (checkByte == gBuffer[theLength - LOC_CHECK_BYTE])
      {
        returnValue = true;
      }
      else
      {
        removeDataForNextMessage(theLength, true);
      }
    }
    else
    {
      removeDataForNextMessage(theLength, true);
    }
  }
  return returnValue;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Calc a check byte from the data
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint8 calcCheckByte(uint8* data, uint8 start, uint8 number)
{
  uint8 checkByte = 0xFF;

  for (uint8 index = 0; index < number; index++)
  {
    checkByte ^= data[start + index];
  }
  return checkByte;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Find the next message after the first one
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint8 findNextMessage()
{
  uint8 nextMessIndex = 0;
  bool found = false;
  uint8 theLength = MAX_RECEIVE;

  //Is the preamble where it should be
  if ((gBuffer[LOC_START] == SER_START) && (gBufferLoc>2))
  {
    theLength = MAX_RECEIVE;
    //From the end of the message search the rest of what we have gotten
    //for another preamble.
    for (nextMessIndex = 1; nextMessIndex < gBufferLoc; nextMessIndex++)
    {
      if (gBuffer[nextMessIndex] == SER_START)
      {
        //If we found one stop
        found = true;
        break;
      }
    }
  }

  //If we found one the nextMessIndex should be good.
  //If not then set it to 0 and return 0.
  if (found == false)
  {
    nextMessIndex = 0;
  }
  return nextMessIndex;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Trim the front of the buffer
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void removeDataForNextMessage(uint8 offset, bool isBad)
{
  int index;

  //Move the first 'offset' number of bytes forward.
  for (index = 0; index < gBufferLoc - offset; index++)  // JSF162 JSF213 Exception
  {
    gBuffer[index] = gBuffer[offset + index];
  }

  //if we have been asked to remove more bytes than we have set the number
  //of bytes to 0.
  if (offset > gBufferLoc)
  {
    gBufferLoc = 0;
  }
  else
  {
    //If not then reduce the number of bytes we have by the offset.
    gBufferLoc -= offset;
  }

  //Move the rest of the message down to right after the 'offset' bytes.
  // Process the rest of the buffer
  for (; index < MAX_PACKET; index++)   // JSF200 JSF162 Exception
  {
    //If we are under the MAX packet size then move the data.
    if ((offset + index) < (gBufferLoc))
    {
      gBuffer[index] = gBuffer[offset + index];
    }
    else
    {
      //If we are over the MAX packet size then clear out the bytes.
      gBuffer[index] = 0;
    }
  }
}
