//----------------------------------------------------------------------------
//
//  $Workfile: LED.ino
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
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <AStar32U4.h>
#include <stdint.h.>

//----------------------------------------------------------------------------
//  Constants
//----------------------------------------------------------------------------
const long  MAX_TIME = 2000;       // Timeout for comm with the Raspberry Pi
const long  SPIN_TIME = 40;       // Time for spining color
const int   OUR_I2C_ADDR = 43;     // Out I2C address
const int   COLOR_REDUCTION = 27;  // The amount of color to remove on each steop of the spin
const int   YELLOW_LED_BLINK = 1000;  // The Yellow LED blink rate
const int   BAUD_RATE = 115200;     //The serial baud rate used for comms
const int   MAX_PING_SENSOR_COUNT = 6; // The number of PING)) sensors connected to the robot


const uint8 MAX_COLORS    = 37;
const uint8 COLOR_BLUE    =  0;
const uint8 COLOR_GREEN   =  1;
const uint8 COLOR_RED     =  2;
const uint8 COLOR_YELLOW  =  3;
const uint8 COLOR_CYAN    =  5;
const uint8 COLOR_MAGENTA =  6;
const uint8 COLOR_WHITE   = 34;
const uint8 COLOR_BLACK   = 35;
const uint8 COLOR_BRIGHT_GREEN   = 36;

//----------------------------------------------------------------------------
//  Pin Mappings
//----------------------------------------------------------------------------
// Digital
const int   BUZZER_PIN = 6;         // Buzzer
const uint8 LED_PIN  =  7;          // LED is on the 6th digial pin
const int   YELLOW_LED_PIN = 13;    // The Yellow LED
const int   PING_0 = 8;             // PING Right Front
const int   PING_1 = 9;             // PING Right Side
const int   PING_2 = 10;            // PING Right Rear
const int   PING_3 = 11;            // PING Left Rear
const int   PING_4 = 12;            // PING Left Side
const int   PING_5 = 5;             // PING Left Front

// Analog
const int   ANALOG_01_PIN = A0;
const int   ANALOG_02_PIN = A1;

//----------------------------------------------------------------------------
//  Variables
//----------------------------------------------------------------------------
Adafruit_NeoPixel mStrip    = Adafruit_NeoPixel(MAX_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

uint8 mSpinColor = 0;
byte mAlliance = 'O';
byte mLocation = 1;
byte mEnabled = 'D';
byte mExtra = 1;
long mLastTime = 0;
StopWatch mRightLED(MAX_TIME); 
StopWatch mSpinColorSW(SPIN_TIME);
uint8 mSensors[MAX_PING_SENSOR_COUNT];
uint8 mPINGPin[] = {PING_0,PING_1,PING_2,PING_3,PING_4,PING_5}; //loads the ping pin array
uint8 mProxSensor = 0;

int mAnalog01;
int mAnalog02;

// Comm with Raspberry Pi vars
uint8 mBuffer[MAX_PACKET];
uint8 mBufferLoc = 0;
uint8 mSend[MAX_SEND];
bool mGoodPacket = false;
uint16 mCommCount = 0;

// LED
StopWatch mYellowLED(YELLOW_LED_BLINK); 
bool mYellowLEDState = true;
AStar32U4Buzzer mBuzzer;


const uint8 colorArray[] = {
0  , 0 , 255 ,
0 , 128 , 0 ,
255 , 0 , 0 ,
255 , 255 , 0 ,
255 , 165 , 0 ,
0 , 255 , 255 ,
255 , 0 , 255 ,
245 , 222 , 179 ,
0 , 0 , 139 ,
0 , 100 , 0 ,
165 , 42  , 42  ,
255 , 215 , 0 ,
255 , 69  , 0 ,
135 , 206 , 235 ,
139 , 0 , 139 ,
189 , 183 , 107 ,
100 , 149 , 237 ,
173 , 255 , 47  ,
240 , 128 , 128 ,
184 , 134 , 11  ,
127 , 255 , 212 ,
106 , 90  , 205 ,
255 , 99  , 71  ,
219 , 112 , 147 ,
0 , 255 , 0 ,
238 , 130 , 238 ,
128 , 0 , 0 ,
60  , 179 , 113 ,
72  , 61  , 139 ,
216 , 191 , 184 ,
128 , 128 , 0 ,
65  , 105 , 225 ,
255 , 192 , 203 ,
160 , 82  , 45  ,
255 , 255 , 255 ,
0 , 0 , 0,
0 , 255 , 0};

const uint8 colorArrayText[] = {
COLOR_WHITE,
COLOR_WHITE,
COLOR_WHITE,
COLOR_BLACK,
COLOR_BLACK,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE,
COLOR_WHITE,
COLOR_WHITE,
COLOR_BLACK,
COLOR_BLACK,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE,
COLOR_WHITE,
COLOR_WHITE,
COLOR_BLACK,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE,
COLOR_BLACK,
COLOR_WHITE}; 

//----------------------------------------------------------------------------
//  Purpose:
//      Setup
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void setup() 
{
  Serial.begin(BAUD_RATE);
  pinMode(LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  Wire.begin(OUR_I2C_ADDR);              
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  mStrip.begin();
  mStrip.show();
  IdlePatternSet();
  mYellowLED.Reset();
  
  Serial.print("Starting up");

  for(int i=0;i<2;i++)
  {
    // Start playing a tone with frequency 440 Hz at maximum
    // volume (15) for 200 milliseconds.
    mBuzzer.playFrequency(NOTE_F(3), 200, 15);
    // Delay to give the tone time to finish.
    delay(500);
    // Start playing note A in octave 4 at maximum volume
    // volume (15) for 200 milliseconds.
    mBuzzer.playNote(NOTE_A(4), 200, 15);
    // Wait for 200 ms and stop playing note.
    delay(200);
  }
  mBuzzer.stopPlaying();
}

//----------------------------------------------------------------------------
//  Purpose:
//      Idle Loop
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void loop() 
{  
  // Handle comm with the Pi
  if(true == mGoodPacket)
  {
    mRightLED.Reset();

    mAlliance = mBuffer[LOC_PI_LED_STATUS];
    int otherCommCount = getU16FrombyteArray(mBuffer, LOC_PI_COUNT);
    mCommCount++;
    putU16IntoU8Array(mSend,LOC_AR_COUNT,mCommCount);
    Serial.print(" got:");
    Serial.print(mAnalog01);
    Serial.print(" ");
    Serial.print(mAnalog02);
    Serial.print(" ");
    Serial.println(otherCommCount);
    RemoveDataForNextMessage(MAX_RECEIVE, true);
    mGoodPacket = false;
  }

  getProxSensors();

  mAnalog01 = analogRead(ANALOG_01_PIN);
  mAnalog02 = analogRead(ANALOG_02_PIN);

  for(int i=0;i<MAX_PING_SENSOR_COUNT;i++)
  {
    mSend[i+LOC_AR_SENSOR_START] = mSensors[i];
  }
  putU16IntoU8Array(mSend,LOC_AR_ANALOG_1_START,mAnalog01);
  putU16IntoU8Array(mSend,LOC_AR_ANALOG_2_START,mAnalog02);


  // Handle the LED ring
  if(true == mRightLED.IsExpired())
  {
    ColorSet(COLOR_BRIGHT_GREEN);
  }
  else
  {
    if('B' == mAlliance)
    {
      SpinColor(COLOR_BLUE);      
    }
    else
    {
      if('R' == mAlliance)
      {
        SpinColor(COLOR_RED);      
      }
      else
      {
        ColorSet(COLOR_BRIGHT_GREEN);
      }
    }
  }

  if(true == mYellowLED.IsExpired())
  {
    mYellowLEDState = !mYellowLEDState;
    digitalWrite(YELLOW_LED_PIN, mYellowLEDState);
    mYellowLED.Reset();
  }

  // Let things settle if need be
  delay(1);
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  Proximity Sensors
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Get all of the prox sensors and fill the global array
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void getProxSensors()
{
  mSensors[mProxSensor] = ping(mPINGPin[mProxSensor]);
  mProxSensor++;

  if(mProxSensor>=MAX_PING_SENSOR_COUNT)
  {
    mProxSensor = 0;
  }
}


int ping(int pingPin)
{
  // create local variables for duration of ping
  long duration, cm;

  // The PING)) is triggered by a HIGH pulse of 2 or more microseconds.
  // To ensure a clean pulse, we'll provide the pin with a LOW pulse prior.

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the 
  // object we take half the distance traveled.
  return microseconds / 29 /2;
}
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  LED Ring
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Spin The Color Ring
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void SpinColor(int color)
{ 
  int red   = colorArray[(color*3)];
  int blue  = colorArray[(color*3)+1];
  int green = colorArray[(color*3)+2];
  
  for(int i=0;i<16;i++)
  {
    uint8 realLoc = (mSpinColor+i)%16;
    mStrip.setPixelColor(realLoc, mStrip.Color((uint8)red, (uint8)blue, (uint8)green));

    if(0 > (red-COLOR_REDUCTION))
    {
      red = 0;
    }
    else
    {
      red-=COLOR_REDUCTION;
    }
    if(0 > (blue-COLOR_REDUCTION))
    {
      blue = 0;
    }
    else
    {
      blue-=COLOR_REDUCTION;
    }
    
    if(0 > (green-COLOR_REDUCTION))
    {
      green = 0;
    }
    else
    {
      green-=COLOR_REDUCTION;
    }
  }
  if(true == mSpinColorSW.IsExpired())
  {
    mStrip.show();
    mSpinColor--;
    mSpinColorSW.Reset();
  }
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  I2C Handler
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Send bytes to the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void requestEvent() 
{
  mSend[LOC_START] = SER_START;
  mSend[MAX_SEND - LOC_CHECK_BYTE] = CalcCheckByte(mSend, LOC_PI_STATUS, MAX_SEND - LOC_DATA_END);
  mSend[MAX_SEND - LOC_END] = SER_END;

  for (uint8 index = 0; index < MAX_SEND; index++)
  {
    Wire.write(mSend[index]);
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
void receiveEvent(int howMany) 
{
  uint8 theByte = 0;
  uint8 count = 0;

  while ((Wire.available() > 0) && (mBufferLoc < MAX_PACKET))
  {
    theByte = Wire.read();

    // Add the byte to the buffer
    mBuffer[mBufferLoc] = theByte;
    mBufferLoc++;
  }

  count = 0;
  // Trim the garbage from the start
  while ((SER_START != mBuffer[0]) && (count < mBufferLoc))
  {
    count++;
  }

  if (count > 0)
  {
    RemoveDataForNextMessage(count, true);
  }

  // find if we are good or have garbage
  int nextMessage = FindNextMessage();

  if (true == DoWeHaveAGoodMessage())
  {
    mGoodPacket = true;
  }
  else
  {
    // Trim garbage if there is any
    if (nextMessage > 0)
    {
      RemoveDataForNextMessage(nextMessage, true);
    }
  }
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  Comm Utils
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Return if the packet is well formed
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
bool DoWeHaveAGoodMessage()
{
  bool returnValue = false;
  uint8 theLength = MAX_RECEIVE;

  if ((mBufferLoc >= theLength) && (mBufferLoc != 0) && (theLength != 0))
  {
    //Is the preamble where it should be
    if ((mBuffer[LOC_START] == SER_START) && (mBuffer[theLength - LOC_END] == SER_END))
    {
      uint8 checkByte = CalcCheckByte(mBuffer, LOC_PI_STATUS, theLength - LOC_DATA_END);

      if (checkByte == mBuffer[theLength - LOC_CHECK_BYTE])
      {
        returnValue = true;
      }
      else
      {
        RemoveDataForNextMessage(theLength, true);
      }
    }
    else
    {
      RemoveDataForNextMessage(theLength, true);
    }
  }
  return returnValue;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Calc a check byte from the data
//
//  Notes:
//      This xors all the data together
//
//----------------------------------------------------------------------------
uint8 CalcCheckByte(uint8* data, uint8 start, uint8 number)
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
uint8 FindNextMessage()
{
  uint8 nextMessIndex = 0;
  bool found = false;
  uint8 theLength = MAX_RECEIVE;

  //Is the preamble where it should be
  if ((mBuffer[LOC_START] == SER_START)&&(mBufferLoc>2))
  {
    theLength = MAX_RECEIVE;
    //From the end of the message search the rest of what we have gotten
    //for another preamble.
    for (nextMessIndex = 1; nextMessIndex < mBufferLoc; nextMessIndex++)
    {
      if (mBuffer[nextMessIndex] == SER_START)
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
void RemoveDataForNextMessage(uint8 offset, bool isBad)
{
  int index;

  //Move the first 'offset' number of bytes forward.
  for (index = 0; index < mBufferLoc - offset; index++)  // JSF162 JSF213 Exception
  {
    mBuffer[index] = mBuffer[offset + index];
  }

  //if we have been asked to remove more bytes than we have set the number
  //of bytes to 0.
  if (offset > mBufferLoc)
  {
    mBufferLoc = 0;
  }
  else
  {
    //If not then reduce the number of bytes we have by the offset.
    mBufferLoc -= offset;
  }

  //Move the rest of the message down to right after the 'offset' bytes.
  // Process the rest of the buffer
  for (; index < MAX_PACKET; index++)   // JSF200 JSF162 Exception
  {
    //If we are under the MAX packet size then move the data.
    if ((offset + index) < (mBufferLoc))
    {
      mBuffer[index] = mBuffer[offset + index];
    }
    else
    {
      //If we are over the MAX packet size then clear out the bytes.
      mBuffer[index] = 0;
    }
  }
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  LED Handler
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Return the combined color
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint32_t GetLEDColor(uint8_t colorIndex)
{
  if(colorIndex>=MAX_COLORS)
  {
    colorIndex = COLOR_BLACK; 
  }

  uint8 red   = colorArray[(colorIndex*3)];
  uint8 blue  = colorArray[(colorIndex*3)+1];
  uint8 green = colorArray[(colorIndex*3)+2];

  return(mStrip.Color(red, blue, green));
}

//----------------------------------------------------------------------------
//  Purpose:
//      
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void ColorSet(uint8 team) 
{
  if(COLOR_NO_COMM == team)
  {
    IdlePatternSet();
  }
  else
  {
    {
      uint8 theColor = team;
      
      uint32_t color = GetLEDColor(theColor);
      
      for(uint8 i=0; i<MAX_LEDS; i++) 
      {
          mStrip.setPixelColor(i, color);
      }
      mStrip.show();
    }
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Set the LEDs to the idle pattern 
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void IdlePatternSet() 
{
  uint32_t color = GetLEDColor(COLOR_BLACK);
  uint8 i;
  
  for(i=0; i<4; i++) 
  {
      mStrip.setPixelColor(i, color);
  }
 
  color = GetLEDColor(COLOR_RED);
  for(i=4; i<8; i++) 
  {
      mStrip.setPixelColor(i, color);
  }

  color = GetLEDColor(COLOR_YELLOW);
  for(i=8; i<12; i++) 
  {
      mStrip.setPixelColor(i, color);
  }

  color = GetLEDColor(COLOR_WHITE);
  for(i=12; i<16; i++) 
  {
      mStrip.setPixelColor(i, color);
  }
  
  mStrip.show();
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  Local Class
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
StopWatch::StopWatch() :
  mLastTime(0),
  mWaitTime(1000)
{
}
  
StopWatch::StopWatch(int waitTime) :
  mLastTime(0),
  mWaitTime(waitTime)
{
}

long StopWatch::Now(void)
{
  return millis();
}

void StopWatch::SetTime(int waitTime)
{
  mWaitTime = waitTime;
}

bool StopWatch::IsExpired(void)
{
  if((Now() - mLastTime)>mWaitTime)
  {
     return true;
  }
  return false;
}

void StopWatch::Reset(void)
{
  mLastTime = Now();
}

long StopWatch::GetTimeLeft()
{
  return mWaitTime - (Now()-mLastTime);
}
