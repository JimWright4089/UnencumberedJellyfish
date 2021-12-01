#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"

// Anything else, defaults!
#define TFT_CS    9
#define TFT_DC   10
#define TFT_RST  11

//0123456789012345
//BBBBBGGGGGGRRRRR
//0111100111101111
// 0111 1001 1110 1111
#define HX8357_GRAY 0x79EF   ///< WHITE color for drawing graphics

const int MAX_WIDTH = 479;
const int STATUS_MAX_HEIGHT = 27;
const int STATUS_TOP_LINE_Y = 0;
const int STATUS_BOTTOM_LINE_Y = 27;
const int STATUS_TEXT_X_OFFSET = 4;
const int STATUS_TEXT_Y_OFFSET = 4;
const int STATUS2_TEXT_Y_OFFSET = 6;
const int STATUS2_TEXT_SIZE = 2;
const int STATUS_TEXT_SIZE = 3;

const int CAS_FRAME_LEFT = 250;
const int CAS_FRAME_TOP  =  58;
const int CAS_FRAME_RIGHT = MAX_WIDTH;
const int CAS_FRAME_BOTTOM = 279;
const int CAS_FRAME_SIZE = 5;

const int CAS_TEXT_TOP = 68;
const int CAS_TEXT_LEFT = 259;
const int CAS_TEXT_HEIGHT = 20;

const int TASK_TEXT_LEFT = 2;
const int TASK_TEXT_HEIGHT = 30;

const int STOP_BTN_TOP = 261;
const int STOP_BTN_LEFT = 2;
const int STOP_BTN_BOTTOM = 314;
const int STOP_BTN_RIGHT = 134;

const int PAUSE_BTN_TOP = STOP_BTN_TOP;
const int PAUSE_BTN_LEFT = 146;
const int PAUSE_BTN_BOTTOM = STOP_BTN_BOTTOM;
const int PAUSE_BTN_RIGHT = 248;

const int DOWN_BTN_TOP = 282;
const int DOWN_BTN_LEFT = CAS_FRAME_LEFT;
const int DOWN_BTN_BOTTOM = 319;
const int DOWN_BTN_RIGHT = 366;

const int NEXT_BTN_TOP = DOWN_BTN_TOP;
const int NEXT_BTN_LEFT = 370;
const int NEXT_BTN_BOTTOM = DOWN_BTN_BOTTOM;
const int NEXT_BTN_RIGHT = MAX_WIDTH;

const int STOP_TEXT_X_OFFSET = 22;
const int STOP_TEXT_Y_OFFSET = 12;
const int STOP_TEXT_SIZE = 4;
const int STOP_LINE_WIDTH = 3;

const int PAUSE_TEXT_X_OFFSET = 35;
const int PAUSE_TEXT_Y_OFFSET = STOP_TEXT_Y_OFFSET;
const int PAUSE_TEXT_SIZE = STOP_TEXT_SIZE;
const int PAUSE_LINE_WIDTH = STOP_LINE_WIDTH;

const int DOWN_TEXT_X_OFFSET = 23;
const int DOWN_TEXT_Y_OFFSET = 9;
const int DOWN_TEXT_SIZE = 3;
const int DOWN_LINE_WIDTH = STOP_LINE_WIDTH;

const int NEXT_TEXT_X_OFFSET = DOWN_TEXT_X_OFFSET;
const int NEXT_TEXT_Y_OFFSET = DOWN_TEXT_Y_OFFSET;
const int NEXT_TEXT_SIZE = DOWN_TEXT_SIZE;
const int NEXT_LINE_WIDTH = STOP_LINE_WIDTH;

const char DEGREE_SIGN = 247;
const char PAUSE_SIGN = 0xDC;
const char THETA_SIGN = 232;
const int MAX_TASKS = 6;

typedef enum 
{
  ST_NO_COMM,
  ST_GOOD,
  ST_PAUSED,
  ST_LOST,
  ST_DOCKED,
  ST_CHARGED
} statusEnum;

Adafruit_HX8357 gTft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

bool gReceivedComm = false;
statusEnum gTheStatus = ST_GOOD;
char gTasks[MAX_TASKS][30];

uint16_t statusLineBackground(statusEnum theStatus)
{
  switch(theStatus)
  {
    case (ST_GOOD):
      return HX8357_GREEN;
    case (ST_PAUSED):
      return HX8357_YELLOW;
    case (ST_LOST):
      return HX8357_RED;
    case (ST_DOCKED):
      return HX8357_BLUE;
    case (ST_CHARGED):
      return HX8357_CYAN;
    default:
      return HX8357_MAGENTA;
  }
}

uint16_t statusLineText(statusEnum theStatus)
{
  switch(theStatus)
  {
    case (ST_GOOD):
      return HX8357_BLACK;
    case (ST_PAUSED):
      return HX8357_BLACK;
    case (ST_LOST):
      return HX8357_WHITE;
    case (ST_DOCKED):
      return HX8357_WHITE;
    case (ST_CHARGED):
      return HX8357_BLACK;
    default:
      return HX8357_BLACK;
  }
}

void setup() {
  Serial.begin(115200);

  sprintf(gTasks[0],"Moved to X,Y");
  sprintf(gTasks[1],"Creating Map");
  sprintf(gTasks[2],"Saving Map");
  sprintf(gTasks[3],"Moved to X,Y");
  sprintf(gTasks[4],"Line 5");
  sprintf(gTasks[5],"Line 6");


  gTft.begin();
  gTft.setRotation(1);
  gTft.fillScreen(HX8357_BLACK);
  drawCASFrame();
  displayCAS();
  displayTasks();
  drawButtons();
  displayStatus(true,true);
}

void loop(void) {
//  displayStatus(true,true);
}

void drawButtons(void)
{
  for(int i=0;i<STOP_LINE_WIDTH;i++)
  {
    gTft.drawLine(STOP_BTN_LEFT,    STOP_BTN_TOP+i,    STOP_BTN_RIGHT,   STOP_BTN_TOP+i,    HX8357_RED);
    gTft.drawLine(STOP_BTN_LEFT,    STOP_BTN_BOTTOM-i, STOP_BTN_RIGHT,   STOP_BTN_BOTTOM-i, HX8357_RED);
    gTft.drawLine(STOP_BTN_LEFT+i,  STOP_BTN_TOP,      STOP_BTN_LEFT+i,  STOP_BTN_BOTTOM,   HX8357_RED);
    gTft.drawLine(STOP_BTN_RIGHT-i, STOP_BTN_TOP,      STOP_BTN_RIGHT-i, STOP_BTN_BOTTOM,   HX8357_RED);
  }
  gTft.setTextColor(HX8357_RED);
  gTft.setTextSize(STOP_TEXT_SIZE);
  gTft.setCursor(STOP_BTN_LEFT+STOP_TEXT_X_OFFSET, STOP_BTN_TOP+STOP_TEXT_Y_OFFSET);
  gTft.println("STOP");

  for(int i=0;i<PAUSE_TEXT_SIZE;i++)
  {
    gTft.drawLine(PAUSE_BTN_LEFT,    PAUSE_BTN_TOP+i,    PAUSE_BTN_RIGHT,   PAUSE_BTN_TOP+i,    HX8357_BLUE);
    gTft.drawLine(PAUSE_BTN_LEFT,    PAUSE_BTN_BOTTOM-i, PAUSE_BTN_RIGHT,   PAUSE_BTN_BOTTOM-i, HX8357_BLUE);
    gTft.drawLine(PAUSE_BTN_LEFT+i,  PAUSE_BTN_TOP,      PAUSE_BTN_LEFT+i,  PAUSE_BTN_BOTTOM,   HX8357_BLUE);
    gTft.drawLine(PAUSE_BTN_RIGHT-i, PAUSE_BTN_TOP,      PAUSE_BTN_RIGHT-i, PAUSE_BTN_BOTTOM,   HX8357_BLUE);
  }
  char stringx[5];
  sprintf(stringx,"%c%c",PAUSE_SIGN,PAUSE_SIGN);
  gTft.setTextColor(HX8357_BLUE);
  gTft.setTextSize(PAUSE_TEXT_SIZE);
  gTft.setCursor(PAUSE_BTN_LEFT+PAUSE_TEXT_X_OFFSET, PAUSE_BTN_TOP+PAUSE_TEXT_Y_OFFSET);
  gTft.println(stringx);

  for(int i=0;i<DOWN_LINE_WIDTH;i++)
  {
    gTft.drawLine(DOWN_BTN_LEFT,    DOWN_BTN_TOP+i,    DOWN_BTN_RIGHT,   DOWN_BTN_TOP+i,    HX8357_CYAN);
    gTft.drawLine(DOWN_BTN_LEFT,    DOWN_BTN_BOTTOM-i, DOWN_BTN_RIGHT,   DOWN_BTN_BOTTOM-i, HX8357_CYAN);
    gTft.drawLine(DOWN_BTN_LEFT+i,  DOWN_BTN_TOP,      DOWN_BTN_LEFT+i,  DOWN_BTN_BOTTOM,   HX8357_CYAN);
    gTft.drawLine(DOWN_BTN_RIGHT-i, DOWN_BTN_TOP,      DOWN_BTN_RIGHT-i, DOWN_BTN_BOTTOM,   HX8357_CYAN);
  }
  gTft.setTextColor(HX8357_CYAN);
  gTft.setTextSize(DOWN_TEXT_SIZE);
  gTft.setCursor(DOWN_BTN_LEFT+DOWN_TEXT_X_OFFSET, DOWN_BTN_TOP+DOWN_TEXT_Y_OFFSET);
  gTft.println("down");

  for(int i=0;i<NEXT_LINE_WIDTH;i++)
  {
    gTft.drawLine(NEXT_BTN_LEFT,    NEXT_BTN_TOP+i,    NEXT_BTN_RIGHT,   NEXT_BTN_TOP+i,    HX8357_CYAN);
    gTft.drawLine(NEXT_BTN_LEFT,    NEXT_BTN_BOTTOM-i, NEXT_BTN_RIGHT,   NEXT_BTN_BOTTOM-i, HX8357_CYAN);
    gTft.drawLine(NEXT_BTN_LEFT+i,  NEXT_BTN_TOP,      NEXT_BTN_LEFT+i,  NEXT_BTN_BOTTOM,   HX8357_CYAN);
    gTft.drawLine(NEXT_BTN_RIGHT-i, NEXT_BTN_TOP,      NEXT_BTN_RIGHT-i, NEXT_BTN_BOTTOM,   HX8357_CYAN);
  }
  gTft.setTextColor(HX8357_CYAN);
  gTft.setTextSize(NEXT_TEXT_SIZE);
  gTft.setCursor(NEXT_BTN_LEFT+NEXT_TEXT_X_OFFSET, NEXT_BTN_TOP+NEXT_TEXT_Y_OFFSET);
  gTft.println("next");
}


void displayTasks(void)
{
  for(int i=0;i<MAX_TASKS;i++)
  {
    switch(i)
    {
      case(0):
        gTft.setTextColor(HX8357_GRAY);
        break;
      case(1):
        gTft.setTextColor(HX8357_WHITE);
        break;
      default:
        gTft.setTextColor(HX8357_CYAN);
        break;
    }
    gTft.setTextSize(3);
    gTft.setCursor(TASK_TEXT_LEFT, CAS_TEXT_TOP+(TASK_TEXT_HEIGHT*i));
    gTft.println(gTasks[i]);
  }
}

void drawCASFrame(void)
{
  for(int i=0;i<CAS_FRAME_SIZE;i++)
  {
    gTft.drawLine(CAS_FRAME_LEFT,    CAS_FRAME_TOP+i,    CAS_FRAME_RIGHT,   CAS_FRAME_TOP+i,    HX8357_CYAN);
    gTft.drawLine(CAS_FRAME_LEFT,    CAS_FRAME_BOTTOM-i, CAS_FRAME_RIGHT,   CAS_FRAME_BOTTOM-i, HX8357_CYAN);
    gTft.drawLine(CAS_FRAME_LEFT+i,  CAS_FRAME_TOP,      CAS_FRAME_LEFT+i,  CAS_FRAME_BOTTOM,   HX8357_CYAN);
    gTft.drawLine(CAS_FRAME_RIGHT-i, CAS_FRAME_TOP,      CAS_FRAME_RIGHT-i, CAS_FRAME_BOTTOM,   HX8357_CYAN);
  }
}

void displayCAS(void)
{
  for(int i=0;i<10;i++)
  {
    char stringx[20];
    sprintf(stringx,"1234567890123456 %d",i);    
    gTft.setTextColor(HX8357_RED);
    gTft.setTextSize(2);
    gTft.setCursor(CAS_TEXT_LEFT, CAS_TEXT_TOP+(CAS_TEXT_HEIGHT*i));
    gTft.println(stringx);
  }
}

void displayStatus(bool topLine, bool bottomLine)
{
  if(true == topLine)
  {
    char string1[23];

    sprintf(string1,"123.123x 123.123y 123%c",THETA_SIGN);
    
    gTft.fillRect(0, STATUS_TOP_LINE_Y, MAX_WIDTH, 
        STATUS_MAX_HEIGHT+STATUS_TOP_LINE_Y, 
        statusLineBackground(gTheStatus));
    gTft.setTextColor(statusLineText(gTheStatus));
    gTft.setTextSize(STATUS_TEXT_SIZE);
    gTft.setCursor(STATUS_TEXT_X_OFFSET, STATUS_TOP_LINE_Y+STATUS_TEXT_Y_OFFSET);
    gTft.println(string1);
  }

  if(true == bottomLine)
  {
    char string1[40];
    sprintf(string1,"  47%c 40'  9.89\" N -122%c 07' 25.95\" W",DEGREE_SIGN,DEGREE_SIGN);

    gTft.fillRect(0, STATUS_BOTTOM_LINE_Y, MAX_WIDTH, 
        STATUS_MAX_HEIGHT, 
        statusLineBackground(gTheStatus));
    gTft.setTextColor(statusLineText(gTheStatus));
    gTft.setTextSize(STATUS2_TEXT_SIZE);
    gTft.setCursor(STATUS_TEXT_X_OFFSET, STATUS_BOTTOM_LINE_Y+STATUS2_TEXT_Y_OFFSET);
    gTft.println(string1);
  }
  
}

void displayHeart()
{
  
}
