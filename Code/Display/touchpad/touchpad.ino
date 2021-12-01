#include <SPI.h>
#include "TouchScreen.h"

#define YP A3   // A2 A2
#define XM A2   // A3 A3
#define YM 11   // 10 11
#define XP 10   // 11 10

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

void setup(void) {
  while (!Serial);     // used for leonardo debugging
 
  Serial.begin(115200);
  Serial.println(F("Touch Pad"));
}

void loop()
{
  TSPoint p = ts.getPoint();
  if(p.z > 10)
  {
    Serial.print("X = "); Serial.print(p.x);
    Serial.print("\tY = "); Serial.print(p.y);
    Serial.print("\tPressure = "); Serial.println(p.z);  
  }
}
