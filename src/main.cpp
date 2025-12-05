#include <Arduino.h>
#include "CircularDebounceBuffer.h"
#include <stdarg.h>


static constexpr uint8_t BUTTON_PIN = 2; // Arduino Nano pin interrupts (2,3)

CircularDebounceBuffer btn(1, BUTTON_PIN); // pin 2, active-low
 
/**
 * @brief 
 * 
 * @param fmt 
 * @param ... 
 */
void serialPrintf(const char* fmt, ...)
{
  char buf[64];              // adjust size, beware of RAM limits on AVR
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}



void setup() 
{
	Serial.begin(115200);
  while(!Serial);

    
  btn.addCallback([] { Serial.println("\nSHORT PRESS!!!\n"); });
  btn.enableLongPress(2000);
  btn.addLongPressCallback([](uint32_t ms) { serialPrintf("\nLONG PRESS!!! : %lums\n", ms); });
  btn.enableDoublePress(400);
  btn.addDoublePressCallback([] { Serial.println("\nDOUBLE PRESS!!!\n"); });  
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), [] { btn.startDebounce(); }, FALLING);
 
 
 }
 
void loop()
{
  btn.update();
}

