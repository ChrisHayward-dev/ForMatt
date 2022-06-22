//
//    FILE: PCF8575_test.ino
//  AUTHOR: Rob Tillaart
//    DATE: 2020-07-20
// PUPROSE: test PCF8575 library
//     URL: https://github.com/RobTillaart/PCF8575


#include "PCF8575.h"

PCF8575 PCF(0x20);


void setup()
{
  Serial.begin(115200);
  while (!Serial)yield();
  Serial.println(__FILE__);
  Serial.print("PCF8575_test version: ");
  Serial.println(PCF8575_LIB_VERSION);

  PCF.begin();

  uint16_t x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
  delay(1000);
}


void loop()
{
  uint32_t ctime = millis();
  Serial.println("HLT");
  while ((millis() - ctime) < 500 && Serial.available() == 0 ) yield();
  if (Serial.available() > 0) {
    switch (Serial.read())
    {
      case 'H': doHigh(); break;
      case 'L': doLow(); break;
      case 'T': doToggle(); break;
    }
  }
  PCF.pinMode(1,INPUT);
  uint16_t x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void doHigh()
{
  PCF.write(4, HIGH);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void doLow()
{
  PCF.write(4, LOW);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void doToggle()
{
  PCF.toggle(4);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void printHex(uint16_t x)
{
  if (x < 0x1000) Serial.print('0');
  if (x < 0x100)  Serial.print('0');
  if (x < 0x10)   Serial.print('0');
  Serial.println(x, HEX);
}


// -- END OF FILE --
