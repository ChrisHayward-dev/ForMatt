// Seismic Source Controller
// (Pickle controller)

#include <PCF8575.h>

/* All the control is done via a PCF8575 I2C port expander.  This allows us to
 *  use the smaller Arduino Feather or Trinkets rather than a Metro board
 */
PCF8575 PCF(0x20);

#define SW_ARM    0
#define SW_SEQ    1
#define SW_START  2
#define SW_FILL   3
#define SW_PURGE  4
#define SW_FIRE   5

#define RL_ARM      10
#define RL_FILL     11
#define RL_PURGE    12
#define RL_PUMP     13
#define RL_FIRE     14
#define RL_AUTO_LED 9
#define RL_ARM_LED  8


void setup()
{
  uint8_t pin;
  Serial.begin(115200);
  while (!Serial)yield();

  PCF.begin();
  PCF.write16(0xFFFF);          // set all high for default

  for(pin=0;pin<6;pin++) {
    while(PCF.read(pin) == HIGH) yield();
    Serial.print("Got pin ");Serial.println(pin);
  }
  Serial.println("Looks like all the switches work!");

  delay(2000);
  Serial.println("Arm LED");PCF.write(RL_ARM_LED,LOW);
  delay(2000);
  Serial.println("Auto LED");PCF.write(RL_AUTO_LED,LOW);
  delay(2000);
  Serial.println("Arm Relay");PCF.write(RL_ARM,LOW);
  delay(2000);
  Serial.println("Fill Relay");PCF.write(RL_FILL,LOW);
  delay(2000);
  Serial.println("Purge Relay");PCF.write(RL_PURGE,LOW);
  delay(2000);
  Serial.println("Pump Relay");PCF.write(RL_PUMP,LOW);
  delay(2000);
  Serial.println("Fire Relay");PCF.write(RL_FIRE,LOW);
  delay(2000);
  PCF.write16(0xFFFF);
  
  uint16_t x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
  delay(2000);
}


void loop()
{
  static int pin = 0;
  uint32_t ctime = millis();
  uint16_t x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
  delay(2000);
//  pin = (++pin)%8 + 8;
//  PCF.write(pin,LOW);
//  delay(500);
//  PCF.write(pin,HIGH);
//  delay(500);
}


void doHigh()
{

}


void doLow()
{

  //PCF.write(P17, LOW);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void doToggle()
{

  //PCF.toggle(P17);
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
