/* Seismic Source controller (ie pickel controller)
 * cth 7/11/2022
 */
#include <PCF8575.h>
PCF8575 PCF(0x20);
#define REDLED    13
#define GREENLED  8

void errorStop(int errornum) {
  while(true) {
    for(int k=0;k<errornum;k++) {
      digitalWrite(REDLED,HIGH);
      delay(250);
      digitalWrite(REDLED,LOW);
      delay(250);
    }
    delay(1000);
  }
}
void setup() {
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  Serial.begin(9600);
  while(!Serial) {
    if((millis()&0x100) == 0x100) {
      digitalWrite(REDLED,!digitalRead(REDLED));
    }
    yield();
  }

  delay(5000);
  digitalWrite(GREENLED,HIGH);
  digitalWrite(REDLED,LOW);
  Serial.println("Starting Pickel Controller");
  Serial.println(__FILE__);
  if(!PCF.begin()) {
    Serial.println("Unable to init PCF");
    errorStop(1);
  }
  if(!PCF.isConnected()) {
    Serial.println("PCF is not connected!");
    errorStop(2);
  }
  Serial.println("PCF connected");
}

void loop() {
  Serial.println("Waiting for code");
  delay(2000);
  // put your main code here, to run repeatedly:

}
