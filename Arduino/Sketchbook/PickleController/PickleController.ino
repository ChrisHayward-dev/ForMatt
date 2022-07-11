/* Seismic Source controller (ie pickel controller)
   cth 7/11/2022
*/
#include <PCF8575.h>
PCF8575 PCF(0x20);
#define REDLED    13
#define GREENLED  8

#define SWARM      0
#define SWAUTO     1
#define SWSEQ      2
#define SWFILL     3
#define SWPURGE    4
#define SWFIRE     5
#define ARMLED     8
#define AUTOLED    9
#define ARMRELAY   10
#define FILLRELAY  11
#define PURGERELAY 12
#define PUMPRELAY  13
#define FIRERELAY  14

#define SWARM_MASK      0x1
#define SWAUTO_MASK     0x2
#define SWSEQ_MASK      0x4
#define SWFILL_MASK     0x8
#define SWPURGE_MASK    0x10
#define SWFIRE_MASK     0x20
#define ARMLED_MASK     0x0100
#define AUTOLED_MASK    0x0200
#define ARMRELAY_MASK   0x0400
#define FILLRELAY_MASK  0x0800
#define PURGERELAY_MASK 0x1000
#define PUMPRELAY_MASK  0x2000
#define FIRERELAY_MASK  0x4000

#define SWALL_MASK      (SWARM_MASK|SWAUTO_MASK|SWSEQ_MASK|SWFILL_MASK|SWPURGE_MASK|SWFIRE_MASK)
#define RELAYALL_MASK   (ARMLED_MASK|AUTOLED_MASK|ARMRELAY_MASK|FILLRELAY_MASK|PURGERELAY_MASK|PUMPRELAY_MASK|FIRERELAY_MASK)

struct {
  uint16_t  swmask;
  uint16_t  swvalue;
  uint16_t  relaymask;
  uint16_t  relayvalue;
} m[] = {
  SWALL_MASK, SWALL_MASK, RELAYALL_MASK, RELAYALL_MASK,
  SWALL_MASK, SWALL_MASK & ~SWARM_MASK, ARMRELAY_MASK, ARMRELAY_MASK
};

#define NSTATES   (sizeof(m)/(4*sizeof(uint16_t)))

void errorStop(int errornum) {
  while (true) {
    for (int k = 0; k < errornum; k++) {
      digitalWrite(REDLED, HIGH);
      delay(250);
      digitalWrite(REDLED, LOW);
      delay(250);
    }
    delay(1000);
  }
}

void safeswitch(int sw, int led) {
  // check ARM switch is in safe position
  while (PCF.read(sw) == LOW) {
    PCF.write(led, HIGH);
    delay(500);
    PCF.write(led, LOW);
    delay(500);
  }
  PCF.write(led, HIGH);
}
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  Serial.begin(9600);
  while (!Serial) {
    if ((millis() & 0x100) == 0x100) {
      digitalWrite(REDLED, !digitalRead(REDLED));
    }
    yield();
  }

  delay(5000);
  digitalWrite(GREENLED, HIGH);
  digitalWrite(REDLED, LOW);
  Serial.println("Starting Pickel Controller");
  Serial.println(__FILE__);
  Serial.print("Found "); Serial.print(NSTATES); Serial.println(" states");
  if (!PCF.begin()) {
    Serial.println("Unable to init PCF");
    errorStop(1);
  }
  if (!PCF.isConnected()) {
    Serial.println("PCF is not connected!");
    errorStop(2);
  }
  Serial.println("PCF connected");

  safeswitch(SWARM, ARMLED);
  safeswitch(SWAUTO, AUTOLED);

}

void loop() {
  uint16_t switches = PCF.readButton16(SWALL_MASK);
  Serial.print("switches: "); Serial.println(switches, HEX);
  for (int state = 0; state < NSTATES; state++) {
    Serial.print("\tChecking: "); Serial.print(m[state].swmask, HEX);
    Serial.print(" for "); Serial.println(m[state].swvalue, HEX);
    if ((switches & m[state].swmask) == m[state].swvalue) {
      Serial.print("State: "); Serial.println(state);
      Serial.print("Relay Mask: ");Serial.println(m[state].relaymask,HEX);
      Serial.print("Relay Set:  ");Serial.println(m[state].relayvalue,HEX);
      uint16_t relaystate = PCF.readButton16(RELAYALL_MASK);
      uint16_t newstate   = (relaystate & ~m[state].relaymask) | m[state].relayvalue;
      PCF.write16(newstate);
      Serial.print("Set Relays from "); Serial.print(relaystate, HEX);
      Serial.print(" To "); Serial.println(newstate, HEX);
    }
  }
  delay(500);
  yield();
}
