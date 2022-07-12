/* Seismic Source controller (ie pickel controller)
   cth 7/11/2022
*/
#include <PCF8575.h>

#define NSTACK 5
#define FILLTIME  15000
#define PURGETIME 500
#define FIRETIME  500
#define DEADTIME  2000

PCF8575 PCF(0x20);
#define REDLED 13
#define GREENLED 8

#define SWARM 0
#define SWAUTO 1
#define SWSEQ 2
#define SWFILL 3
#define SWPURGE 4
#define SWFIRE 5
#define ARMLED 8
#define AUTOLED 9
#define ARMRELAY 10
#define FILLRELAY 11
#define PURGERELAY 12
#define PUMPRELAY 13
#define FIRERELAY 14

#define SWARM_MASK (1 << SWARM)
#define SWAUTO_MASK (1 << SWAUTO)
#define SWSEQ_MASK (1 << SWSEQ)
#define SWFILL_MASK (1 << SWFILL)
#define SWPURGE_MASK (1 << SWPURGE)
#define SWFIRE_MASK (1 << SWFIRE)
#define ARMLED_MASK (1 << ARMLED)
#define AUTOLED_MASK (1 << AUTOLED)
#define ARMRELAY_MASK (1 << ARMRELAY)
#define FILLRELAY_MASK (1 << FILLRELAY)
#define PURGERELAY_MASK (1 << PURGERELAY)
#define PUMPRELAY_MASK (1 << PUMPRELAY)
#define FIRERELAY_MASK (1 << FIRERELAY)

#define SWALL_MASK (SWARM_MASK | SWAUTO_MASK | SWSEQ_MASK | SWFILL_MASK | SWPURGE_MASK | SWFIRE_MASK)
#define RELAYALL_MASK (ARMLED_MASK | AUTOLED_MASK | ARMRELAY_MASK | FILLRELAY_MASK | PURGERELAY_MASK | PUMPRELAY_MASK | FIRERELAY_MASK)
#define RELAYWRK_MASK  (RELAYALL_MASK & ~ARMLED_MASK & ~AUTOLED_MASK)
struct {
  uint16_t swmask;      // mask of switches to consider
  uint16_t swvalue;     // value of switches to match
  uint16_t relaymask;   // mask of relays to set
  uint16_t relayvalue;  // value of relays to set
} m[] = {
  SWALL_MASK , 0, RELAYALL_MASK , 0,  //all off
  SWALL_MASK, SWARM_MASK, RELAYWRK_MASK, ARMRELAY_MASK,
  SWALL_MASK & ~SWARM_MASK, SWFILL_MASK, RELAYWRK_MASK & ~ARMRELAY_MASK, FILLRELAY_MASK | PUMPRELAY_MASK,  // manual fill switch
  SWALL_MASK & ~SWARM_MASK, SWPURGE_MASK, RELAYWRK_MASK & ~ARMRELAY_MASK, PURGERELAY_MASK,                    // manual purge switch
  SWALL_MASK, SWFIRE_MASK | SWARM_MASK, RELAYWRK_MASK, FIRERELAY_MASK|ARMRELAY_MASK,                       // manual fire switch
  SWPURGE_MASK|SWAUTO_MASK, SWPURGE_MASK, ARMLED_MASK, ARMLED_MASK,
  SWFILL_MASK|SWAUTO_MASK,  SWFILL_MASK, ARMLED_MASK, ARMLED_MASK,
  SWFIRE_MASK|SWAUTO_MASK,  SWFIRE_MASK, ARMLED_MASK, ARMLED_MASK,
  SWFIRE_MASK|SWFILL_MASK|SWPURGE_MASK,0,ARMLED_MASK,0,
  SWARM_MASK, 0, ARMRELAY_MASK, 0,                                                           // arm switch in safe position
  SWARM_MASK, SWARM_MASK, ARMRELAY_MASK, ARMRELAY_MASK,                                      // arm switch in arm position
  SWALL_MASK, SWAUTO_MASK, RELAYWRK_MASK & ~ARMRELAY_MASK, 0,                                // all relays off upon auto
  SWALL_MASK & ~SWARM_MASK, SWAUTO_MASK | SWSEQ_MASK, RELAYWRK_MASK & ~ARMRELAY_MASK, 0                    // set relays off to let process run in auto - this MUST be the last state
};

#define NSTATES (sizeof(m) / (4 * sizeof(uint16_t)))

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

/* set16 sets only the mask/value and leaves all switches unset and other relays
    in their current condition */
void set16(uint16_t mask, uint16_t value) {
  uint16_t currentValue = PCF.read16();
  uint16_t setValue = SWALL_MASK | (~mask & currentValue) | (mask&value);
  PCF.write16(setValue);
  Serial.print("Set Relays from ");
  Serial.print(currentValue & RELAYALL_MASK, HEX);
  Serial.print(" To ");
  Serial.println((setValue & RELAYALL_MASK), HEX);
}
bool readsw(uint16_t mask, uint16_t value) {
  uint16_t switches = PCF.readButton16(SWALL_MASK);
  return ((switches & mask) == value);
}

/* save switch checks that switch sw is in the OFF (safe) position
   if not, it blinks the switch LED until it is put in the safe position
*/
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
  uint32_t startTime = millis();
  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  Serial.begin(9600);
  while (!Serial) {
    if ((millis() & 0x100) == 0x100) {
      digitalWrite(REDLED, !digitalRead(REDLED));
    }
    yield();
    if(millis()-startTime > 10000) break;
  }

  delay(1000);
  digitalWrite(GREENLED, HIGH);
  digitalWrite(REDLED, LOW);
  Serial.println("Starting Pickel Controller");
  Serial.println(__FILE__);
  Serial.print("Found ");
  Serial.print(NSTATES);
  Serial.println(" states");
  if (!PCF.begin()) {
    Serial.println("Unable to init PCF");
    errorStop(1);
  }
  if (!PCF.isConnected()) {
    Serial.println("PCF is not connected!");
    errorStop(2);
  }
  Serial.println("PCF connected");

  // process the state table for later use
  /* The value entries need to be inverted since activated is LOW not high */
  for (uint k = 0; k < NSTATES; k++) {
    m[k].swvalue = (~m[k].swvalue) & m[k].swmask;
    m[k].relayvalue = (~m[k].relayvalue) & m[k].relaymask;
  };
  Serial.println("Processed state table");
  safeswitch(SWARM, ARMLED);
  safeswitch(SWAUTO, AUTOLED);
}

void loop() {
  static uint state = 0;
  bool validState;
  digitalWrite(REDLED,HIGH);
  for (state = 0; state < NSTATES; state++) {
    validState = false;
    Serial.print("\tChecking: ");
    Serial.print(m[state].swmask, HEX);
    Serial.print(" for ");
    Serial.println(m[state].swvalue, HEX);
    if (readsw(m[state].swmask, m[state].swvalue)) {
      Serial.print("State: ");
      Serial.println(state);
      Serial.print("Relay Mask: ");
      Serial.println(m[state].relaymask, HEX);
      Serial.print("Relay Set:  ");
      Serial.println(m[state].relayvalue, HEX);
      set16(m[state].relaymask, m[state].relayvalue);
      validState = true;
    }
  }
  digitalWrite(REDLED,LOW);
  /* if we are in auto and sequencing has been started
     then initialize with a purge/fire cycle and then
     commence fill purge fire cycles until we either have
     a different switch state or have NSTACK fires
  */
  digitalWrite(GREENLED,HIGH);
  if (validState && (state == NSTATES)) {
    Serial.println("Auto Mode");
    set16(RELAYALL_MASK & ~ARMRELAY_MASK, PURGERELAY_MASK);
    delay(250);
    if (PCF.read(SWARM) == LOW) {
      PCF.write(FIRERELAY, LOW);
      delay(100);
      PCF.write(FIRERELAY, HIGH);
    }

    for (int k = 0; k < NSTACK; k++) {
      Serial.println("Fill");
      set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~(FILLRELAY_MASK | PUMPRELAY_MASK));               //start fill w/ purge closed
      delay(FILLTIME);                                                                           //fill delay
      Serial.println("Purge");
      set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~(FILLRELAY_MASK | PUMPRELAY_MASK | PURGERELAY_MASK)); //open purge
      delay(PURGETIME);                                                                           //purge delay
      Serial.println("Wait");
      set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~0);                                             //all closed
      delay(DEADTIME);                                                                            //close delay
      if (PCF.read(SWARM) == LOW) {
        Serial.println("Fire");
        set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~FIRERELAY_MASK);
      }
      Serial.println("Wait");
      delay(FIRETIME);
      set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~0);
      Serial.println("--Finished");
      delay(DEADTIME);
    }
    Serial.println("Finished Stack");
  }
  digitalWrite(GREENLED,LOW);
  yield();
}
