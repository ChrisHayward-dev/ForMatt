/* Seismic Source controller (ie pickel controller)
   cth 7/11/2022
*/
#include <PCF8575.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/* Pickle options */
#define USE_CONSOLE 1

#define NSTACK 5
#define FILLTIME 15000
#define PURGETIME 500
#define FIRETIME 500
#define DEADTIME 2000

// Arduino pin definitions
#define SWUP 5
#define SWDOWN 6
#define GREENLED 8
#define SWPLUS 9
#define SWMINUS 10
#define TRIGOUT 11
#define PCF_INTERRUPT 12
#define REDLED 13
#define SPARK A2
#define PHOTO A1

// PCF pin defintions
#define SWARM 0
#define SWAUTO 1
#define SWSEQ 2
#define SWFILL 3
#define SWPURGE 4
#define SWFIRE 5
#define SWAIRFILL 6
#define SWAIRPURGE 7

#define ARMLED 8
#define AUTOLED 9
#define ARMRELAY 10
#define FILLRELAY 11
#define PURGERELAY 12
#define PUMPRELAY 13
#define FIRERELAY 14
#define VALVERELAY 15

#define SWARM_MASK (1 << SWARM)
#define SWAUTO_MASK (1 << SWAUTO)
#define SWSEQ_MASK (1 << SWSEQ)
#define SWFILL_MASK (1 << SWFILL)
#define SWPURGE_MASK (1 << SWPURGE)
#define SWFIRE_MASK (1 << SWFIRE)
#define SWAIRPURGE_MASK (1 << SWAIRPURGE)
#define SWAIRFILL_MASK (1 << SWAIRFILL)

#define ARMLED_MASK (1 << ARMLED)
#define AUTOLED_MASK (1 << AUTOLED)
#define ARMRELAY_MASK (1 << ARMRELAY)
#define FILLRELAY_MASK (1 << FILLRELAY)
#define PURGERELAY_MASK (1 << PURGERELAY)
#define PUMPRELAY_MASK (1 << PUMPRELAY)
#define FIRERELAY_MASK (1 << FIRERELAY)

#define SWALL_MASK 0x00FF
#define SWALL_OFF  0x00FF
#define SWBUTTONS_MASK  (SWFILL_MASK|SWPURGE_MASK|SWFIRE_MASK|SWSEQ_MASK|SWAIRPURGE_MASK|SWAIRFILL_MASK)
#define RELAYALL_MASK 0xFF00
#define RELAYALL_OFF  0xFF00
#define RELAYCONTPL_MASK (RELAYALL_MASK & ~ARMLED_MASK & ~AUTOLED_MASK)

// Library definitions
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET 4         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PCF8575 PCF(0x20);


bool dummySetting();

struct {
  uint16_t swmask;    // mask of switches to consider
  uint16_t swvalue;   // value of switches to match
  bool (*routine)();  // routine to fire upon this switch setting
} m[] = {
  SWALL_MASK, 0, dummySetting  //all off - initialize to all relays off
};

#define NSTATES (sizeof(m) / (sizeof(m[0])))

bool dummySetting() {
  return (true);
}

volatile bool swChange = false;
void switchChange() {
  swChange = true;
}

void errorStop(int errornum) {
  // Flash a simple code on the red led and halt
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


void set16(uint16_t mask, uint16_t value) {
  // set16 sets only the mask/value and leaves all switches unset and other relays
  //   in their current condition
  uint16_t currentValue = PCF.read16();
  uint16_t setValue = SWALL_MASK | (~mask & currentValue) | (mask & value);
  PCF.write16(setValue);
  //  Serial.print("Set Relays from ");
  //  Serial.print(currentValue & RELAYALL_MASK, HEX);
  //  Serial.print(" To ");
  //  Serial.println((setValue & RELAYALL_MASK), HEX);
}
bool readsw(uint16_t mask, uint16_t value) {
  uint16_t switches = PCF.readButton16(SWALL_MASK);
  return ((switches & mask) == value);
}


void safeswitch(int sw, int led) {
  /* save switch checks that switch sw is in the OFF (safe) position
    if not, it blinks the switch LED until it is put in the safe position
  */
  while (PCF.read(sw) == LOW) {
    PCF.write(led, HIGH);
    delay(500);
    PCF.write(led, LOW);
    delay(500);
  }
  PCF.write(led, HIGH);
}


bool fire() {
  uint32_t startTime = millis();
  int32_t   val = 0;
  int       count = 0;
  if (PCF.read(SWARM) == HIGH) return (true);
  Serial.println("Firing!");
  set16(RELAYALL_MASK & ~ARMRELAY_MASK & ~AUTOLED_MASK, ~FIRERELAY_MASK);
  delay(20);
  while ((millis() - startTime) < FIRETIME) {
    val = max(analogRead(SPARK), val);
    count++;
    if (val > 50) {
      set16(RELAYALL_MASK & ~ARMRELAY_MASK & ~AUTOLED_MASK, ~0);
      digitalWrite(REDLED, HIGH);
      digitalWrite(GREENLED, HIGH);
      delay(100);
      digitalWrite(REDLED, LOW);
      return (true);
      break;
    }
  }
  set16(RELAYALL_MASK & ~ARMRELAY_MASK & ~AUTOLED_MASK, ~0);
  Serial.println("Misfire!");
  return (false);
}
void setup() {
  uint32_t startTime = millis();
  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(SWUP, INPUT_PULLUP);
  pinMode(SWDOWN, INPUT_PULLUP);
  pinMode(SWMINUS, INPUT_PULLUP);
  pinMode(SWPLUS, INPUT_PULLUP);
  pinMode(TRIGOUT, OUTPUT);
  pinMode(PCF_INTERRUPT, INPUT_PULLUP);
  pinMode(SPARK, INPUT_PULLDOWN);

  digitalWrite(REDLED, HIGH);
  Serial.begin(9600);
#if USE_CONSOLE == 1
  while (!Serial) {
    if ((millis() & 0x100) == 0x100) {
      digitalWrite(REDLED, !digitalRead(REDLED));
    }
    yield();
    if (millis() - startTime > 10000) break;
  }
#endif
  Serial.println("Starting Pickel Controller");
  Serial.println(__FILE__);
  Serial.print("Found ");
  Serial.print(NSTATES);
  Serial.println(" states");

  if (!PCF.begin()) {  // setup the PCF I2C port expander
    Serial.println("Unable to init PCF");
    errorStop(1);
  }
  if (!PCF.isConnected()) {
    Serial.println("PCF is not connected!");
    errorStop(2);
  }
  set16(RELAYALL_MASK, RELAYALL_OFF);

  Serial.println("PCF connected");
  { // start 1306 display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {  //setup the SSD1306 display
      Serial.println(F("SSD1306 allocation failed"));
      errorStop(3);
    }
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.println("Pickle");
    display.display();
    delay(5000);
  }
  safeswitch(SWARM, ARMLED);
  safeswitch(SWAUTO, AUTOLED);
  attachInterrupt(digitalPinToInterrupt(PCF_INTERRUPT), switchChange, FALLING); // setup the switch detect interrupt
  digitalWrite(GREENLED, HIGH);
  digitalWrite(REDLED, LOW);
}

enum pState { PROG, MANUAL, AUTO, STANDBY};
pState primaryState = PROG;

void loop() {
  uint16_t  switches = SWALL_OFF;
  uint16_t  buttons;
  display.clear();
  if (swChange) {
    delay(10);
    swChange = false;
    switches = PCF.readButton16(SWALL_MASK);
    Serial.print("Switch Change:");
    Serial.println(switches, HEX);
    switch (switches & (SWARM_MASK | SWAUTO_MASK)) {
      case (SWARM_MASK | SWAUTO_MASK):
        Serial.println("All off");
        set16(RELAYALL_MASK, RELAYALL_MASK);
        primaryState = PROG;
        break;
      case SWARM_MASK:
        Serial.println("Auto on");
        set16(RELAYALL_MASK, RELAYALL_MASK);
        primaryState = STANDBY;
        break;
      case SWAUTO_MASK:
        Serial.println("Armed");
        set16(RELAYALL_MASK, RELAYALL_MASK & ~ARMRELAY_MASK & ~ARMLED_MASK);
        primaryState = MANUAL;
        break;
      case 0:
        Serial.println("Armed in Auto");
        set16(RELAYALL_MASK, RELAYALL_MASK & ~(ARMRELAY_MASK | ARMLED_MASK | AUTOLED_MASK));
        primaryState = AUTO;
        break;
    }
    buttons = switches & SWBUTTONS_MASK;
    Serial.print("Buttons:");
    Serial.println(buttons,HEX);
    switch (primaryState) {
    case PROG:
      break;
    case STANDBY:
      break;
    case MANUAL:
      switch (buttons) {
        case SWBUTTONS_MASK:
        Serial.println("No buttons pressed");
        break;
        case SWBUTTONS_MASK & ~SWFILL_MASK:
        Serial.println("Filling");
        display.println("FILL");
        break;
        case SWBUTTONS_MASK & ~SWPURGE_MASK:
        Serial.println("Purging");
        display.println("PURGE");
        break;
        case SWBUTTONS_MASK & ~SWFIRE_MASK:
        Serial.println("Firing");
        display.println("FIRE");
        break;
        case SWBUTTONS_MASK & ~SWSEQ_MASK:
        Serial.println("Auto Seq");
        display.println("SEQ");
        break;
        case SWBUTTONS_MASK & ~SWAIRFILL_MASK:
        Serial.println("Air Fill");
        display.println("Air Fill");
        break;
        case SWBUTTONS_MASK & ~SWAIRPURGE_MASK:
        Serial.println("Air Purge");
        display.println("Air Purge");
        break;
        case SWBUTTONS_MASK & ~(SWFILL_MASK | SWPURGE_MASK):
        Serial.println("Fill & purge");
        display.println("Fill+Pur");
        break;
        case SWBUTTONS_MASK & ~(SWAIRFILL_MASK | SWAIRPURGE_MASK):
        Serial.println("Air fill & purge");
        display.println("Air FP");
        break;
        default:
        Serial.println("Unknown state");
        display.println("Invalid");
      }
      break;
    case AUTO:
      break;
  }
  display.display();
}

// static uint state = 0;
// uint32_t startTime;
// uint16_t curSwitch;
// bool validState = false;
// digitalWrite(REDLED,HIGH);
// for (state = 0; state < NSTATES; state++) {
//   validState = false;
//   Serial.print("\tChecking: ");
//   Serial.print(m[state].swmask, HEX);
//   Serial.print(" for ");
//   Serial.println(m[state].swvalue, HEX);
//   if (readsw(m[state].swmask, m[state].swvalue)) {
//     Serial.print("State: ");
//     Serial.println(state);
//     Serial.print("Relay Mask: ");
//     Serial.println(m[state].relaymask, HEX);
//     Serial.print("Relay Set:  ");
//     Serial.println(m[state].relayvalue, HEX);
//     set16(m[state].relaymask, m[state].relayvalue);
//     validState = true;
//   }
// }
// digitalWrite(REDLED,LOW);
// /* if we are in auto and sequencing has been started
//    then initialize with a purge/fire cycle and then
//    commence fill purge fire cycles until we either have
//    a different switch state or have NSTACK fires
// */
// digitalWrite(GREENLED,HIGH);
// if (validState && (state == NSTATES)) {
//   Serial.println("Auto Mode");
//   set16(RELAYALL_MASK & ~ARMRELAY_MASK, PURGERELAY_MASK);
//   delay(250);
//   if (PCF.read(SWARM) == LOW) {
//     PCF.write(FIRERELAY, LOW);
//     delay(100);
//     PCF.write(FIRERELAY, HIGH);
//   }

//   for (int k = 0; k < NSTACK; k++) {
//     Serial.println("Fill");
//     set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~(FILLRELAY_MASK | PUMPRELAY_MASK));               //start fill w/ purge closed
//     startTime = millis();
//     curSwitch = PCF.read16()&(SWALL_MASK & ~SWAUTO_MASK);
//     while(millis()-startTime < FILLTIME) {
//       if(curSwitch != (PCF.read16()&(SWALL_MASK & ~SWAUTO_MASK))) break;
//       yield();
//     }
//     if(curSwitch != (PCF.read16()&(SWALL_MASK & ~SWAUTO_MASK))) break;                                                                         //fill delay
//     Serial.println("Purge");
//     set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~(FILLRELAY_MASK | PUMPRELAY_MASK | PURGERELAY_MASK)); //open purge
//     delay(PURGETIME);                                                                           //purge delay
//     Serial.println("Wait");
//     set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~0);                                             //all closed
//     delay(DEADTIME);                                                                            //close delay
//     if (PCF.read(SWARM) == LOW) {
//       Serial.println("Fire");
//       set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~FIRERELAY_MASK);
//     }
//     Serial.println("Wait");
//     delay(FIRETIME);
//     set16(RELAYALL_MASK & ~ARMRELAY_MASK, ~0);
//     Serial.println("--Finished");
//     delay(DEADTIME);
//     if(PCF.read(SWAUTO) == HIGH) break;       //end sequence early if swauto is set off
//   }
//   Serial.println("Finished Stack");
// }
// digitalWrite(GREENLED,LOW);
yield();
}
