/* Seismic Source controller (ie pickel controller)
   cth 7/11/2022
*/
#include <PCF8575.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/* Pickle options */
#define USE_CONSOLE   0       //If 1 will pause to allow a console to be connected
#define USE_FIRESENSE 1       //if 1 will use fire sense (optical and spark sense) to get trigger output time

#define SPARKLEVEL      50
#define PHOTOLEVEL      50

#define NSTACK          5
#define FILLTIME        1000
#define PURGETIME       1500
#define DEADTIME        2000
#define MAX_FILLDELAY   15000
#define MIN_PURGEDELAY  1500
#define MAXPURGE        20000
#define FIREDELAY       2500
#define AIRPURGETIME    60000

// Arduino pin definitions
#define SWUP          5
#define SWDOWN        6
#define GREENLED      8
#define SWPLUS        9
#define SWMINUS       10
#define TRIGOUT       11
#define PCF_INTERRUPT 12
#define REDLED        13
#define SPARK         A2
#define PHOTO         A1

// PCF pin defintions
#define SWARM         0
#define SWAUTO        1
#define SWSEQ         2
#define SWFILL        3
#define SWPURGE       4
#define SWFIRE        5
#define SWAIRFILL     6
#define SWAIRPURGE    7

#define ARMLED        8
#define AUTOLED       9
#define ARMRELAY      10
#define FILLRELAY     11
#define PURGERELAY    12
#define PUMPRELAY     13
#define FIRERELAY     14
#define VALVERELAY    15

#define SWARM_MASK      (1 << SWARM)
#define SWAUTO_MASK     (1 << SWAUTO)
#define SWSEQ_MASK      (1 << SWSEQ)
#define SWFILL_MASK     (1 << SWFILL)
#define SWPURGE_MASK    (1 << SWPURGE)
#define SWFIRE_MASK     (1 << SWFIRE)
#define SWAIRPURGE_MASK (1 << SWAIRPURGE)
#define SWAIRFILL_MASK  (1 << SWAIRFILL)

#define ARMLED_MASK     (1 << ARMLED)
#define AUTOLED_MASK    (1 << AUTOLED)
#define ARMRELAY_MASK   (1 << ARMRELAY)
#define FILLRELAY_MASK  (1 << FILLRELAY)
#define PURGERELAY_MASK (1 << PURGERELAY)
#define PUMPRELAY_MASK  (1 << PUMPRELAY)
#define FIRERELAY_MASK  (1 << FIRERELAY)
#define VALVERELAY_MASK (1 << VALVERELAY)

#define SWALL_MASK      0x00FF
#define SWALL_OFF       0x00FF
#define SWBUTTONS_MASK  (SWFILL_MASK|SWPURGE_MASK|SWFIRE_MASK|SWSEQ_MASK|SWAIRPURGE_MASK|SWAIRFILL_MASK)
#define RELAYALL_MASK   0xFF00
#define RELAYALL_OFF    0xFF00
#define RELAYCONTROL_MASK (RELAYALL_MASK & ~ARMLED_MASK & ~ARMRELAY_MASK)

#define SET_CONTROLRELAY(v)   set16(RELAYCONTROL_MASK,RELAYCONTROL_MASK & ~(v) & ~AUTOLED_MASK)
#define SET_CONTROLRELAYOFF   set16(RELAYCONTROL_MASK,RELAYCONTROL_MASK);
#define SW(v)                 (SWBUTTONS_MASK & ~(v))

#define OLED(v) {display.clearDisplay();display.setCursor(0,0);display.println(v);display.display();}
#define OLEDP(v,w){display.clearDisplay();display.setCursor(0,0);display.print(v);display.println(w);display.display();}

// Library definitions
#define SCREEN_WIDTH    128     // OLED display width, in pixels
#define SCREEN_HEIGHT   32     // OLED display height, in pixels
#define OLED_RESET      -1         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PCF8575 PCF(0x20);

enum astate {IDLE, INIT, PREPURGE, NEXT, FILL, PURGE, READY, FIRE, MISFIRE};
astate autoState = IDLE;
uint16_t  stackLimit = 5;
uint16_t  num2stack  = 5;
uint16_t  nextDelay  = 1000;
uint16_t  fillDelay  = FILLTIME;
uint16_t  purgeDelay = 3*MIN_PURGEDELAY;
uint16_t  fillpurgeDelay = MIN_PURGEDELAY;
uint16_t  readyDelay  = 1000;
uint16_t  fireDelay  = FIREDELAY;

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

/* fire roouture returns TRUE if fire was successful, FALSE if FIRE has not yet been sensed*/
bool fire() {
  uint32_t startTime = millis();
  int16_t   valSpark = 0;
  int16_t   valPhoto = 0;
  int       count = 0;
  bool      rtn = false;
  bool      gotSpark = false;
  bool      gotPhoto = false;
  uint32_t  fTime;
  if (PCF.read(SWARM) == HIGH) return (true);
  Serial.println("Firing!");
  SET_CONTROLRELAY(FIRERELAY_MASK);
  delay(20);    //delay long enough for initial pulse to decay
  while ((millis() - startTime) < fireDelay) {
    valSpark = max(analogRead(SPARK), valSpark);
    count++;
    if ((!rtn) && valSpark > 50) {         //Spark was detected (probably)
      digitalWrite(TRIGOUT, LOW);
      fTime = millis();
      SET_CONTROLRELAYOFF;
      rtn = true;
    }
    if (rtn) {
      valPhoto = max(analogRead(PHOTO), valPhoto);
    }
  }
  SET_CONTROLRELAYOFF;
  delay(100);
  digitalWrite(TRIGOUT, HIGH);
  if (!rtn) {
    Serial.println("Misfire!");
  } else {
    Serial.println("Fire Detected!");
  }
  Serial.print("Delay: "); Serial.println(fTime - startTime);
  Serial.print("Count: "); Serial.println(count);
  Serial.print("Spark: "); Serial.println(valSpark);
  Serial.print("Photo: "); Serial.println(valPhoto);
  return (rtn);
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
  pinMode(PHOTO, INPUT);

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
  Serial.println(__DATE__);
  Serial.println(__TIME__);

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
  }
  safeswitch(SWARM, ARMLED);
  safeswitch(SWAUTO, AUTOLED);
  Serial.println("Starting Switch Interrupt");
  Serial.println(__FILE__);
  Serial.println(__DATE__);
  Serial.println(__TIME__);

  attachInterrupt(digitalPinToInterrupt(PCF_INTERRUPT), switchChange, FALLING); // setup the switch detect interrupt
  digitalWrite(GREENLED, HIGH);
  digitalWrite(REDLED, LOW);
}

enum pState { PROG, MANUAL, AUTO, STANDBY};
pState primaryState = PROG;

void loop() {
  uint16_t  switches = SWALL_OFF;
  uint16_t  toggles;
  static uint16_t  prevToggles = SWARM_MASK | SWAUTO_MASK;
  static uint16_t  buttons;
  static uint16_t prevSw = SWARM_MASK | SWAUTO_MASK;
  if (swChange) {
    delay(10);
    swChange = false;
    switches = PCF.readButton16(SWALL_MASK);
    Serial.print("Switch Change:");
    Serial.println(switches, HEX);
    toggles = switches & (SWARM_MASK | SWAUTO_MASK);
    if (toggles != prevToggles) {
      prevToggles = toggles;
      digitalWrite(TRIGOUT, HIGH);  //clear the trigger out just in case it got left on
      display.clearDisplay();
      switch (toggles) {
        case (SWARM_MASK | SWAUTO_MASK):
          Serial.println("All off");
          set16(RELAYALL_MASK, RELAYALL_MASK);
          primaryState = PROG;
          autoState = IDLE;
          OLED("Program");
          break;
        case SWARM_MASK:
          Serial.println("Auto on");
          set16(RELAYALL_MASK, RELAYALL_MASK );
          primaryState = STANDBY;
          autoState = IDLE;
          OLED("Standby");
          break;
        case SWAUTO_MASK:
          Serial.println("Armed");
          set16(RELAYALL_MASK, RELAYALL_MASK & ~ARMRELAY_MASK & ~ARMLED_MASK);
          primaryState = MANUAL;
          OLED("Rdy Manual");
          autoState = IDLE;
          break;
        case 0:
          Serial.println("Armed in Auto");
          set16(RELAYALL_MASK, RELAYALL_MASK & ~(ARMRELAY_MASK | ARMLED_MASK ));
          primaryState = AUTO;
          OLED("Rdy Auto");
          break;
      }
    }
    buttons = switches & SWBUTTONS_MASK;
    Serial.print("Buttons:");
    Serial.println(buttons, HEX);
    switch (primaryState) {
      case PROG:
        programSettings(buttons);
        break;
      case STANDBY:
        break;
      case MANUAL:
        manualSettings(buttons);
        break;
      case AUTO:
        autoSettings(buttons);
        break;
    }
  }
  switch (primaryState) {
    case AUTO:
      autoProcess();
      break;
    case PROG:
      programSettings(buttons);
      break;
  }
  yield();
}
void airPurge() {
  uint32_t  sTime = millis();
  uint16_t  switches = SWALL_OFF;
  Serial.println("Starting Air purge");
  OLED("AirPurge");
  SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK | PURGERELAY_MASK);
  while((millis() - sTime) < AIRPURGETIME) {
    if(swChange) {
      delay(20);
      if((switches=PCF.readButton16(SWALL_MASK))==0xC0FE) {
        Serial.println("Switches as expected");
        swChange = false;
      } else {
        Serial.print("Air Purge Cancel:");
        Serial.println(switches,HEX);
        Serial.println("Ouch - done");
        break;
      }
    }
    yield();
  }
  if(swChange) {
    OLED("AP Cancel");
  } else {
    OLED("AP done");
  }
  SET_CONTROLRELAYOFF;
}

void programSettings(uint16_t buttons) {
  uint16_t prevButtons;
  static uint32_t sTime;
  static uint16_t reqFillDelay = 0;
  static uint16_t prevButton = 0;
  if (reqFillDelay == 0) {
    reqFillDelay = fillpurgeDelay + fillDelay;
  }
  switch (buttons) {
    case SW(0):
      sTime = millis();
      if (prevButton != buttons) {
        if (reqFillDelay > MAX_FILLDELAY) {
          fillpurgeDelay = (reqFillDelay - MAX_FILLDELAY) + MIN_PURGEDELAY;
          fillDelay  = MAX_FILLDELAY;
        } else {
          fillDelay = reqFillDelay - MIN_PURGEDELAY;
          fillpurgeDelay = MIN_PURGEDELAY;
        }
        OLED("Prog Set");
      }
      break;
    case SW(SWFILL_MASK):   // increase Fill time (never more than 150 ml though)
      if ((millis() - sTime) > 500) {
        sTime = millis();
        reqFillDelay += 500;
        OLEDP("FILL:", reqFillDelay);
      }
      break;
    case SW(SWAIRFILL_MASK):  // decrease Fill time
      if ((millis() - sTime) > 500) {
        sTime = millis();
        reqFillDelay -= 500;
        if ( reqFillDelay < MIN_PURGEDELAY ) {
          reqFillDelay = MIN_PURGEDELAY;
        }
        OLEDP("FILL:", reqFillDelay);
      }
      break;
    case SW(SWPURGE_MASK):
      if ((millis() - sTime) > 500) {
        sTime = millis();
        stackLimit++;
        OLEDP("Stack:", stackLimit);
      }
      break;
    case SW(SWAIRPURGE_MASK):
      if ((millis() - sTime) > 500) {
        sTime = millis();
        stackLimit--;
        if (stackLimit <= 0) stackLimit = 1;
        OLEDP("Stack:", stackLimit);
      }
      break;
  }
  prevButton = buttons;
}
void autoSettings(uint16_t buttons) {
  switch (buttons) {
    case SW(0):
      break;
    case SW(SWSEQ_MASK):
      if (autoState == IDLE) {
        autoState = INIT;
        num2stack = stackLimit;
        Serial.println("Init Seq");
      } else {
        autoState = IDLE;
        SET_CONTROLRELAYOFF;
        Serial.println("Cancel Seq");
        OLED("Ready");
      }
      break;
    case SW(SWFIRE_MASK):
      if (autoState == IDLE) {
        autoState = INIT;
        OLED("A Single");
        num2stack = 1;
      } else {
        autoState = IDLE;
        SET_CONTROLRELAYOFF;
        Serial.println("Cancel Seq");
        OLED("Ready");
      }
      break;
    default:
      Serial.println("Unknown state");
      OLED("Ready");
      autoState = IDLE;
      SET_CONTROLRELAYOFF;
      break;
  }
}

void autoProcess() {
  static uint32_t sTime = 0;
  static uint16_t nStacked = 0;
  static astate prevState;
  prevState = autoState;
  switch (autoState) {
    case IDLE:
      sTime = millis();
      nStacked = 0;
      display.setCursor(0, 0);
      display.clearDisplay();
      OLED("A Ready");
      break;
    case INIT:
      autoState = PREPURGE;
      sTime = millis();
      display.setCursor(0, 0);
      display.clearDisplay();
      OLED("A PrePurge");
      SET_CONTROLRELAY(PURGERELAY_MASK);
      break;
    case PREPURGE:
      if ((millis() - sTime) > purgeDelay) {
        autoState = NEXT;
        sTime = millis();
        OLED("A Next");
        SET_CONTROLRELAYOFF;
      }
      break;
    case NEXT:
      if ((millis() - sTime) > nextDelay) {
        autoState = FILL;
        nStacked++;
        sTime = millis();
        OLED("A Fill");
        SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK | VALVERELAY_MASK);
      }
      break;
    case FILL:
      if ((millis() - sTime) > fillDelay) {
        autoState = PURGE;
        sTime = millis();
        OLED("A Purge");
        SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK | VALVERELAY_MASK | PURGERELAY_MASK);
      }
      break;
    case PURGE:
      if ((millis() - sTime) > fillpurgeDelay) {
        autoState = READY;
        sTime = millis();
        OLED("A Armed");
        SET_CONTROLRELAYOFF;
      }
      break;
    case READY:
      if ((millis() - sTime) > readyDelay) {
        autoState = FIRE;
        sTime = millis();
        OLED("A Fire");
#if USE_FIRESENSE == 0
        SET_CONTROLRELAY(FIRERELAY_MASK);
#else
        if (!fire()) {
          OLED("MisFire");
          Serial.println("MisFire!");
          autoState = MISFIRE;
          SET_CONTROLRELAYOFF;
        }
#endif
      }
      break;
    case FIRE:
      if ((millis() - sTime) > fireDelay) {
        sTime = millis();
        SET_CONTROLRELAYOFF;
        if (nStacked >= num2stack) {
          autoState = IDLE;
          OLED("A Ready");
        } else {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print(nStacked);
          display.print(" of ");
          display.println(num2stack);
          display.display();
          autoState = NEXT;
        }
      }
      break;
    case MISFIRE: //only way to reset misfire is to stop AUTO mode
      break;
  }
}

void manualSettings(uint16_t buttons) {
  display.setCursor(0, 0);
  display.clearDisplay();
  switch (buttons) {
    case SW(0):
      Serial.println("No buttons pressed");
      OLED("Manual");
      SET_CONTROLRELAYOFF;
      digitalWrite(TRIGOUT, HIGH);
      break;
    case SW(SWFILL_MASK):
      Serial.println("Filling");
      OLED("FILL");
      SET_CONTROLRELAY(VALVERELAY_MASK | FILLRELAY_MASK | PUMPRELAY_MASK);
      break;
    case SW(SWPURGE_MASK):
      Serial.println("Purging");
      OLED("PURGE");
      SET_CONTROLRELAY(PURGERELAY_MASK);
      break;
    case SW(SWFIRE_MASK):
      Serial.println("Firing");
      OLED("FIRE");
      SET_CONTROLRELAY(FIRERELAY_MASK);
      digitalWrite(TRIGOUT, LOW);
      break;
    case SW(SWSEQ_MASK):
      Serial.println("Auto Seq");
      OLED("SEQ");
      break;
    case SW(SWAIRFILL_MASK):
      Serial.println("Air Fill");
      OLED("Air Fill");
      SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK);
      break;
    case SW(SWAIRPURGE_MASK):
      Serial.println("Air Purge");
      OLED("Air Purge");
      airPurge();
      break;
    case SW(SWFILL_MASK | SWPURGE_MASK):
      Serial.println("Fill & purge");
      OLED("Fill+Pur");
      SET_CONTROLRELAY(VALVERELAY_MASK | FILLRELAY_MASK | PUMPRELAY_MASK | PURGERELAY_MASK);
      break;
    case SW(SWAIRFILL_MASK | SWAIRPURGE_MASK):
      Serial.println("Air fill & purge");
      OLED("Air FP");
      SET_CONTROLRELAY(PURGERELAY_MASK | FILLRELAY_MASK | PUMPRELAY_MASK);
      break;
    default:
      Serial.println("Unknown state");
      OLED("Invalid");
  }
}
