/* Seismic Source controller (ie pickel controller)
   cth 7/11/2022
*/
#include <PCF8575.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <sdios.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/* Pickle options */
#define USE_CONSOLE   0       //If 1 will pause to allow a console to be connected
#define USE_FIRESENSE 1       //if 1 will use fire sense (optical and spark sense) to get trigger output time

#define SPARKLEVEL      50
#define PHOTOLEVEL      50
#define PUMPCAPACITY    462 //mL/min

/* Startup values */
#define NSTACK          5
#define PREPURGE_DELAY  4000    // ms  - open purge line long enough to bleed any excess pressure
#define FILLVOLUME      5       // mL  - miniumum volume to ignite
#define PURGEVOLUME     10      // mL  - sum the purge volume + fill volume to get gas volume
#define DEADTIME        2000    // ms  - time to wait before starting fire sequence
#define MAX_FILLVOLUME  150     // mL  - maximum volume before opening purge (max pickle volume)
#define MIN_PURGEVOLUME 10      // mL  - needs to be at least 1500 ms
#define MAX_PURGEVOLUME 150     // mL  - max volume in excess of pickle
#define READYDELAY      1000    // ms  - delay prior to firing to allow valves to close and settle
#define FIREDELAY       2500    // ms  - enough time for several sparks even w/ low battery
#define AIRPURGEVOLUME  1000    // should be serveral times the hose + pickle volume
#define NEXTDELAY       1000    // ms  - delay to next shot
#define POFFSET         99      // supply pressure reading at 0 psi
#define P23VALUE        744     // supply pressure reading at 23 psi

#define TIME2VOL(x,p)     ((float)((x)/1000.0)*(p/14.7)*PUMPCAPACITY/60.0)    // time in ms, absolute pressure in PSI
#define VOL2TIME(x,p)     (1000L*(x)/((p*PUMPCAPACITY/60/14.7)))
#define PRESSURE(x)       (14.7+(23.0*((float)(x)-POFFSET))/(P23VALUE-POFFSET))   // analog to absolute pressure reading

// Arduino pin definitions
#define SWUP          5
#define SWDOWN        6
#define GREENLED      8
#define SWPLUS        9
#define SWMINUS       10
#define TRIGOUT       11
#define PCF_INTERRUPT 12
#define REDLED        13
#define SUPPLYPRESSURE A3
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
#define OLEDP4(v,w,x,y) {display.clearDisplay();display.setCursor(0,0);display.print(v);display.print(w);display.println(" ");display.print(x);display.println(y);display.display();}

#define MAXBUFR 132
char bufr[MAXBUFR];

// Library definitions
#define SCREEN_WIDTH    128     // OLED display width, in pixels
#define SCREEN_HEIGHT   32     // OLED display height, in pixels
#define OLED_RESET      -1         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PCF8575 PCF(0x20);

//SDFAT defs
#define SPI_SPEED SD_SCK_MHZ(4)
SdFs sd;
FsFile  file;
#define SD_CS 4

char fileName[] = "pickle000.txt";

enum astate {IDLE, INIT, PREPURGE, NEXT, FILL, PURGE, READY, FIRE, MISFIRE};
astate autoState = IDLE;
uint16_t  stackLimit      = NSTACK;
uint16_t  num2stack       = NSTACK;
uint16_t  nextDelay       = NEXTDELAY;
uint16_t  fillVolume      = FILLVOLUME;
uint16_t  prePurgeDelay   = PREPURGE_DELAY;
uint16_t  fillpurgeVolume = MIN_PURGEVOLUME;
uint16_t  readyDelay      = READYDELAY;
uint16_t  fireDelay       = FIREDELAY;

#ifdef DEBUG
#define   DPRINT(...)       Serial.print(__VA_ARGS__)
#define   DPRINTLN(...)     Serial.println(__VA_ARGS__)
#else
#define   DPRINT(...)
#define   DPRINTLN(...)
#endif

ArduinoOutStream  cout(Serial);

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
  int16_t   valPhoto = 32767;
  int       count = 0;
  int       pcount = 0;
  int       peakcount = 0;
  bool      rtn = false;
  bool      gotSpark = false;
  bool      gotPhoto = false;
  uint32_t  fTime;
  float     pressure = 0;
  if (PCF.read(SWARM) == HIGH) return (true);
  DPRINTLN("Firing!");
  SET_CONTROLRELAY(FIRERELAY_MASK);
  delay(20);    //delay long enough for initial pulse to decay
  while ((millis() - startTime) < fireDelay) {
    valSpark = max(analogRead(SPARK), valSpark);
    if ((valSpark > 50) && (!rtn)) {         //Spark was detected (probably)
      digitalWrite(TRIGOUT, LOW);
      fTime = millis();
      SET_CONTROLRELAYOFF;
      rtn = true;
    } else {
      count++;
    }
    if (rtn) {
      int16_t v;
      pcount++;
      if ((v = analogRead(PHOTO)) < valPhoto) {
        valPhoto = v;
        peakcount = pcount;
      }
      valPhoto = min(analogRead(PHOTO), valPhoto);
    }
  }
  SET_CONTROLRELAYOFF;
  delay(100);
  digitalWrite(TRIGOUT, HIGH);
  if (!rtn) {
    DPRINTLN("Misfire!");
  } else {
    DPRINTLN("Fire Detected!");
  }
  pressure = PRESSURE(analogRead(SUPPLYPRESSURE)) - 15;
  snprintf(bufr, MAXBUFR, "%d,Auto Fire,%d,ms,%d,ms delay,%d,count,%d,spark,%d,photo,%d,pcount,%d,psi pressure\n", millis(), fTime, fTime - startTime, count, valSpark, valPhoto, peakcount, (long)pressure);
  Serial.print(bufr);
  file.print(bufr); file.sync();
  DPRINT("Delay: "); DPRINTLN(fTime - startTime);
  DPRINT("Count: "); DPRINTLN(count);
  DPRINT("Spark: "); DPRINTLN(valSpark);
  DPRINT("Photo: "); DPRINTLN(valPhoto);
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

  // Open the SD file for write
  if (!sd.begin(SD_CS, SPI_SPEED)) {
    if (sd.card()->errorCode()) {
      cout << F(
             "\nSD initialization failed.\n"
             "Do not reformat the card!\n"
             "Is the card correctly inserted?\n"
             "Is chipSelect set to the correct value?\n"
             "Does another SPI device need to be disabled?\n"
             "Is there a wiring/soldering problem?\n");
      cout << F("\nerrorCode: ") << hex << showbase;
      cout << int(sd.card()->errorCode());
      cout << F(", errorData: ") << int(sd.card()->errorData());
      cout << dec << noshowbase << endl;
      OLED("SD error");
      errorStop(4);
    }
  }
  Serial.println("SD card ok");
  if (!file.open(fileName, O_CREAT | O_APPEND | O_SYNC | O_WRITE)) {
    OLED("SD open");
    errorStop(5);
  }
  file.println("Pickle Start");
  safeswitch(SWARM, ARMLED);
  safeswitch(SWAUTO, AUTOLED);
  DPRINTLN("Starting Switch Interrupt");
  Serial.println(__FILE__);
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  snprintf(bufr, MAXBUFR, "Pickle Control: %s %s %s\n", __FILE__, __DATE__, __TIME__);
  Serial.print(bufr);
  file.print(bufr); file.sync();

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
    DPRINT("Switch Change:");
    DPRINTLN(switches, HEX);
    toggles = switches & (SWARM_MASK | SWAUTO_MASK);
    if (toggles != prevToggles) {
      prevToggles = toggles;
      digitalWrite(TRIGOUT, HIGH);  //clear the trigger out just in case it got left on
      display.clearDisplay();
      switch (toggles) {
        case (SWARM_MASK | SWAUTO_MASK):
          DPRINTLN("All off");
          set16(RELAYALL_MASK, RELAYALL_MASK);
          primaryState = PROG;
          autoState = IDLE;
          OLED("Program");
          break;
        case SWARM_MASK:
          DPRINTLN("Auto on");
          set16(RELAYALL_MASK, RELAYALL_MASK );
          primaryState = STANDBY;
          autoState = IDLE;
          OLED("Standby");
          break;
        case SWAUTO_MASK:
          DPRINTLN("Armed");
          set16(RELAYALL_MASK, RELAYALL_MASK & ~ARMRELAY_MASK & ~ARMLED_MASK);
          primaryState = MANUAL;
          OLED("Rdy Manual");
          autoState = IDLE;
          break;
        case 0:
          DPRINTLN("Armed in Auto");
          set16(RELAYALL_MASK, RELAYALL_MASK & ~(ARMRELAY_MASK | ARMLED_MASK ));
          primaryState = AUTO;
          OLED("Rdy Auto");
          break;
      }
    }
    buttons = switches & SWBUTTONS_MASK;
    DPRINT("Buttons:");
    DPRINTLN(buttons, HEX);
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
  DPRINTLN("Starting Air purge");
  OLED("AirPurge");
  SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK | PURGERELAY_MASK);
  while ((millis() - sTime) < VOL2TIME(AIRPURGEVOLUME, 15)) {
    if (swChange) {
      delay(20);
      if ((switches = PCF.readButton16(SWALL_MASK)) == 0xC0FE) {
        DPRINTLN("Switches as expected");
        swChange = false;
      } else {
        DPRINT("Air Purge Cancel:");
        DPRINTLN(switches, HEX);
        DPRINTLN("Ouch - done");
        break;
      }
    }
    yield();
  }
  if (swChange) {
    OLED("AP Cancel");
  } else {
    OLED("AP done");
  }
  SET_CONTROLRELAYOFF;
  snprintf(bufr, MAXBUFR, "%d,AirPurge,%d,ms\n", millis(), millis() - sTime);
  Serial.print(bufr);
  file.print(bufr); file.sync();
}

void programSettings(uint16_t buttons) {
  uint16_t prevButtons;
  static uint32_t sTime;
  static uint16_t reqFillVolume = 0;
  static uint16_t prevButton = 0;
  if (reqFillVolume == 0) {
    reqFillVolume = fillpurgeVolume + fillVolume;
  }
  switch (buttons) {
    case SW(0):
      sTime = millis();
      if (prevButton != buttons) {
        if (reqFillVolume > (MAX_FILLVOLUME + MIN_PURGEVOLUME)) {
          fillpurgeVolume = (reqFillVolume - MAX_FILLVOLUME);
          fillVolume  = MAX_FILLVOLUME;
        } else {
          fillVolume = reqFillVolume - MIN_PURGEVOLUME;
          fillpurgeVolume = MIN_PURGEVOLUME;
        }
        OLEDP4("Stk:", stackLimit, " Vol:", reqFillVolume);
        snprintf(bufr, MAXBUFR, "%d,Program,%d,stacks,%d,mL fill\n", millis(), stackLimit, reqFillVolume);
        Serial.print(bufr);
        file.print(bufr); file.sync();
      }
      break;
    case SW(SWFILL_MASK):   // increase Fill (never more than 150 ml though)
      if ((millis() - sTime) > 500) {
        sTime = millis();
        reqFillVolume += 5;
        OLEDP("FILL:", reqFillVolume);
      }
      break;
    case SW(SWAIRFILL_MASK):  // decrease Fill time
      if ((millis() - sTime) > 500) {
        sTime = millis();
        reqFillVolume -= 5;
        if ( reqFillVolume < MIN_PURGEVOLUME ) {
          reqFillVolume = MIN_PURGEVOLUME;
        }
        OLEDP("FILL:", reqFillVolume);
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
        DPRINTLN("Init Seq");
      } else {
        autoState = IDLE;
        SET_CONTROLRELAYOFF;
        DPRINTLN("Cancel Seq");
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
        DPRINTLN("Cancel Seq");
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
  static uint32_t fTime = 0;
  static uint16_t nStacked = 0;
  static astate prevState;
  static float volFilled = 0;
  float  pressure = 0;
  uint32_t  analog = 0;
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
      if ((millis() - sTime) > prePurgeDelay) {
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
        fTime = sTime = millis();
        OLED("A Fill");
        volFilled = 0;
        SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK | VALVERELAY_MASK);
      }
      break;
    case FILL:
    analog = 0;
      for (int k = 0; k < 10; k++) {
        analog += analogRead(SUPPLYPRESSURE);
      }
      pressure = PRESSURE(analog/10);
      volFilled += TIME2VOL(millis() - fTime, pressure);
      fTime = millis();
      if (volFilled >= fillVolume) {
        Serial.print("Fill Time:"); Serial.print(millis() - sTime); Serial.print(" Pressure:"); Serial.println(pressure);
        autoState = PURGE;
        fTime = sTime = millis();
        volFilled = 0;
        OLED("A Purge");

        SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK | VALVERELAY_MASK | PURGERELAY_MASK);
      }
      break;
    case PURGE:
    analog = 0;
      for (int k = 0; k < 10; k++) {
        analog += analogRead(SUPPLYPRESSURE);
      }
      pressure = PRESSURE(analog/10);
      volFilled += TIME2VOL(millis() - fTime, pressure);
      fTime = millis();
      if (volFilled >= fillpurgeVolume) {
        Serial.print("Fill Time:"); Serial.print(millis() - sTime); Serial.print(" Pressure:"); Serial.println(pressure);
        autoState = READY;
        sTime = millis();
        OLED("A Armed");
        SET_CONTROLRELAYOFF;
      }
      break;
    case READY:
      if ((millis() - sTime) > readyDelay) {
        snprintf(bufr, MAXBUFR, "%d,Auto Firing,%d,stacked,%d,fill,%d,purge,%d,delay,%d,pressure\n", millis(), nStacked, fillVolume, fillpurgeVolume, fireDelay, (long)pressure);
        file.print(bufr); file.sync();
        autoState = FIRE;
        sTime = millis();
        OLED("A Fire");
#if USE_FIRESENSE == 0
        SET_CONTROLRELAY(FIRERELAY_MASK);
#else
        if (!fire()) {
          OLED("MisFire");
          DPRINTLN("MisFire!");
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
      DPRINTLN("No buttons pressed");
      OLED("Manual");
      SET_CONTROLRELAYOFF;
      digitalWrite(TRIGOUT, HIGH);
      snprintf(bufr, MAXBUFR, "%d,Manual,Idle\n", millis());
      Serial.print(bufr);
      file.print(bufr); file.sync();
      break;
    case SW(SWFILL_MASK):
      DPRINTLN("Filling");
      OLED("FILL");
      SET_CONTROLRELAY(VALVERELAY_MASK | FILLRELAY_MASK | PUMPRELAY_MASK);
      snprintf(bufr, MAXBUFR, "%d,Manual,Fill\n", millis());
      Serial.print(bufr);
      file.print(bufr); file.sync();
      break;
    case SW(SWPURGE_MASK):
      DPRINTLN("Purging");
      OLED("PURGE");
      SET_CONTROLRELAY(PURGERELAY_MASK);
      snprintf(bufr, MAXBUFR, "%d,Manual,Purge\n", millis());
      Serial.print(bufr);
      file.print(bufr); file.sync();
      break;
    case SW(SWFIRE_MASK):
      DPRINTLN("Firing");
      OLED("FIRE");
      SET_CONTROLRELAY(FIRERELAY_MASK);
      digitalWrite(TRIGOUT, LOW);
      snprintf(bufr, MAXBUFR, "%d,Manual,Fire\n", millis());
      Serial.print(bufr);
      file.print(bufr); file.sync();
      break;
    case SW(SWSEQ_MASK):
      DPRINTLN("Auto Seq");
      OLED("SEQ");
      break;
    case SW(SWAIRFILL_MASK):
      DPRINTLN("Air Fill");
      OLED("Air Fill");
      SET_CONTROLRELAY(FILLRELAY_MASK | PUMPRELAY_MASK);
      snprintf(bufr, MAXBUFR, "%d,Manual,AirFill\n", millis());
      Serial.print(bufr);
      file.print(bufr); file.sync();
      break;
    case SW(SWAIRPURGE_MASK):
      DPRINTLN("Air Purge");
      OLED("Air Purge");
      airPurge();
      break;
    case SW(SWFILL_MASK | SWPURGE_MASK):
      DPRINTLN("Fill & purge");
      OLED("Fill+Pur");
      SET_CONTROLRELAY(VALVERELAY_MASK | FILLRELAY_MASK | PUMPRELAY_MASK | PURGERELAY_MASK);
      snprintf(bufr, MAXBUFR, "%d,Manual,Fill+Purge\n", millis());
      Serial.print(bufr);
      file.print(bufr); file.sync();
      break;
    case SW(SWAIRFILL_MASK | SWAIRPURGE_MASK):
      DPRINTLN("Air fill & purge");
      OLED("Air FP");
      SET_CONTROLRELAY(PURGERELAY_MASK | FILLRELAY_MASK | PUMPRELAY_MASK);
      snprintf(bufr, MAXBUFR, "%d,Manual,Air Fill+Purge\n", millis());
      file.print(bufr); file.sync();
      break;
    default:
      Serial.println("Unknown state");
      OLED("Invalid");
  }
}
