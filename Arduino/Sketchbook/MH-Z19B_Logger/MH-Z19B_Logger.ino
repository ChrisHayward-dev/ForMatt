// MH-Z19A CO2 logger
//  we use pulse width because I couldn't get the UART method to work

#include <RTClib.h>
#include <SdFat.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define REDLED    13
#define GREENLED  8
#define CO2       6
#define VBAT      A7

#define CONSOLE_WAIT        20000L
#define SAMPLES_PER_FILE    3600    // open new file after this many samples
#define SAMPLES_PER_FLUSH   4       // flush the file after this many samples
#define SNOOZE_TIME         4000    // ms between sleeps

inline int digitalReadFast(int pin) {
  return !!(PORT->Group[g_APinDescription[pin].ulPort].IN.reg & (1ul << g_APinDescription[pin].ulPin));
}

RTC_DS3231 rtc;
SdFat sd;
File  sdfile;
#define SD_CS 4
#define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI)
#define SPI_SPEED       SD_SCK_MHZ(10)                    //  12 - may need lower speed for Sapphire Mini

Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)

bool sdcard_begin(){
 
  Serial.println("Setup SD card");
  if(!sd.begin(SD_CS,SPI_SPEED)) {
    Serial.println("Unable to start sd!!");
    return(false);
  }
  return(sdcard_firstfile());
}
bool sdcard_firstfile() {
  char fname[]="CO2MN001.txt";
  static int filenum = 65;
  do {
    fname[4] = filenum++;
    if(filenum==91) filenum = 97;
    if(filenum==124) return(false);
    Serial.print("Checking ");Serial.println(fname);
  } while (sd.exists(fname));
  sdfile.open(fname,O_RDWR|O_CREAT);
  if(!sdfile) return(false);
  return(true);
}

bool sdcard_nextfile() {
  char fname[]="CO2MNnnn.txt";
  static int filenum = 0;
  int segn = 0;
  do {
    segn = ++filenum;
    fname[5]=segn/100 + '0';
    segn = segn%100;
    fname[6]=segn/10 + '0';
    segn = segn%10;
    fname[7]=segn+'0';
    Serial.print("Checking ");Serial.println(fname);
  } while (sd.exists(fname));
  sdfile.open(fname,O_RDWR|O_CREAT);
  if(!sdfile) return(false);
  return(true);
}

void dumpDS3231(Stream *dest) {
  DateTime now = rtc.now();
  dest->print(now.year(), DEC);
  dest->print('/');
  dest->print(now.month(), DEC);
  dest->print('/');
  dest->print(now.day(), DEC);
  dest->print("T");
  dest->print(now.hour(), DEC);
  dest->print(':');
  dest->print(now.minute(), DEC);
  dest->print(':');
  dest->print(now.second(), DEC);
}
void setup() {
  uint32_t  stime = millis();
  pinMode(REDLED,OUTPUT);
  pinMode(GREENLED,OUTPUT);
  pinMode(CO2,INPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  digitalWrite(REDLED,HIGH);
  while((!Serial)&&((millis()-stime)<CONSOLE_WAIT )) yield();

  Serial.println("Starting CO2 monitor w/ MH-Z14A");
  if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      while (1) delay(10);
    }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("DS3231 init");
  dumpDS3231(&Serial);
  Serial.println("Init BMP388");
  if(!bmp.begin_I2C()){
    Serial.println("Unable to init BMP388");
    while(true);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("Opening SD Card");
  if(!sdcard_begin()) while(true) delay(10);
  sdcard_firstfile();
  sdfile.println("Time,Temperature,Pressure,VBattery,CO2");
  Serial.println("\nReady for reading");
  Serial.println("Time Temperature Pressure VBattery CO2");

}

void loop() {
  static uint32_t sampleCount = 0;
  uint32_t th,tl,ppm;
  uint32_t riseTime,fallTime,lastFallTime;
  float    measuredvbat;
  // Serial.print("\n----- Time from start: ");
  // Serial.print(millis() / 1000);
  // Serial.println(" s");
  lastFallTime = fallTime;
  while(digitalReadFast(CO2)==LOW);
  riseTime = micros();
  while(digitalReadFast(CO2)==HIGH);
  fallTime = micros();
  th = fallTime - riseTime;
  tl = 1000000L - th;
  digitalWrite(REDLED,LOW);
  // Serial.print(th);Serial.print(" ");
  // Serial.print(tl);Serial.print(" ");

  ppm = 5000L * (th - 2000) / (th + tl - 4000);

  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  measuredvbat = analogRead(VBAT);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  dumpDS3231(&Serial);Serial.print(" ");
  Serial.print(bmp.temperature);Serial.print(" ");
  Serial.print(bmp.pressure);Serial.print(" ");
  Serial.print(measuredvbat);Serial.print(" ");
  Serial.println(ppm);

  dumpDS3231(&sdfile);
  sdfile.print(",");
  sdfile.print(bmp.temperature);sdfile.print(",");
  sdfile.print(bmp.pressure);sdfile.print(",");
  sdfile.print(measuredvbat);sdfile.print(",");
  sdfile.println(ppm);
  if(++sampleCount % SAMPLES_PER_FLUSH == 0) {
      Serial.println("flushed file");
      sdfile.flush();
      digitalWrite(GREENLED,!digitalRead(GREENLED));
  }
  if(sampleCount % SAMPLES_PER_FILE == 0) {
    sdfile.close();
    sdcard_nextfile();
  }
  digitalWrite(GREENLED,!digitalRead(GREENLED));
  delay(SNOOZE_TIME);
}
