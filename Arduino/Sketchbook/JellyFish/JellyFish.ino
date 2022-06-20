// JellyFish.ino
//  This combines a temperature measurement with a simple IMU (and perhaps later a pressure gauge)
#include <SdFat.h>
#include <ZeroPowerManager.h>
#include <SPI.h>

SdFat  sd;
File  sdfile;

#define REDLED              13      // pin definitions
#define GREENLED            8
#define VBATPIN             A7

#define SAMPLES_PER_FILE    64000   // open new file after this many samples
#define SAMPLES_PER_FLUSH   100     // flush the file after this many samples
#define SAMPLES_UNTIL_SLEEP 2100    // start sleeping after this many samples
#define SNOOZE_TIME         200     // ms between sleeps
#define DUMPIT(...)           Serial.print(__VA_ARGS__);sdfile.print(__VA_ARGS__)
#define DUMPITLN(...)         Serial.println(__VA_ARGS__);sdfile.println(__VA_ARGS__)
#define SPACE               Serial.print(",");sdfile.print(",")

void setup() {
  uint32_t  startTime = millis();
  pinMode(GREENLED,OUTPUT);
  pinMode(REDLED,OUTPUT);
  digitalWrite(REDLED,HIGH);
  analogReadResolution(12);
  Serial.begin(115200);
  while((!Serial) && ((millis()-startTime)<15000L)) yield();  //wait 15 seconds for console to open
  Serial.println("Starting JellyFish");
  zpmRTCInit();
  if(!sdcard_begin()) {
    blink_error();
  }
  
  SM172_setup();
  MPU9250_setup();
  digitalWrite(REDLED,LOW);
  DUMPITLN("Sample,Time(ms),Pressure,Temperature,acc_res,accX,accY,accZ,gyroX,gyroY,gyroZ,InternalTemperature,Battery");
  zpmRTCInterruptEvery(SNOOZE_TIME,NULL);
}


void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t startTime = millis();
  static uint32_t nextSaveTime = millis()+10000L;
  static uint32_t sampleCount = 0;
  static int16_t  pressureCount = 1;
  static int32_t  pressureSum = 0;
  float temperature;
  float resultantG;
  float acc_x,acc_y,acc_z;
  float gyro_x,gyro_y,gyro_z;
  float internalTemperature;
  float measuredvbat = 0;
  float pressureCounts;
  if(sampleCount<SAMPLES_UNTIL_SLEEP) {
  } else {
    zpmPortDisableUSB();
  }
  __WFI();
  if(SM172_getTemperature(&temperature)){
    sampleCount++;
    MPU9250_getValues(&resultantG,&acc_x,&acc_y,&acc_z,&gyro_x,&gyro_y,&gyro_z,&internalTemperature);
    pressureCounts = (float)pressureSum/pressureCount;
    pressureCount = pressureSum = 0;
    measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= (1024*4); // convert to voltage
    DUMPIT(sampleCount);SPACE;
    DUMPIT(millis()-startTime);SPACE;
    DUMPIT(pressureCounts);SPACE;
    DUMPIT(temperature,4);SPACE;
    DUMPIT(resultantG,4);SPACE;
    DUMPIT(acc_x,4);SPACE;
    DUMPIT(acc_y,4);SPACE;
    DUMPIT(acc_z,4);SPACE;
    DUMPIT(gyro_x,3);SPACE;
    DUMPIT(gyro_y,3);SPACE;
    DUMPIT(gyro_z,3);SPACE;
    DUMPIT(internalTemperature,4);SPACE;
    DUMPIT(measuredvbat,2);
    DUMPITLN();
    digitalWrite(GREENLED,!digitalRead(GREENLED));
  
    if(sampleCount % SAMPLES_PER_FLUSH == 0) {
      Serial.println("flushed file");
      sdfile.flush();
      digitalWrite(8,!digitalRead(8));
    }
    if(sampleCount % SAMPLES_PER_FILE == 0) {
      sdfile.close();
      if(!sdcard_nextfile()) {
        blink_error();
      }
    }
  } else {
    pressureCount++;
    pressureSum += analogRead(A1);
  }
}

void blink_error() {
  EIC->EVCTRL.reg &= ~EIC_EVCTRL_EXTINTEO4;             // clear the pin interrupt
  zpmRTCInterruptEvery(250,NULL);
  while(true) {
    digitalWrite(REDLED,!digitalRead(REDLED));
    zpmSleep();
  }
}