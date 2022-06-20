// JellyFish SDCard routines

#include <SPI.h>
#include <SdFat.h>
#include <sdios.h>

#define SD_CS 4
#define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI)
#define SPI_SPEED       SD_SCK_MHZ(10)                    //  12 - may need lower speed for Sapphire Mini


bool sdcard_begin(){
 
  Serial.println("Setup SD card");
  if(!sd.begin(SD_CS,SPI_SPEED)) {
    Serial.println("Unable to start sd!!");
    return(false);
  }
  return(sdcard_firstfile());
}
bool sdcard_firstfile() {
  char fname[]="Jelly001.txt";
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
  char fname[]="Jellynnn.txt";
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

