// MPU 9250 output
// we don't seem to have a magnetometer on this chip...
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
void MPU9250_setup(){
  Wire.begin();
if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Offset cali done!");

  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  delay(200);
  Serial.println("MP9250 is ready!");
}

void MPU9250_getValues(float *resultantG,float *acc_x,float *acc_y,float *acc_z,float *gyro_x,float *gyro_y,float *gyro_z,float *temperature){
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  *resultantG = myMPU9250.getResultantG(gValue);
  *acc_x = gValue.x;
  *acc_y = gValue.y;
  *acc_z = gValue.z;
  *gyro_x= gyr.x;
  *gyro_y= gyr.y;
  *gyro_z= gyr.z;
  *temperature = temp;
}
