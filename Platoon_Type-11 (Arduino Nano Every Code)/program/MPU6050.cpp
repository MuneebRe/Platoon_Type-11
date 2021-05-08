
#include "MPU6050.h"

MPU6050a::MPU6050a()
{
  mpu6050 = new MPU6050(Wire);
  Wire.begin();
  mpu6050->begin();
  mpu6050->calcGyroOffsets(true);

  //c1 = mpu6050->getAngleZ();
  //c2 = mpu6050->getAngleZ();
  //compensator = c1 - c2;

  
}

void MPU6050a::update()
{
  mpu6050->update();
  yaw = mpu6050->getAngleZ();
  //yaw -= compensator;

  Serial.println(yaw);
  
}

MPU6050a::~MPU6050a()
{
  delete mpu6050;
}
