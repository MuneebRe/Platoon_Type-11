#include <MPU6050_tockn.h>
#include <Wire.h>

class MPU6050a
{
  private:
  double yaw;
  double compensator, c1, c2;
  MPU6050* mpu6050;

  public:
  MPU6050a();
  double get_yaw(){return yaw;}
  void update();
  ~MPU6050a();
  
};
