#include <Arduino.h>
#include "bot.h"
#include "MPU6050.h"
#include "HC-SR04.h"
#include "HS422.h"
#include "LED.h"
#include "Hall_Sensor.h"
#include "HC05.h"

class Platoon_Type_11: bot
{
private:
  //MPU6050a* mpu6050;
  HC_SR04* ping;
  HS422* servo_r;
  HS422* servo_L;
  LED* led;
  Hall_Sensor* hall;
  HC05* btSerial;
  
  double ping_distance;
  double t;
  int shared_mem[3];

public:
  using bot::bot;

  void begin();
  void update();
  void scout();
  void drive(HS422* servo_L, HS422* servo_r);
  
  ~Platoon_Type_11();
  

};
