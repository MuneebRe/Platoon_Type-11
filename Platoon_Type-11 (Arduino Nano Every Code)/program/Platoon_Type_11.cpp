 #include "Platoon_Type_11.h"

void Platoon_Type_11::begin()
{
  //mpu6050 = new MPU6050a();
  ping = new HC_SR04(7, 8);
  servo_r = new HS422(10);
  servo_L = new HS422(9);
  led = new LED(3,5,6);
  hall = new Hall_Sensor(A6);
  btSerial = new HC05(2,4);

  shared_mem[0]=0;
  shared_mem[1]=0;
  shared_mem[2]=0;
}

void Platoon_Type_11::update()
{
  t = millis();
  //mpu6050->update();
  //theta = mpu6050->get_yaw();
  
  ping->update();
  //servo_r->update(90);
  //servo_L->update(90);
  //hall->read();
  btSerial->update(shared_mem);

  ping_distance = ping->get_distance();

  //scout();
  drive(servo_L, servo_r);

}

void Platoon_Type_11::scout()
{
  //Serial.println(shared_mem[0]);
  
  if(shared_mem[0]==1)
  {
    int x = 90;
    int rando;
    led->set(1,1,1);
    
    servo_r->update(90+x);
    servo_L->update(90-x);
  
  
    if(ping_distance <100)
    {
      x = 5;
  
      servo_r->update(90-x);
      servo_L->update(90+x);
  
      delay(2000);
      
      rando = random(0,2);
  
      if(rando == 0)
      {
        servo_r->update(90+x);
        servo_L->update(90+x);
      } else if (rando == 1)
      {
        servo_r->update(90-x);
        servo_L->update(90-x);
      }
  
      delay(1000);
  
    servo_r->update(90);
    servo_L->update(90);
    delay(200);
    
    }
  } else if(shared_mem[0]==0)
  {
    servo_r->update(90);
    servo_L->update(90);
    led->set(0,0,0);
  }
}

void Platoon_Type_11::drive(HS422* servo_L, HS422* servo_r)
{
  servo_r->update(90+shared_mem[0]);
  servo_L->update(90-shared_mem[1]);
}

Platoon_Type_11::~Platoon_Type_11()
{
  //delete mpu6050;
  delete ping;
  delete servo_r;
  delete servo_L;
  delete led;
  delete hall;
  delete btSerial;
}
