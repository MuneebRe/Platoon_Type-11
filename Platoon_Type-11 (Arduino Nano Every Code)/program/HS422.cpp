#include <Arduino.h>
#include "HS422.h"
#include <Servo.h>


HS422::HS422(int pin)
{
  myservo = new Servo;
  myservo->attach(pin);
  myservo->write(90);
}

HS422::update(int servo_write)
{
    myservo->write(servo_write);
}

HS422::~HS422()
{
  delete myservo;
}
