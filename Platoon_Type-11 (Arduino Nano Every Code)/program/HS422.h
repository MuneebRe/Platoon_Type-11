#include <Arduino.h>
#include <Servo.h>

class HS422
{
  protected:
  Servo* myservo;

  public:
  HS422(int pin);
  update(int servo_write);
  ~HS422();
};
