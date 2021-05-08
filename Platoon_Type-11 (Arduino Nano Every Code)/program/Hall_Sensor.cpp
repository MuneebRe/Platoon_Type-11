#include <Arduino.h>
#include "Hall_Sensor.h"

Hall_Sensor::Hall_Sensor(int pin)
{
  this->pin = pin;
}

void Hall_Sensor::read()
{
  sensor_value = analogRead(pin);
  Serial.println(sensor_value);
}
