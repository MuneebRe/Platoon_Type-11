#include "HC-SR04.h"

HC_SR04::HC_SR04(int trigPin, int echoPin)
{
  this->trigPin = trigPin;
  this->echoPin = echoPin;
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void HC_SR04::update()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  mm = 10*((duration/2) / 29.1);     // Divide by 29.1 or multiply by 0.0343

  /*
  Serial.print(mm,4);
  Serial.print("mm");
  Serial.println();
  */
  
  delay(250);
}
