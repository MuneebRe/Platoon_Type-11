#include <Arduino.h>

class HC_SR04
{
  private:
  int trigPin, echoPin;
  double duration, mm;

  public:
  HC_SR04(int trigPin, int echoPin);
  void update();
  double get_distance(){return mm;}
};
