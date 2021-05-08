
class Hall_Sensor
{
  private:
  int pin;
  int sensor_value;
  bool state;

  public:
  Hall_Sensor(int pin);
  void read();
};
