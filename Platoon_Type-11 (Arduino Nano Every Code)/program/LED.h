
class LED
{
  private:
  int rPin, gPin, bPin;
  int RGB[3];
  
  public:
  LED(int rPin, int gPin, int bPin);
  void set(bool r, bool g, bool b);
  
};
