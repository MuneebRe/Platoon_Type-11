
#include "Platoon_Type_11.h"

Platoon_Type_11* PT11;

void setup() {
  Serial.begin(115200);
  PT11 = new Platoon_Type_11(0, 0, 0);
  PT11->begin();
  
}

void loop() {
  PT11->update();

}
