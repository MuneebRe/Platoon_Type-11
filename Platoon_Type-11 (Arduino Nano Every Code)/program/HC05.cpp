#include <Arduino.h>

#include "HC05.h"

HC05::HC05(int rxPin, int txPin)
{
  this->rxPin = rxPin;
  this->txPin = txPin;

  btSerial = new SoftwareSerial(rxPin,txPin);
  delay(1000);
  btSerial->begin(115200);
  btSerial->setTimeout(100);
  previousMillis = 0;
  interval = 500;
  ledB = "";
  ledState = LOW;
}

void HC05::update(int shared_mem[])
{

  /*
  if (btSerial->available() > 0) {    // check if bluetooth module sends some data to esp8266
    char data = btSerial->read();  // read the data from HC-05
    switch (data)
    {
      case 'B':         // if receive data is 'B'
        ledB = "blink";   // write the string
        break;
      case 'S':              // if receive data is 'S'
        ledB = "stop";
        break;
      default:
        break;
    }
  }
  unsigned long currentMillis = millis();
  if (ledB == "blink") {          // if received data is 'B' the start blinking 
    Serial.println("blinking started");
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(12, ledState);
    }
  }
  */
  
  /*
  if (btSerial->available() > 0) {    // check if bluetooth module sends some data to esp8266
      char data = btSerial->read();  // read the data from HC-05

      switch(data)
      {
        case 'A':
        shared_mem[0] = 1;
        break;
        case 'a':
        shared_mem[0] = 0;
      }
  }
  */

  //GordoN's Code
  /*
      switch(buffer_in[0])
      {
        case 'O':
        shared_mem[0] = 1;
        break;
        case 'P':
        shared_mem[0] = 0;
        break;
      }
      //Serial.readBytes(buffer_in,n);
      */
      
      /*
      p_buffer = buffer_in;
      p = p_buffer;
      
      pc = (char*)p;
      
      shared_mem[0] = (int)*pi;
      p += sizeof(char);

      pc = (char*)p;
      shared_mem[1] = (int)*pi;
      */

      //THIS LINE WAS WORKING LAST TIME
      
      //n = 2;
      btSerial->readBytes(buffer_in,3);

      shared_mem[0] = buffer_in[0];
      shared_mem[1] = buffer_in[1];

      if(buffer_in[3] == 's' || abs(shared_mem[0]) > 40 || abs(shared_mem[1] > 40))
      {
        shared_mem[0] = 0;
        shared_mem[1] = 0;
      }
      
      Serial.print(shared_mem[0]);
      Serial.print("  ");
      Serial.println(shared_mem[1]);

      //delay(100);
       
      /*
      p_buffer = buffer_in;
      p = p_buffer;

      pi = (int *)p;
      shared_mem[0] = *pi;
      Serial.println(*pi);
      Serial.println(share_mem[0]);
      //if (buffer_in[0] == 'O') shared_mem[0] = 1;
      //if (buffer_in[0] == 'P') shared_mem[0] = 0;
      */

      
      
  /*if( buffer_in[0] == 's' )
  {
    t0 = micros()*1.0e-6;

    while(1) {
      
      // clock time since the beginning of the program
      t = micros()*1.0e-6 - t0;
      y = sin(t);
      
  //    Serial.print("\nt = ");
      btSerial->print(t,5); // print out time to 3 decimal places
      
      // for a csv file that can be read by a spreadsheet
      btSerial->print(",");
      
      btSerial->print(y,5);
  
      btSerial->print("\n");
  
      // print out time once every 10 ms seconds (100 Hz)
      delay(10);
      
      // stop save serial program after 10 s
      if( t > 1 ) {
        // output save serial termination character
        btSerial->print("#");
        break;
      }
  }
  
    delay(1000);
  }
  */

  
}

HC05::~HC05()
{
  delete btSerial;
}
