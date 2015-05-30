#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>

class Bhoreal {
  public:  
    void begin();
    void check();
    void timer1Initialize();
    void timer1Stop();
    float readBattery();
    void chargeBatterry();
    void sleep();
    void masterSend(byte val);
  private:


};
