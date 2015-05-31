#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>

class BhorealSlave {
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
    void timer1SetPeriod(long microseconds);
    //Variables for timer1
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;
    char oldSREG;                   // To hold Status 

};
