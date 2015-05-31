#include "bhoreal_Slave.h"
#include <Wire.h>
// Auxiliary analog output definitions
#define VBAT A0 //BATERIA
#define GATE_CMD 15
#define VUSB A2
#define CHG 17
#define BOT 2

#define CHANNEL 4 //Channel for i2c

//Constant parameters
#define VCC 4463  //Voltage in mV
#define BAT_MIN 3700  //Voltage minimun
#define BAT_CH  2000  //Voltage retry recharge
#define RESOLUTION 1023.  //Reslution for ADC
    
BhorealSlave Bhoreal__;

void sleepNow()         // here we put the arduino to sleep
{   
    delay(100);
    cli();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
  
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

}


void BhorealSlave::sleep()         // here we put the arduino to sleep
{
  Bhoreal__.timer1Stop();
  
  PORTB |= B00011001;
  PORTD |= B11111000;
  
  
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()

int request = 0;
void i2cEvent(int howMany)
{
  if (Wire.available()) // loop through all but the last
  {
    byte var = Wire.read(); // receive byte as a character
    request = 0;
//    Serial.println(var);
    switch (var) 
      {
        
        case 1:
          Bhoreal__.timer1Initialize();
          break;
        case 2:
          Bhoreal__.sleep();
          sleepNow();
          break;
        case 3:
          if (analogRead(VUSB)>500) request = 1;
          break;
        case 4:
          digitalWrite(CHG, HIGH);
          break;
        case 5:
          digitalWrite(CHG, LOW);
          break;
//        default: 
//          // si nada coincide, ejecuta el "default"
//          // el "default" es opcional
      }
  }
}

void requestEvent()
{
  Wire.write(request); // as expected by master
}

unsigned long time_charge;

void BhorealSlave::begin()
  {

//    pinMode(GATE_CMD, INPUT); //15, PC1
//    pinMode(CHG, OUTPUT);  //17, PC3
    DDRC |= B00001000;  // 17  OUTPUT
    DDRC &= B11111110;  // 15 INPUT
    
    pinMode(BOT, INPUT); //2, PD2
    DDRD &= B11111011;  // 2 INPUT
    
    if (readBattery() > 2000) PORTC &= B11110111; //digitalWrite(CHG, LOW);
    else PORTC |= B00001000; //digitalWrite(CHG, HIGH);
    
    PORTC &= B11111110; //digitalWrite(GATE_CMD, LOW);

    DDRB  |= B00011001;
    DDRD  |= B11111000;
    
    PORTB |= B00011001;
    PORTD |= B11111000;

    
    Serial.begin(57600);  
        
    time_charge = millis();
    
    Wire.begin(CHANNEL);      // join i2c bus with address #4
       
    Wire.onReceive(i2cEvent); // register event
    Wire.onRequest(requestEvent); // register event
  }

boolean charge_state = true;

void BhorealSlave::chargeBatterry()
  {
    if (analogRead(VUSB)>500)
      {
        float charge = readBattery();
        if (charge < 2000)
          {
             digitalWrite(CHG, HIGH);
             if ((millis() - time_charge)>60000) 
             {
               time_charge = millis();
               digitalWrite(CHG, LOW);
             }
             
          }
        else digitalWrite(CHG, LOW);
        if ((charge<(BAT_MIN - 700))&&(charge_state)&&(charge>2000))/*&&(!digitalRead(GATE_CMD))*/
         { 
           charge_state = false; 
           sleep();
         } 
        else if (((charge>BAT_MIN)&&(!charge_state))||(charge<2000))
         {
           charge_state = true;
         }
      }
    else digitalWrite(CHG, HIGH);
  }


boolean fullcolor = false;
unsigned long time_full;

void BhorealSlave::check()
  {       
    /* Modo full color pasados 2 segundos de seguridad*/
    if (digitalRead(GATE_CMD))
     {
       if ((millis()-time_full) >= 2000)
         {
           fullcolor = true;
         }
     }
    else
     { 
       time_full = millis();
       fullcolor = false;
     }
    ////////////////////////////////////////////////////
  }

float BhorealSlave::readBattery()
  {
    return analogRead(VBAT)*(VCC/RESOLUTION);
  }
  
#define RESOLUTION 65536    // Timer1 is 16 bit


void BhorealSlave::timer1SetPeriod(long microseconds)		// AR modified for atomic access
{
  long cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
  
  oldSREG = SREG;				
  cli();							// Disable interrupts for 16 bit register access
  ICR1 = pwmPeriod = cycles;                                          // ICR1 is TOP in p & f correct pwm mode
  SREG = oldSREG;
  
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                          // reset clock select register, and starts the clock
}


void BhorealSlave::timer1Initialize()
{
    TCCR1A = 0;                 // clear control register A 
    TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
    timer1SetPeriod(5); 
    TIMSK1 = _BV(TOIE1);                                     
}

void BhorealSlave::timer1Stop()
{
  TIMSK1 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit 
}

byte var = 0;
ISR(TIMER1_OVF_vect)
{
  if (fullcolor == true)
  {
    PORTB &= B11100110;
    PORTD &= B00000111;
  }
  else 
  {
    switch (var) {
    case 0:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTB &= B11111110;
      var = 1;
      break;
    case 1:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTB &= B11110111;
      var = 2;
      break;
    case 2:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTB &= B11101111;
      var = 3;
      break;
    case 3:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTD &= B11110111;
      var = 4;
      break;
    case 4:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTD &= B11101111;
      var = 5;
      break;
    case 5:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTD &= B11011111;
      var = 6;
      break;
    case 6:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTD &= B10111111;
      var = 7;
      break;
    case 7:
      PORTB |= B00011001;
      PORTD |= B11111000;
      PORTD &= B01111111;
      var = 0;
      break;
    }
  } 
}

void BhorealSlave::masterSend(byte val) {
   Wire.beginTransmission(3); //start transmission to device 
   Wire.write(val);        // write value to write
   Wire.endTransmission(); //end transmission
}
