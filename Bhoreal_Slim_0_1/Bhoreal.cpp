
#include "bhoreal.h"
#include <EEPROM.h>

#if MODEL == SLIMPRO
  const char mySSID[] = "hangar_lab";  
  const char myPass[] = "labinteractius";
  const char *IP = "172.26.255.255";
  const char myAuth[] = WPA2;
  const int protocol = UDP;
  
//  const char mySSID[] = "DI&L";  
//  const char myPass[] = "vdossier";
//  const char *IP = "172.26.0.255";
  
//  const char mySSID[] = "Mi$Red";  
//  const char myPass[] = "FINALFANTASY";
//  const char *IP = "192.168.0.255";
//  const char myAuth[] = WPA2;
  
  const char antenna[] = INT_ANT;
  const uint16_t outPort = 8000;
  const uint16_t localPort = 9000;   
  
#endif

#define MESSAGE_SIZE 36
char OSC_SEND[MESSAGE_SIZE] = { // Message template
  '/', 'b', 'h', 'o',
  'r', 'e', 'a', 'l',
  '/', 'p', 'r', 'e',   
  's', 's',  B0,  B0,
  ',', 'i', 'i', 'i', 
  B0 , B0 ,  B0,  B0,
  B0 , B0 ,  B0,  B0,
  B0 , B0 ,  B0,  B0,
  B0 , B0 ,  B0,  B0
};

byte tempR;
byte tempC;
byte lastread;
byte command = 0;
boolean ready = true;
uint16_t IntensityMAX = 255;

// Default draw colour. Each channel can be between 0 and 255.
int red = 0;
int green = 0;
int blue = 0;

// Auxiliary analog output definitions
#define ANALOG0 A5 // SLIDER POT MINI
#define ANALOG1 A1
boolean adc[2] = { // On or off state analog inputs
  1, 0};
byte analogval[2]; //The last reported value
byte tempADC; //Temporary storage for comparison purposes

  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
    // Pin definitions for the 74HC164 SIPO shift register (drives button rows high)
    #define DATAPIN     9 // aka analog pin 2 (what, you didn't know that analog pins 0-5 are also digital pins 14-19? Well, now you do!)
    #define CLOCKPIN    8
    // Pin definitions for the 74HC165 PISO shift register (reads button column state)
    #define INDATAPIN  13
    #define INCLOCKPIN  5
    #define INLOADPIN  10 // toggling this tell the 165 to read the value into its memory for reading
    
    #define VBAT  A0 
    #define FACTORY  A5 
    #define AWAKE  22 // AWAKE WIFLY
    #define RST 11
    #define MUX 12
    #define BOT 7  // sleep pushbotom
    #define POWER_VCC 4
    
    #define CHANNEL 3 //Channel for i2c
    
    //Accelerometer parameters
    #define lim 512
    #define DEVICE 0x53    //ADXL345 device address
    #define TO_READ 6        //num of bytes we are going to read each time (two bytes for each axis)
    int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
    
    byte mode = 0;
    
    uint16_t MAX   =  8; 
    int NUM_LEDS   =  64; 
    const uint16_t numBytes = 192;
    #define PIN_LED 6
    
    //Constant parameters
    #define VCC_BATTERY 4618  //Voltage in mV
    #define BAT_MIN 3300  //Voltage minimun
    #define RESOLUTION 1023.  //Reslution for ADC
  #else
    uint16_t MAX   =  4; 
    int NUM_LEDS   =  16;  
    const uint16_t numBytes = 48;
    #define PIN_LED 11
    byte row[4]    = {         // ROW pins for matrix pushbottons 
      13, 5, 10, 9};
    byte column[4] = {         // COL pins for matrix pushbottons
      8, 6, 12, 4};
  #endif
  
  Bhoreal Bhoreal_;
  uint8_t pixels[numBytes];
  uint32_t baud[3]={ 115200, 57600, 9600};
  
boolean pressed[8][8] = {      // pushbottons states matrix
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1}
};

const byte remapMini[4][4] =    // mapping matrix for Mini
{
  { 3, 4, 11, 12 }, 
  { 2, 5, 10, 13 }, 
  { 1, 6,  9, 14 }, 
  { 0, 7,  8, 15 } 
};

uint8_t GIR = 0;

byte remapSlim[4][8][8] = 
{
  {             // mapping matrix for Slim
    {7,8,23,24,       39,40,55,56}, 
    {6,9,22,25,       38,41,54,57}, 
    {5,10,21,26,      37,42,53,58}, 
    {4,11,20,27,      36,43,52,59}, 
    {3,12,19,28,      35,44,51,60}, 
    {2,13,18,29,      34,45,50,61}, 
    {1,14,17,30,      33,46,49,62}, 
    {0,15,16,31,      32,47,48,63}, 
  },
  {             // mapping matrix for Slim
    {56,57,58,59,      60,61,62,63}, 
    {55,54,53,52,      51,50,49,48}, 
    {40,41,42,43,      44,45,46,47}, 
    {39,38,37,36,      35,34,33,32}, 
    {24,25,26,27,      28,29,30,31}, 
    {23,22,21,20,      19,18,17,16}, 
    {8,9,10,11,        12,13,14,15}, 
    {7,6,5,4,              3,2,1,0}, 
  },
  {             // mapping matrix for Slim
    {63,48,47,32,       31,16,15,0}, 
    {62,49,46,33,       30,17,14,1}, 
    {61,50,45,34,       29,18,13,2}, 
    {60,51,44,35,       28,19,12,3}, 
    {59,52,43,36,        27,20,11,4}, 
    {58,53,42,37,       26,21,10,5}, 
    {57,54,41,38,        25,22,9,6}, 
    {56,55,40,39,        24,23,8,7}, 
  },
  {             // mapping matrix for Slim
    {0,1,2,3,              4,5,6,7}, 
    {15,14,13,12,        11,10,9,8}, 
    {16,17,18,19,      20,21,22,23}, 
    {31,30,29,28,      27,26,25,24}, 
    {32,33,34,35,      36,37,38,39}, 
    {47,46,45,44,      43,42,41,40}, 
    {48,49,50,51,      52,53,54,55}, 
    {63,62,61,60,      59,58,57,56}, 
  }
};


void Bhoreal::slaveSend(byte val) {
   Wire.beginTransmission(4); //start transmission to device 
   Wire.write(val);        // write value to write
   Wire.endTransmission(); //end transmission
}

byte Bhoreal::slaveRead(byte reg) 
  {
    unsigned long time_on = millis();
    slaveSend(reg);
    Wire.requestFrom(4, 1);
    while(!Wire.available()&&((millis()-time_on)<5000)) {slaveSend(reg); Wire.requestFrom(4, 1); delay(4);}//Apaga atmega328    // slave may send less than requested
    byte c = 0;
    while (Wire.available()) c = Wire.read(); // receive byte as a character
    return c;
  }
//////////////////////////////////////////////////////////////////////
//////////////////////       BHOREAL BEGIN      //////////////////////
//////////////////////////////////////////////////////////////////////


void Bhoreal::begin()
{
  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
     
    DDRB |= B11110000;  // 11,10,9,8 OUTPUT
    DDRC |= B01000000;  // 5  OUTPUT
    DDRC &= B01111111;  // 13 INPUT
    DDRE &= B10111111;  // 7 INPUT
    DDRD |= B11010000;  // 12, 6, 4 OUTPUT
    DDRF |= B00000011;  // A4, A5   OUTPUT
    PORTF |= B00000010; //A4 HIGH
    PORTF &= B11111110; //A5 LOW
    PORTD |= B01010000; // 12, 4  HIGH
    PORTD &= B01111111; //6 LOW
    PORTB &= B01111111; //11 LOW, Reset atmega328 ON
    PORTB |= B10000000; // 11 HIGH

    //Inicializa como encendido o apagado
    Wire.begin();    
    slaveRead(3);  //Check slave ON
    //Gestion de sleep del Bhoreal
    #if (MODEL == SLIMPRO) 
      #if ENERGY_CONTROL
        if ((EEPROM.read(0)>0)||((readBattery()<BAT_MIN)&&(readBattery()>2000)))
          {
            EEPROM.write(0, 0);       
            slaveSend(2); //Apaga atmega328
            sleep(); 
            sleepNow();
          }
        else 
      #endif
        {
          EEPROM.write(0, 1);   
          slaveSend(1); //Activa atmega328
        } 
      // Start the serial port   
      Serial.begin(57600); //USB inicializado a 57600 
    #else
      #if (MODEL == SLIM) 
        PORTD &= B11101111; //digitalWrite(POWER_VCC, LOW);
        slaveSend(1); //Activa atmega328
      #endif
    #endif
    
    port = portOutputRegister(digitalPinToPort(PIN_LED));
    pinMask = digitalPinToBitMask(PIN_LED);
    
    // Start the wifi
    #if (MODEL == SLIMPRO)
      Serial1.begin(baud[0]); //WIFI inicializado a 9600
      #if ENERGY_CONTROL
        if ((readBattery()<2000)||(slaveRead(3)>0)) //Low battery level or usb connected
          {
            PORTD &= B11101111; //digitalWrite(POWER_VCC, LOW);
            WIFIsleep();
          }
        else if (readBattery()>BAT_MIN)
      #else
      if (true)
      #endif
        {
          awake();
          BaudSetup();
          if (Connect()) 
            {
              Serial.println("Conectado!!");
                int report = checkWiFly();
                if (report == 1) Serial.println(F("Wifly Updated."));
                else if (report == 2) Serial.println(F("Update Fail."));
                else if (report == 0) Serial.println(F("WiFly up to date."));
                else if (report == -1) Serial.println(F("Error reading the wifi version."));
                Serial.print("MAC: "); Serial.println(getMAC()); 
                Serial.print("IP: "); Serial.println(getIP()); 
            }
          else Serial.println("Desconectado :(");
        }
      else 
        {
          sleep();
        } 
       timer1Initialize();  
    #endif
    
    show();

    writeTo(DEVICE, 0x2D, 0x08);
//  writeTo(DEVICE, 0x31, 0x00); //2g
//  writeTo(DEVICE, 0x31, 0x01); //4g
    writeTo(DEVICE, 0x31, 0x02); //8g
//  writeTo(DEVICE, 0x31, 0x03); //16g
    AttachInterrupt6(RISING); //Cambio de 0 a 1
  #else
    for(byte i = 0; i<4; i++) 
    {
      pinMode(column[i], INPUT);
      pinMode(row[i], OUTPUT);
      digitalWrite(row[i], LOW);
    }
    // Start the serial port
    #if SERIAL_ENABLE
      Serial.begin(BAUD);
    #endif    
    // Setup the timer interrupt
    PORTE |= B01000000;
    DDRE  |= B01000000;
    timer1Initialize();  
    
  #endif
}



////////////////////////////////////////////////////////////////
//////////////////////       STARTUP      //////////////////////
////////////////////////////////////////////////////////////////

// Run this animation once at startup. Currently unfinished.
void Bhoreal::startup(){
  
  for(int x = 0; x < NUM_LEDS; ++x){ 
    #if (MODEL == SLIM) || (MODEL == SLIMPRO)
      uint32_t c = hue2rgb(x*2);  // 128 HUE steps / 64 leds, 2 steps x led
      uint8_t
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
      b = (uint8_t)c;
      setPixelColor(remapSlim[GIR][x>>3][x%8], r, g, b);
    #else
      uint32_t c = hue2rgb(x*8); // 128 HUE steps / 16 leds, 8 steps x led
      uint8_t
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
      b = (uint8_t)c;  
      setPixelColor(remapMini[x>>2][x%4], r, g, b);
    #endif
  }  
  show();
}

//////////////////////////////////////////////////////////////////////
//////////////////////  SERIAL PRESS & RELEASE  //////////////////////
//////////////////////////////////////////////////////////////////////


void Bhoreal::on_press(byte r, byte c){

#if SERIAL_DATA
  Serial.print(1);
  Serial.print(" ");
  Serial.println( (r << 4) | c, HEX);
#endif
  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
    MIDIEvent e1 = { 0x09, 0x90, ((r << 3) | c) , 64  };
    #if (MODEL == SLIMPRO)
      #if WIFI_SEND
        WIFISend(r, c, 1);
      #endif
    #endif
//    MIDIEvent e1 = {0x09, 0x90, ((c << 3) | r) , 64    };
  #else
    MIDIEvent e1 = { 0x09, 0x90, ((r << 2) | c) , 64  };
//    MIDIEvent e1 = {0x09, 0x90, ((c << 2) | r) , 64    };
  #endif
//  MIDIUSB.write(e1);

}

void Bhoreal::on_release(byte r, byte c){
  
#if SERIAL_DATA
  Serial.print(0);
  Serial.print(" ");
  Serial.println( (r << 4) | c, HEX); 
#endif
  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
    //MIDIEvent e1 = { 0x09, 0x90, ((r << 3) | c) , 0  };
    #if (MODEL == SLIMPRO)
      #if WIFI_SEND
        WIFISend(r, c, 0);
      #endif
    #endif
//    MIDIEvent e1 = {0x08, 0x80, ((c << 3) | r) , 0    };
  #else
    MIDIEvent e1 = { 0x09, 0x90, ((r << 2) | c) , 0  };
//    MIDIEvent e1 = {0x08, 0x80, ((c << 2) | r) , 0    };
  #endif

//  MIDIUSB.write(e1);

}



///////////////////////////////////////////////////////////////
//////////////////////   CHECK BUTTONS   //////////////////////
///////////////////////////////////////////////////////////////


byte count_column = 0;
byte count_file = 0;
unsigned long time_button = 0;

void Bhoreal::checkButtons(){
  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
//      if (mode == 0) checkMatrix();
//      else if (mode == 1) programMode();
    switch (mode) {
      case 0:
        checkMatrix();
        break;
      case 1:
        programMode();
        break;
      case 2:
        demoAccel();
        break;
      case 3:
        white();
        break;
      }
  #else
    if ((millis()-time_button)>1)
      {
        time_button = millis();
        for(int r= MAX - 1; r >= 0; r--)
        {
          if(pressed[count_column][r] != digitalRead(column[r]))
          { // read the state
            //delay(1); // to prevent bounces!!!
            pressed[count_column][r] = digitalRead(column[r]);
            if(pressed[count_column][r]) on_press(count_column, r);
            else on_release(count_column, r);
          }
        }
        digitalWrite(row[count_column],LOW);
        count_column++;
        if (count_column>MAX) count_column=0;
        digitalWrite(row[count_column],HIGH);
        
      }
  #endif
}

byte r[8] = {4, 5, 6, 7, 0, 1, 2, 3};
boolean in_mat = 0;

void Bhoreal::checkMatrix()
{
   PORTB &= B11101111; //digitalWrite(CLOCKPIN,LOW);
   PORTB |= B00100000; //digitalWrite(DATAPIN, HIGH); 
   for(byte c = 0; c < MAX; c++)
     {
      PORTB |= B00010000; //digitalWrite(CLOCKPIN, HIGH);
    
      PORTB &= B10111111; //digitalWrite(INLOADPIN, LOW); // read into register
      delayMicroseconds(1);
      PORTB |= B01000000; //digitalWrite(INLOADPIN, HIGH); // done reading into register, ready for us to read
      
      for(int i= 0; i < MAX; i++){ // read each of the 165's 4 inputs (or its snapshot of it rather)
        // tell the 165 to send the first inputs pin state
        PORTC &= B10111111; //digitalWrite(INCLOCKPIN, LOW);
        // read the current output
        if(pressed[c][r[i]] != PINC>>7){ //digitalRead(INDATAPIN)){ // read the state
          pressed[c][r[i]] = PINC>>7; //digitalRead(INDATAPIN);
          if(!pressed[c][r[i]]){
            on_press(c, r[i]);
          }
          else {
            on_release(c, r[i]);
          }
        }
        // tell the 165 we are done reading the state, the next inclockpin=0 will output the next input value
         PORTC |= B01000000; //digitalWrite(INCLOCKPIN, HIGH);
      }
      
      PORTB &= B11101111; //digitalWrite(CLOCKPIN,LOW);
      PORTB &= B11011111; //digitalWrite(DATAPIN, LOW);  
     }
}

////////////////////////////////////////////////////////////////
//////////////////////    REFRESH LED     //////////////////////
////////////////////////////////////////////////////////////////

byte refresh_led = 0;
unsigned long time_led = 0;

void Bhoreal::refresh(){ 
      if (((refresh_led>0)&&((millis()-time_led)>1))||(refresh_led>=NUM_LEDS)||((refresh_led<NUM_LEDS)&&((millis()-time_led)>25))&&(refresh_led>0))
        {
          refresh_led=0;
          show();
        }
}



////////////////////////////////////////////////////////////////
////////////////////// REFRESH MIDI & LED  /////////////////////
////////////////////////////////////////////////////////////////

void Bhoreal::midiRefresh(){ 
  while(MIDIUSB.available() > 0) 
  {
    MIDIEvent e;
    e = MIDIUSB.read();

#if SERIAL_ENABLE
    if(MIDI_DEBUG)
    {
      if(e.type != 0x0F) // timestamp 1 BYTE
      {
        Serial.print("Midi Packet: ");
        Serial.print(e.type);
        Serial.print("\t");
        Serial.print(e.m1);
        Serial.print("\t");
        Serial.print(e.m2);
        Serial.print("\t");
        Serial.println(e.m3);
      }
    }
#endif
#if (MODEL == SLIM) || (MODEL == SLIMPRO)
    if((e.type == 0x09) && (e.m3))  // NoteON midi message with vel > 0
    {  
      uint32_t c = hue2rgb(e.m3);  // velocity is used to HUE color selection and HUE is converted to RGB uint32 
      uint8_t
        r = (uint8_t)(c >> 16),
        g = (uint8_t)(c >>  8),
        b = (uint8_t)c;

      setPixelColor(remapSlim[GIR][e.m2>>3][e.m2%8], r, g, b);
      refresh_led++;
      time_led = millis();
      //refresh();
    }
    else if( (e.type == 0x08) || ((e.type == 0x09) && !e.m3) ) // NoteOFF midi message
    {  
      setPixelColor(remapSlim[GIR][e.m2>>3][e.m2%8], 0, 0, 0);
      refresh_led++;
      time_led = millis();
      //refresh();
    }  
#else
    if((e.type == 0x09) && (e.m3))  //  NoteON midi message with vel > 0
    {  
      uint32_t c = hue2rgb(e.m3);
      uint8_t
        r = (uint8_t)(c >> 16),
        g = (uint8_t)(c >>  8),
        b = (uint8_t)c;

      setPixelColor(remapMini[e.m2>>2][e.m2%4], r, g, b);
      refresh_led++;
      time_led = millis();
    }
    else if( (e.type == 0x08) || ((e.type == 0x09) && !e.m3) ) // NoteOFF midi message
    {  
      setPixelColor(remapMini[e.m2>>2][e.m2%4], 0, 0, 0);
      refresh_led++;
      time_led = millis();
    }
#endif
    MIDIUSB.flush(); // delete it???
  
  }
}

////////////////////////////////////////////////////////////////
//////////////////////  CHECK ADC INPUTS  //////////////////////
////////////////////////////////////////////////////////////////

#if (MODEL == SLIM) || (MODEL == SLIMPRO)
      float Bhoreal::readBattery()
        {
          return analogRead(VBAT)*(VCC_BATTERY/RESOLUTION);
        }
        
      //---------------- Functions
      //Writes val to address register on device
      void Bhoreal::writeTo(int device, byte address, byte val) {
         Wire.beginTransmission(device); //start transmission to device 
         Wire.write(address);        // write register address
         Wire.write(val);        // write value to write
         Wire.endTransmission(); //end transmission
      }
      
      //reads num bytes starting from address register on device in to buff array
      void Bhoreal::readFrom(int device, byte address, int num, byte buff[]) {
        Wire.beginTransmission(device); //start transmission to device 
        Wire.write(address);        //writes address to read from
        Wire.endTransmission(); //end transmission
        
        Wire.beginTransmission(device); //start transmission to device
        Wire.requestFrom(device, num);    // request 6 bytes from device
        
        int i = 0;
        while(Wire.available())    //device may write less than requested (abnormal)
        { 
          buff[i] = Wire.read(); // read a byte
          i++;
        }
        Wire.endTransmission(); //end transmission
      }
      
      int x=0;
      int y=0;
      int z=0;     
      byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
#endif

void Bhoreal::checkADC(){
  
  #if (MODEL == SLIM) || (MODEL == SLIMPRO)   
          checkBattery();
          readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
          x = (((int)buff[1]) << 8) | buff[0]; 
          x = map(x,-lim,lim,0,1023);  
          y = (((int)buff[3])<< 8) | buff[2];
          y = map(y,-lim,lim,0,1023); 
          z = (((int)buff[5]) << 8) | buff[4];
          z = map(z,-lim,lim,0,1023); 
          int limitx = map(x, 448, 575, 7, 0);
          if (limitx==0) limitx =1;
          int limity = map(y, 448, 575, 0, 7);
          if (limity==7) limity=6;
          if (limitx<2) GIR=2;
          else if (limitx>5) GIR=0;
          else if (limity<2) GIR=1;
          else if (limity>5) GIR=3;
          
          #if SERIAL_DATA
            Serial.print("x= ");
            Serial.print(x);
            Serial.print(", ");
            Serial.print("y= ");
            Serial.print(y);
            Serial.print(", ");
            Serial.print("z= ");
            Serial.println(z);
          #endif
          
  #else
      // For all of the ADC's which are activated, check if the analog value has changed,
      // and send a message if it has.
      if(adc[0]){
        tempADC = (analogRead(ANALOG0) >> 2);
        if(abs((int)analogval[0] - (int)tempADC) > 2 ){
        //if(analogval[0] != (analogRead(ANALOG0) >> 2)){
          //analogval[0] = (analogRead(ANALOG0) >> 2);
          analogval[0] = tempADC;
    #if SERIAL_DATA
          Serial.write(14 << 4);
          Serial.write(analogval[0]);
    #endif
          // Send the control change message for the slider potentiometer,
          // by defect we use CC64 controller
          MIDIEvent e1 = {0x0B, 0xB0, 64, analogval[0]>>1};
          MIDIUSB.write(e1);
        }
      }
      
      if(adc[1]){
        if(analogval[1] != (analogRead(ANALOG1) >> 2)){
          analogval[1] = (analogRead(ANALOG1) >> 2);
    #if SERIAL_DATA
          Serial.write(14 << 4 | 1);
          Serial.write(analogval[1]);
    #endif
        }
      }
  #endif
}

#if (MODEL == SLIM) || (MODEL == SLIMPRO)
  boolean charge_state = false;
  float charge = 0;
  
  void Bhoreal::checkBattery()
      {
        #if (MODEL == SLIMPRO)
          #if ENERGY_CONTROL
            charge = readBattery();
            if ((charge<(BAT_MIN - 300))&&(charge>2000)&&(!charge_state))
            {        
              charge_state = true;
              slaveSend(2); //Duerme atmega328
              sleep();
              sleepNow();
            }
            else if (((charge>BAT_MIN)||(charge<2000))&&(charge_state))
                     {
                       charge_state = false;
                       slaveRead(1); //Activa atmega328
                       awake();
                     }
          #endif
        #endif
      }
#endif

///////////////////////////////////////////////////////////////
//////////////////////  TIMERS SETTINGS  //////////////////////
///////////////////////////////////////////////////////////////

#if (MODEL == MINI)

#define RESOLUTION 65535    // Timer1 is 16 bit
unsigned int pwmPeriod;
unsigned char clockSelectBits;
char oldSREG;					// To hold Status 

void setPeriodTimer1(long microseconds)		// AR modified for atomic access
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
  ICR1 = pwmPeriod = cycles;                                    // ICR1 is TOP in p & f correct pwm mode
  SREG = oldSREG;

  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                    // reset clock select register, and starts the clock
}


void Bhoreal::timer1Initialize()
{
  TCCR1A = 0;                 // clear control register A 
  TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  setPeriodTimer1(5);         // Time interruption in ms
  TIMSK1 = _BV(TOIE1);                                  
}
 
ISR(TIMER1_OVF_vect)
{
       if (PORTE&B01000000) PORTE &= B10111111;
       else PORTE |= B01000000;
}

#endif
///////////////////////////////////////////////////////////////
//////////////////////     HUE -> RGB    //////////////////////
///////////////////////////////////////////////////////////////

uint32_t Bhoreal::hue2rgb(uint16_t hueValue)
{
  
  uint8_t r;
  uint8_t g;
  uint8_t b;
  hueValue<<= 3;  // 128 midi steps -> 1024 hue steps
  
  if (hueValue < 341)  { // Lowest third of the potentiometer's range (0-340)
    hueValue = (hueValue * 3) / 4; // Normalize to 0-255

    r = 255 - hueValue;  // Red from full to off
    g = hueValue;        // Green from off to full
    b = 1;               // Blue off
  }
  else if (hueValue < 682) { // Middle third of potentiometer's range (341-681)
    hueValue = ( (hueValue-341) * 3) / 4; // Normalize to 0-255

    r = 1;              // Red off
    g = 255 - hueValue; // Green from full to off
    b = hueValue;       // Blue from off to full
  }
  else  { // Upper third of potentiometer"s range (682-1023)
    hueValue = ( (hueValue-683) * 3) / 4; // Normalize to 0-255

    r = hueValue;       // Red from off to full
    g = 1;              // Green off
    b = 255 - hueValue; // Blue from full to off
  }
  
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;

}

///////////////////////////////////////////////////////////////
//////////////////////     WIFI     ///////////////////////////
///////////////////////////////////////////////////////////////

#if MODEL == SLIMPRO
 boolean FindInResponse(const char *toMatch,
                                    unsigned int timeOut = 1000) {
  int byteRead;

  unsigned long timeOutTarget; // in milliseconds

  for (unsigned int offset = 0; offset < strlen(toMatch); offset++) {
    timeOutTarget = millis() + timeOut; // Doesn't handle timer wrapping
    while (!Serial1.available()) {
      // Wait, with optional time out.
      if (timeOut > 0) {
        if (millis() > timeOutTarget) {
          return false;
        }
      }
      delay(1); // This seems to improve reliability slightly
    }
    byteRead = Serial1.read();
    //Serial.print((char)byteRead);
    delay(1); // Removing logging may affect timing slightly

    if (byteRead != toMatch[offset]) {
      offset = 0;
      // Ignore character read if it's not a match for the start of the string
      if (byteRead != toMatch[offset]) {
        offset = -1;
      }
      continue;
    }
  }

  return true;
}

boolean SendCommand(const __FlashStringHelper *command,
                                 boolean isMultipartCommand = false,
                                 const char *expectedResponse = "AOK") {
  Serial1.print(command);
  delay(20);
  if (!isMultipartCommand) {
    Serial1.flush();
    Serial1.println();

    // TODO: Handle other responses
    //       (e.g. autoconnect message before it's turned off,
    //        DHCP messages, and/or ERR etc)
    if (!FindInResponse(expectedResponse, 3000)) {
      return false;
    }
    //sckFindInResponse(expectedResponse);
  }
  return true;
}

boolean SendCommand(const char *command,
                                 boolean isMultipartCommand = false,
                                 const char *expectedResponse = "AOK") {
  Serial1.print(command);
  delay(20);
  if (!isMultipartCommand) {
    Serial1.flush();
    Serial1.println();

    // TODO: Handle other responses
    //       (e.g. autoconnect message before it's turned off,
    //        DHCP messages, and/or ERR etc)
    if (!FindInResponse(expectedResponse, 3000)) {
      return false;
    }
    //findInResponse(expectedResponse);
  }
  return true;
}

#define COMMAND_MODE_ENTER_RETRY_ATTEMPTS 2

#define COMMAND_MODE_GUARD_TIME 250 // in milliseconds

boolean EnterCommandMode() {
    for (int retryCount = 0; retryCount < COMMAND_MODE_ENTER_RETRY_ATTEMPTS; retryCount++) 
     {
      delay(COMMAND_MODE_GUARD_TIME);
      Serial1.print(F("$$$"));
      delay(COMMAND_MODE_GUARD_TIME);
      Serial1.println();
      Serial1.println();
      if (FindInResponse("\r\n<", 1000))
      {
        return true;
      }
    }
    return false;
}

boolean Reset() {
      EnterCommandMode();
      SendCommand(F("factory R"), false, "Set Factory Defaults"); // Store settings
      SendCommand(F("save"), false, "Storing in config"); // Store settings
      SendCommand(F("reboot"), false, "*READY*");
}

boolean ExitCommandMode() {
    for (int retryCount = 0; retryCount < COMMAND_MODE_ENTER_RETRY_ATTEMPTS; retryCount++) 
     {
      if (SendCommand(F("exit"), false, "EXIT")) 
      {
      return true;
      }
    }
    return false;
}

void SkipRemainderOfResponse(unsigned int timeOut) {
  unsigned long time = millis();
  while (((millis()-time)<timeOut))
  {
    if (Serial1.available())
      { 
        byte temp = Serial1.read();
        //Serial.write(temp);
        time = millis();
      }
  }
}

void Bhoreal::WIFIsleep() {
      EnterCommandMode();
      SendCommand(F("sleep"));
}

static char buffer[32];

char* itoa(int32_t number)
  {
   byte count = 0;
   uint32_t temp;
   if (number < 0) {temp = number*(-1); count++;} 
   while ((temp/10)!=0) 
   {
     temp = temp/10;
     count++;
   }
   int i;
   if (number < 0) {temp = number*(-1);} 
   else temp = number;
   for (i = count; i>=0; i--) 
   { 
     buffer[i] = temp%10 + '0'; 
     temp = temp/10; 
   }
   if (number < 0) {buffer[0] = '-';} 
   buffer[count + 1] = 0x00;
   return buffer;   
  }
  
void Bhoreal::sleep() {
      digitalWrite(POWER_VCC, HIGH); 
}

boolean Bhoreal::awake() {
      PORTD &= B11101111; //digitalWrite(POWER_VCC, LOW); 
      //delay(500);
      writeTo(DEVICE, 0x2D, 0x08);
  //  writeTo(DEVICE, 0x31, 0x00); //2g
  //  writeTo(DEVICE, 0x31, 0x01); //4g
      writeTo(DEVICE, 0x31, 0x02); //8g
  //  writeTo(DEVICE, 0x31, 0x03); //16g
}

boolean Ready()
{
  if (EnterCommandMode())
    {
      Serial1.println(F("join"));
      if (FindInResponse("Associated!", 8000)) 
      {
        SkipRemainderOfResponse(3000);
        ExitCommandMode();
        return(true);
      }
   } 
  else return(false);
}

boolean Bhoreal::Connect()
  {
    if (!Ready())
    {
      if(EnterCommandMode())
        {    
            SendCommand(F("set wlan join 1")); // Disable AP mode
            SendCommand(F("set ip dhcp 1")); // Enable DHCP server
            SendCommand(F("set ip proto "), true);
            SendCommand(itoa(protocol));
            SendCommand(F("set ip host "), true);
            SendCommand(IP);
            SendCommand(F("set ip localport "), true);
            SendCommand(itoa(localPort));
            SendCommand(F("set ip remote "), true);
            SendCommand(itoa(outPort));
            SendCommand(F(DEFAULT_WIFLY_FTP_UPDATE)); //ftp server update
            SendCommand(F("set ftp mode 1"));
            SendCommand(F("set wlan auth "), true);
            SendCommand(myAuth);
            boolean mode = true;
            if ((myAuth==WEP)||(myAuth==WEP64)) mode=false;
            Serial.print(myAuth);
            SendCommand(F("set wlan ssid "), true);
            SendCommand(mySSID);
            Serial.print(F(" "));
            Serial.print(mySSID);
            if (mode) SendCommand(F("set wlan phrase "), true);  // WPA1, WPA2, OPEN
            else SendCommand(F("set wlan key "), true);
            SendCommand(myPass);
            Serial.print(F(" "));
            Serial.print(myPass);
            SendCommand(F("set wlan ext_antenna "), true);
            SendCommand(antenna);
            Serial.print(F(" "));
            Serial.println(antenna);
            SendCommand(F("save"), false, "Storing in config"); // Store settings
            SendCommand(F("reboot"), false, "*READY*");
            if (Ready()) return true;
        }
        return false;   
    }
     else return true;  
  }
  
  boolean Bhoreal::reset() {
    EnterCommandMode();
    SendCommand(F("factory R"), false, "Set Factory Defaults"); // Store settings
    SendCommand(F("save"), false, "Storing in config"); // Store settings
    SendCommand(F("reboot"), false, "*READY*");
  }

  int Bhoreal::checkWiFly() {
    int ver = getWiFlyVersion();
    if (ver > 0)
    {
      if (ver < WIFLY_LATEST_VERSION)
       {
        int state = 1;
        if(update()); //Wifly Updated.
        else state = 2; //Update Fail.
        reset();
        BaudSetup();
        return state;
       }   
      else return 0; //WiFly up to date.
    }
    else return -1; //Error reading the wifi version.
  }
  
  #define MAC_ADDRESS_BUFFER_SIZE 18 // "FF:FF:FF:FF:FF:FF\0"
  
  char* Bhoreal::getMAC() {
    if (EnterCommandMode()) 
    {
      if (SendCommand(F("get mac"), false, "Mac Addr="))
      {
        char newChar;
        byte offset = 0;
  
        while (offset < MAC_ADDRESS_BUFFER_SIZE) {
          if (Serial1.available())
          {
            newChar = Serial1.read();
            //Serial.println(newChar);
            if ((newChar == '\n')||(newChar < '0')) {
              buffer[offset] = '\x00';
              break;
            } 
            else if (newChar != -1) {
              buffer[offset] = newChar;
              offset++;
            }
          }
        }
        buffer[MAC_ADDRESS_BUFFER_SIZE-1] = '\x00';
        ExitCommandMode();
      }        
    }
  
    return buffer;
  }
  
  char* Bhoreal::getIP() {
    if (EnterCommandMode()) 
    {
      if (SendCommand(F("get ip"), false, "IP="))
      {
        char newChar;
        byte offset = 0;
  
        while (offset < MAC_ADDRESS_BUFFER_SIZE) {
          if (Serial1.available())
          {
            newChar = Serial1.read();
            //Serial.println(newChar);
            if (newChar == '\n') {
              buffer[offset] = '\x00';
              break;
            } 
            else if (newChar != -1) {
              buffer[offset] = newChar;
              offset++;
            }
          }
        }
        buffer[MAC_ADDRESS_BUFFER_SIZE-1] = '\x00';
        ExitCommandMode();
      }        
    }
  
    return buffer;
  }
  
  int Bhoreal::getWiFlyVersion() {
    if (EnterCommandMode()) 
    {
      if (SendCommand(F("ver"), false, "wifly-GSX Ver"))
      {
        char newChar;
        byte offset = 0;
        boolean prevWasNumber = false;
        while (offset < 3) {
          if (Serial1.available())
          {
            newChar = Serial1.read();
            if ((newChar != -1 && isdigit(newChar)) || newChar == '.') {
              if (newChar != '.') {
                buffer[offset] = newChar;
                offset++;
              }
              prevWasNumber = true;
            } 
            else {
              if (prevWasNumber){
                break;
              }
              prevWasNumber = false; 
            }
          }
        }
        ExitCommandMode();
        buffer[offset] = 0x00;
       return atoi(buffer);
      } 
      return 0;
    }
    return 0;
  }
  
  
  boolean Bhoreal::update() {
    if (EnterCommandMode())
    {
      SendCommand(F(DEFAULT_WIFLY_FIRMWARE));
      delay(1000);
      if (FindInResponse("FTP OK.", 60000))
      {
        return true;
      }
    }
    else return false;
  }
  
  void Bhoreal::BaudSetup()
  {
    if(!EnterCommandMode())
    {
      boolean repair = true;
      for (int i=0; ((i<3)&&repair); i++)
      {
        Serial1.begin(baud[i]);
        if(EnterCommandMode()) 
        {
          SendCommand(F("set u b "), true);
          SendCommand(itoa(baud[0]));
          SendCommand(F("save"), false, "Storing in config"); // Store settings
          SendCommand(F("reboot"), false, "*READY*");
          repair = false;
          Serial.println("Baudrate actualizado!!");
        }
        Serial1.begin(baud[0]);
      }
    }
  }

  boolean Bhoreal::WIFISend(byte r, byte c, boolean state)
  {
    Serial1.write((state<<7)|((r << 3) | c));
  }

#endif

///////////////////////////////////////////////////////////////
//////////////////////    Interrupccion 6    //////////////////////
///////////////////////////////////////////////////////////////  


//Esta interrupcion no esta soportada por la libreria arcore

#if (MODEL == SLIM) || (MODEL == SLIMPRO)
  void Bhoreal::AttachInterrupt6(int mode)
  {
      EICRB = (EICRB & ~((1<<ISC60) | (1<<ISC61))) | (mode << ISC60);
      EIMSK |= (1<<INT6);
  }
  
  void Bhoreal::detachInterrupt6()
  {
      EIMSK &= ~(1<<INT6);
  }
  
  void Bhoreal::sleepNow()         // here we put the arduino to sleep
  {    
      delay(100);
      cli();
  
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
      
      
      sleep_enable();          // enables the sleep bit in the mcucr register
                               // so sleep is possible. just a safety pin
    
      sleep_cpu();            // here the device is actually put to sleep!!
                               // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
   
      //sleep_disable();         // first thing after waking from sleep:
                               // disable sleep...
   
  }
  
  unsigned long time_mode = 0;
  
  ISR(INT6_vect) {
   if ((millis()-time_mode)>500)
     {
       mode++;
       if (mode > 3) mode = 0;
       time_mode = millis();
     }
  }
  
  ///////////////////////////////////////////////////////////////
  //////////////////////    Program mode   //////////////////////
  ///////////////////////////////////////////////////////////////    
  
  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
  void Bhoreal::programMode()
  {
    unsigned long time = 0;
    boolean flagprog = false;
    Serial1.begin(57600);
    digitalWrite(MUX, LOW);
    for(int x = 0; x < NUM_LEDS; ++x) setPixelColor(x, 0, 0, 0);
    setPixelColor(0, 255, 0, 0);
    show();
    int limit_ant = 0;
    int limit = 1;
    while (mode==1)
      {
        #if (MODEL == SLIMPRO)
         float charge = readBattery();
         limit = map(charge, 3000, 4200, 0, 7);
         if (limit!=limit_ant)
          {
            for(int x = 0; x <=limit; ++x) setPixelColor(x, 255, 0, 0);
            if (limit>=3) for(int x = 3; x <=limit; ++x) setPixelColor(x, 0, 0, 255);
            if (limit>=6) for(int x = 6; x <=limit; ++x) setPixelColor(x, 0, 255, 0);
            show();
            limit_ant = limit;
          }
        #endif
         if (Serial.available())
          {
            if (flagprog)
            {
              digitalWrite(RST, LOW);
              delay(1);
              digitalWrite(RST, HIGH);
              flagprog = false;
            }
            Serial1.write(Serial.read());
  
            time = millis();
          }
        if (Serial1.available())
          Serial.write(Serial1.read());
        if ((millis()- time)>=1000) flagprog = true;
     }
     digitalWrite(MUX, HIGH);
     Serial1.begin(9600);
  }
#endif
///////////////////////////////////////////////////////////////
//////////////////////       Demo        //////////////////////
///////////////////////////////////////////////////////////////  

void Bhoreal::demoAccel()
{          
  for (int i=0; i<8; i++) 
    {
      for (int j=0; j<8; j++)  
        {
          setPixelColor(remapSlim[GIR][i][j], 0, 0, 255);
        }
    } 
  int limitx = map(x, 448, 575, 7, 0);
  if (limitx==0) limitx =1;
  int limity = map(y, 448, 575, 0, 7);
  if (limity==7) limity=6;
  setPixelColor(remapSlim[0][limitx][limity], 255, 0, 0);
  setPixelColor(remapSlim[0][limitx-1][limity], 255, 0, 0);
  setPixelColor(remapSlim[0][limitx][limity+1], 255, 0, 0);
  setPixelColor(remapSlim[0][limitx-1][limity+1], 255, 0, 0);
  show();
}

#endif

void Bhoreal::white()
{          
  for(int x = 0; x < NUM_LEDS; ++x) setPixelColor(x, 255, 255, 255);
  show();  
}






///////////////////////////////////////////////////////////////
//////////////////////    Control led    //////////////////////
///////////////////////////////////////////////////////////////  

void Bhoreal::show(void) {

  if(!pixels) return;

  // Data latch = 50+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while((micros() - endTime) < 50L);
  // endTime is a private member (rather than global var) so that mutliple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  noInterrupts(); // Need 100% focus on instruction timing
  
  volatile uint16_t i   = numBytes; // Loop counter
  volatile uint8_t  *ptr = pixels;   // Pointer to next byte
  volatile uint8_t  b   = *ptr++;   // Current byte value
  volatile uint8_t  hi;             // PORT w/output bit set high
  volatile uint8_t  lo;             // PORT w/output bit set low
    
  // WS2811 and WS2812 have different hi/lo duty cycles; this is
  // similar but NOT an exact copy of the prior 400-on-8 code.

  // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
  // ST instructions:         ^   ^        ^       (T=0,5,13)

  volatile uint8_t next, bit;

  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;
  bit  = 8;

  asm volatile(
   "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
    "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
    "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
     "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
    "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
    "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
    "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
    "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
    "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
    "nop"                      "\n\t" // 1    nop           (T = 13)
    "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
    "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
   "nextbyte20:"               "\n\t" //                    (T = 10)
    "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
    "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
    "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
     "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

  interrupts();
  endTime = micros(); // Save EOD time for latch on next call
}

// Set pixel color from separate R,G,B components:
void Bhoreal::setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if(n < NUM_LEDS) {
    uint8_t *p = &pixels[n * 3];
    *p++ = g;
    *p++ = r; 
    *p = b;
  }
}

/*TIMER*/

#if (MODEL == SLIMPRO)
  
static char buffer_int[64];
byte count_char = 0;
byte count_char2 = 0;

//  boolean Bhoreal::addData(byte inByte)
//    {
//      if ((inByte == '/')&& (count_char2==0))
//        {
//           buffer_int[count_char] = inByte;
//           count_char = 1;
//           count_char2 = 1;
//           return false;
//        }
//      else if((count_char==(MESSAGE_SIZE-1))||(count_char2>2))
//        {
//          buffer_int[count_char] = inByte;
//          buffer_int[count_char + 1] = 0x00;
//          count_char = 0;
//          count_char2 = 0;
//          return true;
//        }
//      else
//        {
//          if (inByte == '/') count_char2 = count_char2 + 1;
//          buffer_int[count_char] = inByte;
//          count_char = count_char + 1;
//        } 
//      return false;
//  }
  
//    char* Bhoreal::getIP() {
//    if (EnterCommandMode()) 
//    {
//      if (SendCommand(F("get ip"), false, "IP="))
//      {
//        char newChar;
//        byte offset = 0;
//  
//        while (offset < MAC_ADDRESS_BUFFER_SIZE) {
//          if (Serial1.available())
//          {
//            newChar = Serial1.read();
//            //Serial.println(newChar);
//            if (newChar == '\n') {
//              buffer[offset] = '\x00';
//              break;
//            } 
//            else if (newChar != -1) {
//              buffer[offset] = newChar;
//              offset++;
//            }
//          }
//        }
//        buffer[MAC_ADDRESS_BUFFER_SIZE-1] = '\x00';
//        ExitCommandMode();
//      }        
//    }
//  
//    return buffer;
//  }
  
  boolean offsetWIFI = 0;
  byte inByte = 0;
  byte ledNumber = 0;
  
  void Bhoreal::WIFIRead()
    {
     while (Serial1.available())
      {
        inByte = Serial1.read();
        if (inByte>127) 
          {
                offsetWIFI = true; 
                ledNumber = inByte&0x7F;
          }
        else if ((inByte<127)&&(offsetWIFI==true))
          {
            uint32_t c = hue2rgb(inByte);  // velocity is used to HUE color selection and HUE is converted to RGB uint32 
            uint8_t
              r = (uint8_t)(c >> 16),
              g = (uint8_t)(c >>  8),
              b = (uint8_t)c;
      
            setPixelColor(remapSlim[GIR][ledNumber>>3][ledNumber%8], r, g, b);
            refresh_led++;
            time_led = millis();
            offsetWIFI = false; 
          }
      }
    }

  void Bhoreal::serialRequests()
  {
    sei();
    Bhoreal_.timer1Stop();
    WIFIRead();
    Bhoreal_.timer1Initialize(); // set a timer of length 1000000 microseconds (or 1 sec - or 1Hz)  
  }

  ISR(TIMER1_OVF_vect)
  {
    Bhoreal_.serialRequests();
  }
  
  #define RESOLUTION 65536    // Timer1 is 16 bit
  unsigned int pwmPeriod;
  unsigned char clockSelectBits;
  char oldSREG;					// To hold Status 
  
  void Bhoreal::timer1SetPeriod(long microseconds)		// AR modified for atomic access
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
  
  void Bhoreal::timer1Initialize()
  {
    TCCR1A = 0;                 // clear control register A 
    TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
    timer1SetPeriod(20); 
    TIMSK1 = _BV(TOIE1);                                  
  }
  
  void Bhoreal::timer1Stop()
  {
    TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
    TIMSK1 &= ~(_BV(TOIE1)); 
    
  }
#endif



