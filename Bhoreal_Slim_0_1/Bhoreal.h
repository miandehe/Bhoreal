#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define MINISLIM  0  //Tamaño de la matriz
#define SLIM  1  //Tamaño de la matriz
#define SLIMPRO  2  //Tamaño de la matriz

#define  SERIAL_ENABLE     false
#define  MIDI_DEBUG        false
#define  SERIAL_DATA       false
#define  DEMO_ACCEL        true
#define  ENERGY_CONTROL    false
#define  WIFI_SEND         true

#define OPEN  "0"
#define WEP   "1"
#define WPA1  "2"
#define WPA2  "4"
#define WEP64 "8"

#define UDP  1
#define TCP  2

#define EXT_ANT "1" // antena externa
#define INT_ANT "0" // antena interna

#define WIFLY_LATEST_VERSION 441
#define DEFAULT_WIFLY_FIRMWARE "ftp update wifly3-441.img"
#define DEFAULT_WIFLY_FTP_UPDATE "set ftp address 198.175.253.161"

#define EE_ADDR_TIME_VERSION                        0   //32BYTES 
#define EE_ADDR_POWER                              32   //1BYTE 

#define buffer_length        32
static char buffer[buffer_length];

//#define  MODEL  MINISLIM //Modelo
#define  MODEL  SLIMPRO //Modelo
//#define  MODEL  SLIM //Modelo

class Bhoreal {
  public:  
  
    // Constructor: number of LEDs, pin number, LED type
    void show(void);
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
    
    void begin();
    void config();
    boolean compareData(char* text, char* text1);
    void writeData(uint32_t eeaddress, char* text);
    char* readData(uint16_t eeaddress);
    void checkButtons();
    void refresh();
    void midiRefresh();
    void startup();
    float readBattery();
    void checkADC();
    void checkBattery();
    uint32_t hue2rgb(uint16_t hueValue);
    boolean awake();
    void slaveSend(byte val);
    byte slaveRead(byte reg);
    void sleep();
    void sleepNow();
    
    //WIFI
    boolean Connect();
    boolean reConnect();
    void WIFIsleep();
    boolean WIFISend(byte r, byte c, boolean state);
    boolean reset();
    int checkWiFly();
    char* getMAC();
    char* getIP();
    int getWiFlyVersion();
    boolean update();
    void BaudSetup();
    
    void WIFIRead();
    
    void timer1Initialize();
    void timer1SetPeriod(long microseconds);
    void timer1Stop();
    void serialRequests();
    
    
  private:
  
    uint32_t endTime;                     // Latch timing reference
    const volatile uint8_t *port;         // Output PORT register
    uint8_t pinMask;                // Output PORT bitmask    
    
    void checkMatrix();
    void demoAccel();
    void white();
    void programMode();
    void on_press(byte r, byte c);
    void on_release(byte r, byte c);

    void AttachInterrupt6(int mode);
    void detachInterrupt6();
    //Accelerometer
    void writeTo(int device, byte address, byte val);
    void readFrom(int device, byte address, int num, byte buff[]);
};
