/* 

Bhoreal models

*/

#define MINISLIM  0  //Tamaño de la matriz
#define SLIM  1  //Tamaño de la matriz
#define SLIMPRO  2  //Tamaño de la matriz

/* 

WIFI modes

*/
#define NORMAL 0
#define AP     1
#define PROG_NORMAL 2
#define PROG_AP     3
/* 

WIFI AND SERVER STATICS - WiFly, Http server parameters.

*/
// WiFly Auth Modes
#define OPEN   0
#define WEP    1
#define WPA2   2
#define WPA1   3
#define MIXED  4
#define WEP64  8

#define UDP  1
#define TCP  2
#define HTML 16

#define EXT_ANT "1" // External Antenna
#define INT_ANT "0" // Internal Antenna

/* 

WIFLY Firmware Setting

*/

#if MODEL == SLIMPRO
//  const char mySSID[] = "Miguel";  
//  const char myPass[] = "FINALFANTASY";
//  const char *IP = "192.168.0.255";
//  const byte myAuth = WPA2;
  
//  const char mySSID[] = "hangar_lab";  
//  const char myPass[] = "labinteractius";
//  const char *IP = "172.26.255.255";
//  const byte myAuth = WPA2;
  
//  const char mySSID[] = "hangar_oficines";  
//  const char myPass[] = "m1cr0fug4s";
//  const char *IP = "172.26.255.255";
//  const byte myAuth = WPA2;
  
  const char mySSID[] = "Mi$Red";  
  const char myPass[] = "FINALFANTASY";
  const char *IP = "192.168.0.255";
  const byte myAuth = WPA2;
  
  const char mySSIDAP[] = "bhoreal";  
  const char myPassAP[] = "";
  const char *IPAP = "192.168.0.255";
  const byte myAuthAP = OPEN;
  
//  #if ServerMode
//    const int protocol = HTML + TCP;
//  #else  
    const int protocol = UDP;
//  #endif
  const char antenna[] = INT_ANT;
  const uint16_t outPort = 8000;
  const uint16_t localPort = 9000;   
  
#endif

#define WIFLY_LATEST_VERSION 441
#define DEFAULT_WIFLY_FIRMWARE "ftp update wifly3-441.img"
#define DEFAULT_WIFLY_FTP_UPDATE "set ftp address 198.175.253.161"

/* 

ARDUINO ports definitions - GPIOS and ADCs 

*/

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
    
    int mode = 1;
    int WIFIMode = NORMAL;
    uint16_t MAX   =  8; 
    int NUM_LEDS   =  64; 
    const uint16_t numBytes = 192;
    #define PIN_LED 6
    
    //Constant parameters
    #define VCC_BATTERY 4618  //Voltage in mV
    #define BAT_MIN 3300  //Voltage minimun
    #define RESOLUTION 1023.  //Reslution for ADC
  #else
    // Auxiliary analog output definitions
    #define ANALOG0 A5 // SLIDER POT MINI
    #define ANALOG1 A1
    boolean adc[2] = { // On or off state analog inputs
      1, 0};
    byte analogval[2]; //The last reported value
    byte tempADC; //Temporary storage for comparison purposes
    uint16_t MAX   =  4; 
    int NUM_LEDS   =  16;  
    const uint16_t numBytes = 48;
    #define PIN_LED 11
    byte row[4]    = {         // ROW pins for matrix pushbottons 
      13, 5, 10, 9};
    byte column[4] = {         // COL pins for matrix pushbottons
      8, 6, 12, 4};
  #endif
  
  
/* 

Internal EEPROM Memory Addresses

*/ 

#define EE_ADDR_TIME_VERSION                        0   //32BYTES 
#define EE_ADDR_POWER                              32   //1BYTE 

static char* WEB[4]={                  
                  "apps.ioapps.net",
                  "GET /xavidev/bhoreal/notifications.php HTTP/1.1 \n",
                  "Host: apps.ioapps.net \n",
                  "User-Agent: Bhoreal \n\n"  
                  };
                  
uint8_t character[10][8] =
  {{ B11111000, //0
     B10001000,
     B10001000,
     B10001000,
     B10001000,
     B10001000,
     B10001000,
     B11111000},
   { B00100000, //1
     B00100000,
     B00100000,
     B00100000,
     B00100000,
     B00100000,
     B00100000,
     B00100000},
   { B11111000, //2
     B00001000,
     B00001000,
     B00001000,
     B11111000,
     B10000000,
     B10000000,
     B11111000},
   { B11111000, //3
     B00001000,
     B00001000,
     B00001000,
     B11111000,
     B00001000,
     B00001000,
     B11111000},
   { B10001000, //4
     B10001000,
     B10001000,
     B10001000,
     B11111000,
     B00001000,
     B00001000,
     B00001000},
   { B11111000, //5
     B10000000,
     B10000000,
     B10000000,
     B11111000,
     B00001000,
     B00001000,
     B11111000},
   { B11111000, //6
     B10000000,
     B10000000,
     B10000000,
     B11111000,
     B10001000,
     B10001000,
     B11111000},
   { B11111000, //7
     B00001000,
     B00001000,
     B00001000,
     B00001000,
     B00001000,
     B00001000,
     B00001000},
   { B11111000, //8
     B10001000,
     B10001000,
     B10001000,
     B11111000,
     B10001000,
     B10001000,
     B11111000},
   { B11111000, //9
     B10001000,
     B10001000,
     B10001000,
     B11111000,
     B00001000,
     B00001000,
     B00001000}};
    
  
