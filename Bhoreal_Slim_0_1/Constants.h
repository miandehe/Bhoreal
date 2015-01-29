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

#define SELECTOR 0  // ???????
#define MIDI  2


#define EXT_ANT "1" // External Antenna
#define INT_ANT "0" // Internal Antenna

/* 

WIFLY Firmware Setting

*/

#if MODEL == SLIMPRO
//  const char mySSID[] = "Miguel";  
//  const char myPass[] = "FINALFANTASY";
//  const char *IPHOST = "192.168.0.255";
//  const byte myAuth = WPA2;
  
//  const char mySSID[] = "hangar_lab";  
//  const char myPass[] = "labinteractius";
//  const char *IPHOST = "172.26.255.255";
//  const byte myAuth = WPA2;
  
//  const char mySSID[] = "hangar_oficines";  
//  const char myPass[] = "m1cr0fug4s";
//  const char *IPHOST = "172.26.255.255";
//  const byte myAuth = WPA2;
  
//  const char mySSID[] = "Hello_pepe";  // SSID EXTERNAL WIFI
//  const char myPass[] = "labinteractius";   // PASSWORD EXTERNAL WIFI
//  const char *IPHOST = "192.168.0.255"; // IP destination AP MODE
//  const byte myAuth = WPA2;
//  
  const char mySSID[] = "mid";  // SSID EXTERNAL WIFI
  const char myPass[] = "";   // PASSWORD EXTERNAL WIFI
  const char *IPHOST = "192.168.0.255"; // IP destination AP MODE
  const byte myAuth = OPEN;
  
  const char mySSIDAP[] = "bhoreal";  // SSID AP MODE
  const char myPassAP[] = "";         // PASSWORD AP MODE
  const char *IPHOST_AP = "192.168.0.255"; // IP destination AP MODE
  const byte myAuthAP = OPEN;
   
  const int protocol = UDP; // UDP DEFAULT
  const char antenna[] = INT_ANT; // INTERNAL ANTENNA
  const uint16_t outPort = 8000;  // DESTINATION UDP PORT
  const uint16_t localPort = 9000;   // LOCAL UDP PORT
  
  // WIFLY FIRMWARE AUTO UPDATE AND VERSIONS
  #define WIFLY_LATEST_VERSION 441
  #define DEFAULT_WIFLY_FIRMWARE "ftp update wifly3-441.img"
  #define DEFAULT_WIFLY_FTP_UPDATE "set ftp address 198.175.253.161"
#endif



/* 

ARDUINO ports definitions - GPIOS and ADCs 

*/

  #if (MODEL == SLIM) || (MODEL == SLIMPRO)
    // Pin definitions for the 74HC164 SIPO shift register (drives button rows high)
    #define DATAPIN     9 
    #define CLOCKPIN    8
    // Pin definitions for the 74HC165 PISO shift register (reads button column state)
    #define INDATAPIN  13
    #define INCLOCKPIN  5
    #define INLOADPIN  10 // toggling this tell the 165 to read the value into its memory for reading
    
    #define VBAT  A0      // Battery State
    #define FACTORY  A5   // FACTORY RESET Wifly
    #define AWAKE  22     // AWAKE Wifly
    #define RST 11        // RESET ATMEGA2???? 
    #define MUX 12        // MUX????
    #define BOT 7         // sleep pushbotom
    #define POWER_VCC 4   // Power control PMOS ??????
    
    #define CHANNEL 3 // Channel for i2c  ????? eliminar?????
    
    //Accelerometer parameters
    #define lim 512         //Central position of the values for each axis (acel = 0) 
    #define DEVICE 0x53     //ADXL345 device address
    #define TO_READ 6       //num of bytes we are going to read each time (two bytes for each axis)
    int regAddress = 0x32;  //first axis-acceleration-data register on the ADXL345
    
    int mode = 1; // mode = MIDI, UDP, TCP, ACEL, .... (1 = MIDI BY DEFAULT)
    boolean charge_on = true; // CHARGER Activated
    int WIFIMode = NORMAL;    // Normal Mode by default (connection to external Wifi network)
    uint16_t NUM_ROWS =  8; 
    int NUM_LEDS   =  64; 
    const uint16_t numBytes = 192;
    #define PIN_LED 6    // ??????
    
    //Constant parameters
    #define VCC_BATTERY 4618  //Maximum Voltage in mV
    #define BAT_MIN 3300  // Minimum Voltage in mV
    #define RESOLUTION 1023.  //Resolution for ADC

  #else // MINI SLIM

    #define PIN_LED 11  // ??????

    // Auxiliary analog output definitions
    #define ANALOG0 A5 // SLIDER POT MINI
    #define ANALOG1 A1 // ?????    
    boolean adc[2] = { 1, 0 }; // ON or OFF state of the analog inputs
    byte analogval[2]; //The Slider Pot Sensor
    byte tempADC; //Temporary storage for comparison purposes

    uint16_t NUM_ROWS = 4; 
    int NUM_LEDS   =  16;  
    const uint16_t numBytes = 48;
    byte row[4]    = {         // ROW pins for matrix pushbottons 
      13, 5, 10, 9};
    byte column[4] = {         // COL pins for matrix pushbottons
      8, 6, 12, 4};
  #endif
  
  
/* 

Internal EEPROM Memory Addresses

*/ 

#define EE_ADDR_TIME_VERSION        0   //32BYTES 
#define EE_ADDR_POWER               32   //1BYTE 

static char* WEB[4]={                  
                  "apps.ioapps.net",
//                  "GET /xavidev/bhoreal/notifications.php HTTP/1.1 \n",
                  "GET /xavidev/bhoreal/notifications.php?user=miguel HTTP/1.1 \n",
                  "Host: apps.ioapps.net \n",
                  "User-Agent: Bhoreal \n\n"  
                  };
                  
uint8_t character[10][8] =    // LCD codes for the 0-9 numbers
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
    
  
