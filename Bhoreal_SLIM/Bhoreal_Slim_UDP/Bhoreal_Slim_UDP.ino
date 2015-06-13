//
// BHOREAL SLIM / MINI BETA FIRMWARE v0.1            //
// Programmed by M. A. de Heras y A. Posada          //
// Thanks to Adafruit for the great library Neopixel //
//                                                   // 
///////////////////////////////////////////////////////

#include <Arduino.h>
#include <BhorealSlim.h>
#include <Wire.h>
#include <EEPROM.h>

BhorealSlim Bhoreal;

char mySSID[] = "Hello_pepe";  
char myPass[] = "labinteractius";
byte myAuth = 2; //WPA2
char *IPHOST = "192.168.1.255";
int protocol = 1; //UDP
uint16_t outPort = 8000;  // DESTINATION UDP PORT
uint16_t localPort = 9000;   // LOCAL UDP PORT

void setup() {
  Bhoreal.begin(true);
  
  Bhoreal.wifiBegin(mySSID, myPass, myAuth, IPHOST, protocol, outPort, localPort);
  // Run the startup animation
  Bhoreal.startup();
}
  
void loop () {
 
  Bhoreal.checkButtonsUDP();

  // Check and report the ADC states, if necessary
  Bhoreal.checkAccel();
  
  Bhoreal.displayRefresh();

}


