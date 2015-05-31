
//////////////////////////////////////////////////////////////
// BHOREAL SLIM SLAVE
//////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "Bhoreal_Slave.h"
#include <Wire.h>

BhorealSlave Bhoreal;

void setup() {
  Bhoreal.begin();
}
  
void loop () {
  
  Bhoreal.check();
  //Bhoreal.chargeBatterry();
}


