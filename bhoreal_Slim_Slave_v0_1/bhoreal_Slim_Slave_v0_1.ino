
//////////////////////////////////////////////////////////////
// BHOREAL SLIM SLAVE
//////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "Bhoreal_Slave.h"
#include <Wire.h>

Bhoreal Bhoreal;

void setup() {
  Bhoreal.begin();
}
  
void loop () {
  
  Bhoreal.check();
  //Bhoreal.chargeBatterry();
}


