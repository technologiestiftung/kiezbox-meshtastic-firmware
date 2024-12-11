#include <Arduino.h>
#include "feinstaub.h"
#include "rtc.h"
#include "bme680.h"
#include "memsmicro.h"
#include "bat.h"

//const int ve = 36;

#define I2C_SDA 45
#define I2C_SCL 46


//int myFunction(int, int);

void setup() {
  Serial.begin(9600);
   analogReadResolution(12);
  //pinMode(ve, OUTPUT);
  //digitalWrite(ve, LOW); //LOW switch it on
  Serial2.begin(9600, SERIAL_8N1, 48, 47);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
  Serial.println(sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode
  //delay(1000);
  // put your setup code here, to run once:
//  int result = myFunction(2, 3);
}

void loop() {

  //bat_read();
  //tell_me_the_noise();
  //readbme680();
  //set_time();
  getfinedust();
  if(sdssleeping == true){
    Serial.println("Jetzt aktivieren wir es wieder");
    bat_read();
    tell_me_the_noise();
    readbme680();
    set_time();
    delay(5000);
    sdssleeping = false;
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

