#include "feinstaub.h"

SdsDustSensor sds(Serial2);

const unsigned int getwarm = 30000;
unsigned long prevtime = 0;
bool sds011iswarm, gettingwarm, sdssleeping;

void getfinedust() {
    unsigned long currentMillis = millis();  // Aktuelle Zeit in Millisekunden

    if (sds011iswarm == false && gettingwarm == false && sdssleeping == false) {
        sds.wakeup();
        gettingwarm = true;
        prevtime = currentMillis;  // Zeit des Weckens speichern
        Serial.println("Sensor is awake and getting warm");
    } else if (gettingwarm == true) {
        if (currentMillis - prevtime >= getwarm){
            sds011iswarm = true;
            gettingwarm = false;
            Serial.println("Sensor is now warm and can start to messure");
        }
    } else if (sds011iswarm == true){
        Serial.println("Zeit für Messung ist erreicht");
        sds011iswarm = false;
        PmResult pm = sds.queryPm();
        if (pm.isOk()) {
            Serial.print("PM2.5 = ");
            Serial.print(pm.pm25);
            Serial.print(", PM10 = ");
            Serial.println(pm.pm10);
            Serial.println(pm.toString());
        } else {
            Serial.print("Could not read values from sensor, reason: ");
            Serial.println(pm.statusToString());
        }

        // Jetzt sensor schlafen legen
        WorkingStateResult state = sds.sleep();
        if (state.isWorking()) {
            Serial.println("Problem with sleeping the sensor.");
        } else {
            Serial.println("Sensor is sleeping");
            sdssleeping = true;
            delay(10000);
        }  
    }
    else {
        Serial.println("SDS011 is sleeping");
    }
}
