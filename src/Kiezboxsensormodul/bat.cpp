#include "bat.h"
#include <Arduino.h>

int batMvolt;
#define batpin 3

void bat_read(){
    int batvolt = analogRead(batpin);
    int batvolt1 = analogReadMilliVolts(3);
    batMvolt = ((batvolt1 * 1480)/1000);
    Serial.print("Spannung: ");
    Serial.println(batMvolt);

}