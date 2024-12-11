#ifndef FEINSTAUB_H
#define FEINSTAUB_H

#include <HardwareSerial.h>
#include "SdsDustSensor.h"

extern SdsDustSensor sds;

extern bool sdssleeping;

void getfinedust();

#endif
