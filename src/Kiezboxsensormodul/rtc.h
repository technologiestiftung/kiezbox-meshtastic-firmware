#ifndef RTC_H
#define RTC_H

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>

extern String timeString, dateString, dateAndTime;
extern float rtc_temp;

void set_time();


#endif
