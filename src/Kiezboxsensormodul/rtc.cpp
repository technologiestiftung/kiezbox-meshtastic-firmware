#include "rtc.h"

RTC_DS3231 rtc;
float rtc_temp;

int i = 0;

//std::string daysOfTheWeek[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String daysOfTheWeek[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

String timeString, dateString, dateAndTime;

bool timeset = true;
void set_time()
{
    if (! rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        
    
    //    //Serial.flush();
    //    //abort();
    }
    if (rtc.lostPower())
    {
        Serial.println("RTC lost power, setting the time!");
        rtc.adjust(DateTime(2024, 12, 10, 13, 54, 00));
    }
    else
    {  
        if(timeset == false)
        {
            Serial.println("Set time ...");
            rtc.adjust(DateTime(2024, 12, 11, 8, 56, 00));
            timeset = true;
        }
        else
        {
            DateTime now = rtc.now();
            dateString = String(now.year(), DEC) + '/' + String(now.month(), DEC) + '/' + String(now.day(), DEC);
      
            timeString = String(now.hour(), DEC) + ':' + String(now.minute(), DEC) +  ':' + String(now.second(), DEC);
            dateAndTime = dateString + " " + timeString;
            Serial.println(dateAndTime);

            rtc_temp = rtc.getTemperature(); //The Realtime Clock also has an internal temperature sensor which can be used
            Serial.println(rtc_temp);
        
        
        }               
    }
}