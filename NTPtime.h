#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

unsigned long getTime(int timezone, bool daylightSavingTime);
unsigned long getNTPTime();
unsigned long sendNTPpacket(IPAddress& address);

