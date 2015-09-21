#include "NTPtime.h"

static unsigned int localPort = 2390;      // local port to listen for UDP packets
static IPAddress timeServerIP; // time.nist.gov NTP server address
static const char* ntpServerName = "time.nist.gov";
static const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
static byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

static WiFiUDP udp;

static unsigned long lastNTPtime = 0;
static unsigned long millisAtLastNTPtime = 0;

unsigned long getTime(int timezone, bool daylightSavingTime)
{
  static unsigned long nextNTPupdate = 0;
  unsigned long currentMillis = millis();
  unsigned long epoch = lastNTPtime + (currentMillis - millisAtLastNTPtime)/1000;
  if (epoch > nextNTPupdate) {
    unsigned long NTPTime = getNTPTime();
    if (NTPTime > 0) {
      lastNTPtime=NTPTime;
      millisAtLastNTPtime = millis();
      epoch = lastNTPtime;
      nextNTPupdate = epoch + 86400L; //next update in 24 hours
    }
  }
  epoch = epoch + (3600 * timezone);
  if (daylightSavingTime){
  	epoch = epoch + 3600;
  }
  return epoch;
}

unsigned long getNTPTime()
{
  udp.begin(localPort);
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    return 0;
  }
  else {
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
  
    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
  
/*
    // print the hour, minute and second: 
    int hours = (epoch  % 86400L) / 3600;
    int minutes = (epoch % 3600) / 60;
    int seconds = (epoch % 60);
    char timeString[8];
    sprintf(timeString,"%02d:%02d:%02d",hours, minutes, seconds);
    BLYNK_LOG("The UTC time is %s", timeString);       // UTC is the time at Greenwich Meridian (GMT)
*/
	udp.stop();
    return epoch;
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}