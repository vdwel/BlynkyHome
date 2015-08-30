/**************************************************************
 * Blynk is a platform with iOS and Android apps to control
 * Arduino, Raspberry Pi and the likes over the Internet.
 * You can easily build graphic interfaces for all your
 * projects by simply dragging and dropping widgets.
 *
 *   Downloads, docs, tutorials: http://www.blynk.cc
 *   Blynk community:            http://community.blynk.cc
 *   Social networks:            http://www.fb.com/blynkapp
 *                               http://twitter.com/blynk_app
 *
 * Blynk library is licensed under MIT license
 * This example code is in public domain.
 *
 **************************************************************
 * This example runs directly on ESP8266 chip.
 *
 * You need to install this for ESP8266 development:
 *   https://github.com/esp8266/Arduino
 *
 * Change WiFi ssid, pass, and Blynk auth token to run :)
 *
 **************************************************************/

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
//#define BLYNK_RETRY_SEND
//#define BLYNK_SEND_CHUNK 64 // Split whole command into chunks (in bytes)
//#define BLYNK_SEND_THROTTLE 10 // Wait after sending each chunk (in milliseconds)

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <OneWire.h>
#include <RemoteSwitch.h>

//#define HOME //uncomment for home server
#define LED 5
#define transmitterPIN 2
bool LEDstatus = HIGH;
const char* SID = "Avalon Network";
const char* PAS = "1Lwe1vd1";
const char* HOMESERVER = "192.168.10.95";
char globalauth[] = "b7ba8b6b4b9341f180fef5f58621ad79";
char homeauth[] = "beb48d48eb734b31938756266ebed6e5";
String thingspeakAPIkey = "M2X7EQ8HJKA062WH";


#ifndef HOME
char* auth = globalauth;
#else
char* auth = homeauth;
#endif

long counter = 0;
SimpleTimer timer;

float temperature = 0;

OneWire  ds(4);

void connectWiFi(const char* ssid = SID, const char* pass = PAS, int timeout = 10);

void setup()
{
  Serial.begin(9600);
  connectWiFi();
  #ifdef HOME
  Blynk.config(auth, HOMESERVER);
  #else
  Blynk.config(auth);
  #endif
  timer.setInterval(1000L, callBack);
  timer.setInterval(300000L, notifyUptime);
  timer.setInterval(15000L, readTemp);
  timer.setInterval(60000, periodicUpdateThingspeak);
  pinMode(LED, OUTPUT);
  pinMode(transmitterPIN, OUTPUT);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  Blynk.run();
  timer.run();
}

void callBack(){
  digitalWrite(LED, LEDstatus);
  LEDstatus = !LEDstatus;
  Blynk.virtualWrite(V8, LEDstatus);
//  Serial.print(".");
  Blynk.virtualWrite(V7, counter);
  counter += 1;
  if (counter > 1000) {
    counter = 0;
  }
  long uptime = millis() / 60000L;
  Blynk.virtualWrite(V2, uptime);
}

void notifyUptime()
{
  long uptime = millis() / 60000L;

  // Actually send the message.
  // Note:
  //   We allow 1 notification per minute for now.
  Blynk.notify(String("Running for ") + uptime + " minutes.");
}


BLYNK_WRITE(31)
{
  Blynk.virtualWrite(V30, HIGH);
  if(param[0].asInt()){
    licht_aan();
  } else {
    licht_uit();
  }
  Blynk.virtualWrite(V30, LOW);
}


void readTemp()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
//      Serial.println("CRC is not valid!");
      return;
  }
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  //delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.println(celsius);
  Blynk.virtualWrite(V1, celsius);
  temperature = celsius;
  ds.reset_search();
}

void periodicUpdateThingspeak()
{
  char t_buffer[10];
  String temp = dtostrf(temperature, 3, 2, t_buffer);
  updateThingspeak(thingspeakAPIkey, "field1="+temp);

}

void connectWiFi(const char* ssid, const char* pass, int timeout)
{
    int timeoutCounter = 0;

    while (WiFi.status() != WL_CONNECTED) {
      timeoutCounter = 0;
      BLYNK_LOG("Connecting to %s", ssid);
      if (pass && strlen(pass)) {
        WiFi.begin(ssid, pass);
      } else {
        WiFi.begin(ssid);
      }
      
      while ((WiFi.status() != WL_CONNECTED) & (timeoutCounter<timeout*2)) {
          timeoutCounter += 1;
          ::delay(500);
      }
    }
    BLYNK_LOG("Connected to WiFi");

    IPAddress myip = WiFi.localIP();
    BLYNK_LOG("My IP: %d.%d.%d.%d", myip[0], myip[1], myip[2], myip[3]);

}

void licht_aan() {
  
  //digitalWrite(transmitterPIN, HIGH);
  Serial.println("Lights on");
  schakel_lamp(4,1,1);
  schakel_lamp(4,2,1);
  schakel_lamp(4,3,1);
}

void licht_uit() {
  //digitalWrite(transmitterPIN, LOW);
  Serial.println("Lights off");
  schakel_lamp(4,1,0);
  schakel_lamp(4,2,0);
  schakel_lamp(4,3,0);
}

long int calc_code(int chan, int lamp, int state)
{
  long int code = 522690;
  unsigned int period = 424;
  chan = 3 - chan + 1;
  lamp = lamp - 1;
  for (int i=0; i <= chan; i++){
    code = code - (pow(3,i) * 8748);
  }
  code = code - 8748;
  for (int i=0; i <= lamp; i++){
    code = code + (pow(3,(4-i)) * 108);  
  }
  code=code + (state * 2);
  
  code |= (unsigned long)period << 23;
  code |= 3L << 20;
  return code;
}

void schakel_lamp(int chan, int lamp, int state)
{
//  int maxt = 10;
  long int code = calc_code(chan, lamp, state);
//  for (int i = 0; i < maxt; i++)
//  {
    Blynk.run();
    timer.run();
    RemoteSwitch::sendTelegram(code,transmitterPIN);
//  }
}

void updateThingspeak(String APIkey, String tsData)
{
    WiFiClient client;
    const int httpPort = 80;
//if (!client.connect("api.thingspeak.com", httpPort)) {
  if (!client.connect("184.106.153.149", httpPort)) {
      Serial.println("connection to thingspeak failed");
      return;
    }
    
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+APIkey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(tsData.length());
    client.print("\n\n");
    client.print(tsData);
    
    client.stop();
}

