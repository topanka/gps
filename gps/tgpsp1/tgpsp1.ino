#include <TinyGPSPlus.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GPSBaud = 57600;
#define Monitor Serial2

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(57600);     //GPS
  Serial2.begin(19200);     //radio

  Monitor.println(F("DeviceExample.ino"));
  Monitor.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Monitor.print(F("Testing TinyGPSPlus library v. ")); 
  Monitor.println(TinyGPSPlus::libraryVersion());
  Monitor.println(F("by Mikal Hart"));
  Monitor.println();
}

void loop()
{
  char c;
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0) {
    c=Serial1.read();
//    Serial.print(c);
    if (gps.encode(c)) {
      displayInfo();
    }
  }
/*
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
*/  
}

void displayInfo()
{
  static int xxx=0;

  xxx=(xxx+1)%20;
  if(xxx != 0) return;

  Monitor.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Monitor.print(gps.location.lat(), 6);
    Monitor.print(F(","));
    Monitor.print(gps.location.lng(), 6);
  }
  else
  {
    Monitor.print(F("INVALID"));
  }

  Monitor.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Monitor.print(gps.date.month());
    Monitor.print(F("/"));
    Monitor.print(gps.date.day());
    Monitor.print(F("/"));
    Monitor.print(gps.date.year());
  }
  else
  {
    Monitor.print(F("INVALID"));
  }

  Monitor.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Monitor.print(F("0"));
    Monitor.print(gps.time.hour());
    Monitor.print(F(":"));
    if (gps.time.minute() < 10) Monitor.print(F("0"));
    Monitor.print(gps.time.minute());
    Monitor.print(F(":"));
    if (gps.time.second() < 10) Monitor.print(F("0"));
    Monitor.print(gps.time.second());
    Monitor.print(F("."));
    if (gps.time.centisecond() < 10) Monitor.print(F("0"));
    Monitor.print(gps.time.centisecond());
  }
  else
  {
    Monitor.print(F("INVALID"));
  }

  Monitor.println();
}

