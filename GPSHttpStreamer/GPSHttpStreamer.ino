// A simple GPS data streamer over GPRS and HTTP
//
// This code is written to work with the Linkit One board and 
// the Adafruit Ultimate GPS v3 unit.
// 
// This code is released under the MIT License.
//
// (c) 2015 -- Jim Spring (jmspring@gmail.com)

// To use this sketch, you must have:
//  - A Linkit One development board
//  - The Adafruit Ultimate GPS v3 connected into Serial1 (D0 & D1)
//      - Specific pin mapping: GPS Tx -> D0, GPS Rx -> D1
//  - A SIM card installed into the Linkit One
//
// If playing with a GPS indoors, it is helpful to have an antenna 
// attached to the GPS unit.

#include "LTcpClient.h"
#include "LGPRSClient.h"
#include "LGPRS.h"

#include <Adafruit_GPS.h>

#include <time.h>

// buffer information.  
// setting the max data to buffer
#define BUFFER_SIZE     2048
uint8_t dataBuffer[BUFFER_SIZE];
uint32_t bufferPos;

// which pins are used
#define chipSelect 10

// gps information
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
// GPS_ECHO set to true to log raw GPS data to the Serial console
#define GPS_ECHO true
// GPS_LOG_FIX_ONLY set to true to log only when the GPS actually has a fix.
// It is handy for debugging purposes.
#define GPS_LOG_FIX_ONLY false
// GPS_UPDATE_FREQUENCY_HZ determines how often the GPS gathers more data.
// Allowed values are from 1 to 5x a second.  The more frequent data is
// gathered the more data will be logged.
#define GPS_UPDATE_FREQUENCY_HZ 1

// GPRS and Service information
uint32_t gprsConnectTimeout = 10000; // timeout in milliseconds.
char *apn = "fast.t-mobile.com";
char *server = "jims-carcloud.cloudapp.net";
uint16_t port = 8080;
char *path = "/checkin";
LGPRSClient connectionClient;
bool newConnectionEachSend = false;    // force a connect every time

// an error occurred, just stop processing and loop
void error(char *message)
{
  Serial.println(message);
  while(1) {
    delay(200);
  }
}

long date_to_timestamp(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
  struct tm t;
  t.tm_year = year + 100;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  long r = mktime(&t);
  return r;
}

bool send_data(void)
{
  bool r = true;
  
  // connect the socket
  Serial.println("connecting...");
  if(!connectionClient.connected()) {
    if(!connectionClient.connect(server, port)) {
      Serial.println("Unable to connect to server.");
      r = false;
    }
  }

  // manual http request, we won't care about the response
  if(r) {
    Serial.println("sending header...");
    char requestHeader[512];
    if(newConnectionEachSend) {
      sprintf(requestHeader, "POST %s HTTP/1.1\r\n"
                         "Host: %s\r\n"
                         "Connection: close\r\n"
                         "Content-Type: text/plain\r\n"
                         "Content-Length: %ld\r\n"
                         "\r\n", 
                         path, server, bufferPos);
    } else {
      sprintf(requestHeader, "POST %s HTTP/1.1\r\n"
                         "Host: %s\r\n"
                         "Connection: Keep-Alive\r\n"
                         "Content-Type: text/plain\r\n"
                         "Content-Length: %ld\r\n"
                         "\r\n", 
                         path, server, bufferPos);      
    }
    int t;
    t = connectionClient.write((uint8_t *)requestHeader, strlen(requestHeader));
    if(t != strlen(requestHeader)) {
        Serial.println("Unable to send HTTP header.");
        r = false;
    } else {
      Serial.println("sending body...");
      int idx = 0;
      while(idx != bufferPos) {
        t = connectionClient.write((uint8_t *)(dataBuffer + idx), bufferPos - idx);
        if(t <= 0) {
          Serial.println("Sending body failed.");
          r = false;
          break;
        } else {
          idx += t;
        }
      }

      // if we still haven't errored, read the result
      if(r) {
        Serial.println("reading response...");
        // give server a bit of time to respond
        long start = millis();
        while((millis() - start < 5000) && !connectionClient.available()) {
          delay(100);
        }
        
        while(connectionClient.available()) {
          char c = connectionClient.read();
          Serial.print(c);
        }
      }
    }
  }

  if(!r || newConnectionEachSend) {
    connectionClient.stop();
  }

  return r;
}

void setup() {
  // setup serial console
  Serial.begin(115200);
  Serial.println("GPS Data Logger");
  
  // setup GPS
  Serial.println("Setting up GPS.");
  GPS.begin(9600);
  GPSSerial.begin(9600);
  // we want RMC and GGA stanzas
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  if(GPS_UPDATE_FREQUENCY_HZ == 1) {
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  } else if(GPS_UPDATE_FREQUENCY_HZ == 5) {
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  } else {
    error("Invalid GPS config.");
  }

  // setup GPRS connection
  Serial.println("Connecting to GPRS network.");
  uint32_t start = millis();
  if(!LGPRS.attachGPRS(apn, NULL, NULL)) {
    delay(500);
    if(millis() - start > 10000) {
      error("Unable to connect to GPRS network.");
    }
  }

  // setup data buffer
  memset(dataBuffer, 0, BUFFER_SIZE);
  bufferPos = 0;
}

struct gps_info_t {
  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds;
  float latitude, longitude;
  float altitude;
  float speed, angle;
};
struct gps_info_t gpsInfo = { 0 };

bool gps_differs(void)
{
  return ((GPS.hour != gpsInfo.hour) ||
           (GPS.minute != gpsInfo.minute) ||
           (GPS.seconds != gpsInfo.seconds) ||
           (GPS.year != gpsInfo.year) ||
           (GPS.month != gpsInfo.month) ||
           (GPS.day != gpsInfo.day) ||
           (GPS.milliseconds != gpsInfo.milliseconds) ||
           (GPS.latitude != gpsInfo.latitude) ||
           (GPS.longitude != gpsInfo.longitude) ||
           (GPS.speed != gpsInfo.speed) ||
           (GPS.angle != gpsInfo.angle));
}

void gps_info_update(void) {
  gpsInfo.hour = GPS.hour;
  gpsInfo.minute = GPS.minute;
  gpsInfo.seconds = GPS.seconds;
  gpsInfo.year = GPS.year;
  gpsInfo.month = GPS.month;
  gpsInfo.day = GPS.day;
  gpsInfo.milliseconds = GPS.milliseconds;
  gpsInfo.latitude = GPS.latitude;
  gpsInfo.longitude = GPS.longitude;
  gpsInfo.speed = GPS.speed;
  gpsInfo.angle = GPS.angle;
}

void loop() {
  // arduino runs the loop continuously.  given the speed of the
  // processor, the GPS will not be providing data faster than the
  // loop will run.
  char c = GPS.read();
  if(GPS_ECHO) {
    if(c) {
      Serial.print(c);
    }
  }

  // the GPS itself is working internally based off an interupt and
  // can be queried through the newNMEAreceived method to see if data
  // needs to be processed.  unlike other GPS libraries, the parsing
  // and what not are handled internally.
  if(GPS.newNMEAreceived()) {
    // NMEA statements have a checksum associated with them, it is best
    // to make sure the statement parses.  Otherwise, we should not log it.
    if(GPS.parse(GPS.lastNMEA())) {
      // do we require a fix?  and do we have one?
      if((GPS_LOG_FIX_ONLY && GPS.fix) || (!GPS_LOG_FIX_ONLY)) {
        // while we may be reading multiple NMEA sentences, we don't want to
        // send duplicate information.
        //
        // TODO -- need a better definition of "differs".  Or, we need to decouple
        // the "recording rate" from the updating of NMEA statements.
        if(gps_differs()) {
          gps_info_update();          

          char statement[256];
          long timestamp = date_to_timestamp(gpsInfo.year, gpsInfo.month, gpsInfo.day,
                                             gpsInfo.hour, gpsInfo.minute, gpsInfo.seconds);
          sprintf(statement, "type,location;timestamp,%ld%03u;latitude,%f;longitude,%f;speed,%f;heading,%f;altitude,%f",
                  timestamp, gpsInfo.milliseconds, gpsInfo.latitude, gpsInfo.longitude, gpsInfo.speed, gpsInfo.angle, gpsInfo.altitude);
          uint32_t length = (uint32_t)strlen(statement);
    
          if(bufferPos + length + 1 > BUFFER_SIZE) {
            if(!send_data()) {
              char s[60];
              sprintf(s, "Error sending data to server.  Length: %u");
              Serial.println(s);
            }
            bufferPos = 0;
          }
          memcpy(dataBuffer + bufferPos, statement, length);
          dataBuffer[bufferPos + length] = '\n';
          bufferPos = bufferPos + length + 1;
        }
      }
    }
  }
}
