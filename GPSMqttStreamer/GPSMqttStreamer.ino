// A simple GPS data streamer over GPRS and MQTT
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

#include <PubSubClient.h>

#include <time.h>

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
char *apn = "your apn";
char *server = "your server";
uint16_t port = 1883;
char *mqtt_id = "gpsmqttstreamer";
char *mqtt_publish_topic = "location";
LGPRSClient radioClient;
PubSubClient mqttClient(radioClient);

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

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // print the topic
  Serial.print("Message arrived.");
  Serial.print("Topic: ");
  Serial.println(topic);

  // print the payload
  Serial.println("Payload:");
  Serial.println("  ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void mqtt_connect(void) {
  while(!mqttClient.connected()) {
    Serial.println("Attempting to set up MQTT connection.");

    // attempt to connect
    if(mqttClient.connect(mqtt_id)) {
      Serial.println("Connected...");

      // we aren't subscribing to anything, otherwise
      // you would do so here
    } else {
      Serial.print("Connection failed.  Error: ");
      Serial.println(mqttClient.state());
      Serial.println("Will try again in 5 seconds...");

      delay(5000);
    }
  }
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

  // setup MQTT client
  mqttClient.setServer(server, port);
  mqttClient.setCallback(mqtt_callback);
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
  // enable mqtt connect and some processing
  if(!mqttClient.connected()) {
    mqtt_connect();
  }
  mqttClient.loop();
  
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
          sprintf(statement, "ts:%ld%03u;lat:%f;lon,%f;spd,%f;head,%f;alt,%f",
                  timestamp, gpsInfo.milliseconds, gpsInfo.latitude, gpsInfo.longitude, gpsInfo.speed, gpsInfo.angle, gpsInfo.altitude);
          uint32_t length = (uint32_t)strlen(statement);

          Serial.print("Publishing: ");
          Serial.println(statement);
          if(!mqttClient.publish(mqtt_publish_topic, statement)) {
            Serial.println("Unable to publish statement.");
          }
        }
      }
    }
  }
}
