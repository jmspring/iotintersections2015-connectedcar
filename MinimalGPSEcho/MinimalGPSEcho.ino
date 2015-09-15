// A simple GPS stanza echo
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

#include <Adafruit_GPS.h>

// Serial1 = Pins D0/D1 on Linkit One
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

void setup() {
  // put your setup code here, to run once:
  // setup serial output
  Serial.begin(115200);

  // setup gps
  GPS.begin(9600);
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
}

void loop() {
  // put your main code here, to run repeatedly:
  char c = GPS.read();
  if(c) {
    Serial.print(c);
  }
}
