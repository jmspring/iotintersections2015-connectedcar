// A simple GPS data logger
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
//
// If playing with a GPS indoors, it is helpful to have an antenna 
// attached to the GPS unit.

#include <LFlash.h>
#include <LSD.h>
#include <LStorage.h>

#include <Adafruit_GPS.h>

#include <time.h>

// which storage to use?
// Linkit One has 10Meg of internal flash as well as support for SD cards
#define DISK    LFlash
//#define DISK    LSD

// file information.  
// setting the max file size low to force multiple files
#define MAX_FILE_SIZE     65536
// how many milliseconds between each time we flush the file
#define FLUSH_FILE_FREQUENCY    5000
LFile logFile;
uint32_t fileSize;
uint32_t lastFlush;

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
#define GPS_UPDATE_FREQUENCY_HZ 5


// cycle through up to 1000 files.
// realistically, filenames based off of time stamp or another metric
// is often safer
bool open_file(void) 
{
  uint16_t i;
  char filename[15];
  for(i = 0; i < 1000; i++) {
    sprintf(filename, "log-%03u.txt", i);
    if(!DISK.exists(filename)) {
      break;
    }
  }
  if(i != 1000) {
    logFile = DISK.open(filename, FILE_WRITE);
    if(!logFile) {
      return false;
    }
    fileSize = 0;
  }
  return (i != 1000);
}

// flush the logfile contents and close it.
void close_file(void)
{
  logFile.flush();
  logFile.close();
}

// an error occurred, just stop processing and loop
void error(char *message)
{
  Serial.println(message);
  while(1) {
    delay(200);
  }
}

unsigned long date_to_timestamp(uint8_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second)
{
  struct tm t;
  t.tm_year = year + 100;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  unsigned long r = mktime(&t);
  return r;
}

void setup() {
  // setup serial console
  Serial.begin(115200);
  Serial.println("GPS Data Logger");

  // setup storage object and open file
  pinMode(chipSelect, OUTPUT);
  if(!DISK.begin()) {
    error("Unable to initialize storage.");
  }
  if(!open_file()) {
    error("Unable to open file.");
  }
  lastFlush = 0;
  
  // setup GPS
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
        char statement[256];
        unsigned long timestamp = date_to_timestamp(GPS.year, GPS.month, GPS.day,
                                                    GPS.hour, GPS.minute, GPS.seconds);
        sprintf(statement, "type,location;timestamp,%u%03u;latitude,%f;longitude,%f;speed,%f;heading,%f;altitude,%f",
                timestamp, GPS.latitude, GPS.longitude, GPS.speed, GPS.angle, GPS.altitude);
        uint32_t length = (uint32_t)strlen(statement);

        // write the statement to the log file
        if(logFile.write((uint8_t *)statement, length) != length) {
          error("Unable to write full statement.");
        }

        // update the amount of data written, if around MAX_FILE_SIZE,
        // then close the file and open a new one.
        fileSize = fileSize + length;
        if(fileSize > MAX_FILE_SIZE) {
          close_file();
          if(!open_file()) {
            error("Unable to open new file.");  
          }
        } else {
          if(millis() - lastFlush >= FLUSH_FILE_FREQUENCY) {
            logFile.flush();
            lastFlush = millis();
          }
        }
      }
    }
  }
}
