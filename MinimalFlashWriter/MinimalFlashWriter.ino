// A simple example of writing a character to flash every second or so.
//
// This code is written to work with the Linkit One board.
// 
// This code is released under the MIT License.
//
// (c) 2015 -- Jim Spring (jmspring@gmail.com)

#include <LFlash.h>
#include <LSD.h>
#include <LStorage.h>

#define DISK    LFlash
LFile logFile;
#define chipSelect 10
int iterations;

void setup() {
  // put your setup code here, to run once:
  // setup serial console
  Serial.begin(115200);

  // setup storage object and open file
  iterations = 0;
  pinMode(chipSelect, OUTPUT);
  if(!DISK.begin()) {
    Serial.println("Unable to initialize storage.");
    while(1) {
      delay(1000);
    }
  }
  uint16_t i;
  char filename[15];
  for(i = 0; i < 1000; i++) {
    sprintf(filename, "dummy%03u.txt", i);
    if(!DISK.exists(filename)) {
      break;
    }
  }
  Serial.print("Opening ");
  Serial.println(filename);
  logFile = DISK.open(filename, FILE_WRITE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(iterations < 120) {
    Serial.println("Writing a character...");
    logFile.write("A");
  } else if(iterations == 120) {
    Serial.println("Closing file...");
    logFile.flush();
    logFile.close();
  }
  iterations++;
  delay(1000); // pause for a second
}
