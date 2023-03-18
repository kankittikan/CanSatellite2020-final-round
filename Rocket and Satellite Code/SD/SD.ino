#include <SPI.h>
#include <SD.h>

File myFile;
const int chipSelect = 4; //depend on CS pin

void setup() {
  /////////////////////////////////////////////////////////////////////////// SD CARD TEST ////////////////////////////////////
  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("log.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("log.txt");
  if (myFile) {
    Serial.println("log.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  ////////////////////////////////////////////////////////////////////////// SD CARD TEST////////////////////////////////////

}

void loop() {
  // put your main code here, to run repeatedly:

}
