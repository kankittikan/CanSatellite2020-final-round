#include <SoftwareSerial.h>

#include <TinyGPS.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;
SoftwareSerial ss(3, 2);
SoftwareSerial lora(8,7);

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  lora.begin(9600);
  
  lora.print("Simple TinyGPS library v. "); lora.println(TinyGPS::library_version());
  lora.println("by Mikal Hart");
  lora.println();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      lora.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    lora.print("LAT=");
    lora.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    lora.print(" LON=");
    lora.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    lora.print(" SAT=");
    lora.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    lora.print(" PREC=");
    lora.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  lora.print(" CHARS=");
  lora.print(chars);
  lora.print(" SENTENCES=");
  lora.print(sentences);
  lora.print(" CSUM ERR=");
  lora.println(failed);
  if (chars == 0)
    lora.println("** No characters received from GPS: check wiring **");
}
