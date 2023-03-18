#include<SoftwareSerial.h>

SoftwareSerial lora(8,7);
void setup() {
  lora.begin(9600);
  Serial.begin(9600);

}

void loop() {
  lora.println("go");
  delay(100);

}
