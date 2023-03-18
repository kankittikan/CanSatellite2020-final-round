#include<SoftwareSerial.h>

SoftwareSerial lora(5,4);
void setup() {
  lora.begin(9600);
  Serial.begin(9600);

}

void loop() {
 if(lora.available()){
  Serial.write(lora.read());
  tone(8,2000);
 }
 else{
  noTone(8);
 }
 

}
