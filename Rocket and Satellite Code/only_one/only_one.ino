#include<Servo.h>
#include<SoftwareSerial.h>

SoftwareSerial lora(10, 11);
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
int power;
void setup() {
  esc1.attach(5, 1000, 2000);
  esc1.write(0);
  Serial.begin(9600);


}

void loop() {
  // lora.println("Cansat online");

  if (Serial.find("go")) {
    //esc1.write(30);
    /*esc1.write(50);
      delay(5000);*/
    //esc4.write(30);
    esc1.write(30);
    delay(2000);
    /*esc3.write(10);
      esc1.write(10);
      delay(500);*/
    //esc4.write(0);
    esc1.write(0);
  }
}
