#include<Servo.h>
#include<SoftwareSerial.h>

SoftwareSerial lora(10, 11 );
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
int power;
int i, j, k, l;
void setup() {
  esc1.attach(5, 1000, 2000);
  esc2.attach(6, 1000, 2000);
  esc3.attach(9, 1000, 2000);
  esc4.attach(10, 1000, 2000);
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);
  Serial.begin(115200);
  lora.begin(9600);
  lora.println("Cansat online");
  lora.println("Please fill out password");
}

void loop() {
  if (lora.find("hnoonnahoh")) {
    lora.println("Fill out motor thrust(0-160) - A");
    while (lora.available());
    i = lora.read();
    lora.println("Fill out motor thrust(0-160) - B");
    while (!lora);
    j = lora.read();
    lora.println("Fill out motor thrust(0-160) - C");
    while (!lora);
    k = lora.read();
    lora.println("Fill out motor thrust(0-160) - D");
    while (!lora);
    l = lora.read();
    delay(1000);
    motor(i, j, k, l);
    delay(2000);
    motorstop();
  } else if (lora.available()) {
    lora.println("Password Wrong...");
  }
}

void motor(int a, int b, int c, int d) {
  esc1.write(a);
  esc2.write(b);
  esc3.write(c);
  esc4.write(d);
  //delay(2000);
}

void motorstop() {
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);
}
