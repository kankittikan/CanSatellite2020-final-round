#include<Servo.h>

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
int i, j, k, l;
char inbyte;
void setup() {
  Serial.begin(9600);
  esc1.attach(5, 1000, 2000);
  esc2.attach(6, 1000, 2000);
  esc3.attach(9, 1000, 2000);
  esc4.attach(10, 1000, 2000);
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);
  while (!Serial.find("g"));
  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  esc4.write(30);
}

void loop() {
  inbyte = Serial.read();
  if (inbyte == 'a') {
    //speedX = 30;
    esc1.write(90);
    esc2.write(80);
    esc3.write(85);
    esc4.write(55);
  }
  else if (inbyte == ' ') {
    //speedX = 0;
    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);
  }

}




void motor(int a, int b, int c, int d) {
  esc1.write(a);
  esc2.write(b);
  esc3.write(c);
  esc4.write(d);
}
