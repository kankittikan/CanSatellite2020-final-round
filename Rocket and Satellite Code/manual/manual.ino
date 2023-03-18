#include<Servo.h>

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
int i, j, k, l;
const int n1 = 90;
const int n2 = 80;
const int n3 = 85;
const int n4 = 55;
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
  if (inbyte == 'w') {
    //speedX = 30;
    esc1.write(n1 - 10);
    esc2.write(n2 - 10);
    esc3.write(n3 + 10);
    esc4.write(n4 + 10);
  }
  else if (inbyte == 'a') {
    //speedX = 30;
    esc1.write(n1 - 10);
    esc2.write(n2 + 10);
    esc3.write(n3 - 10);
    esc4.write(n4 + 10);
  }
  else if (inbyte == 's') {
    //speedX = 30;
    esc1.write(n1 + 10);
    esc2.write(n2 + 10);
    esc3.write(n3 - 10);
    esc4.write(n4 - 10);
  }
  else if (inbyte == 'd') {
    //speedX = 30;
    esc1.write(n1 + 10);
    esc2.write(n2 - 10);
    esc3.write(n3 + 10);
    esc4.write(n4 - 10);
  }
  else if (inbyte == 'q') {
    //speedX = 30;
    esc1.write(n1 + 5);
    esc2.write(n2);
    esc3.write(n3);
    esc4.write(n4 + 5);
  }
  else if (inbyte == 'e') {
    //speedX = 30;
    esc1.write(n1);
    esc2.write(n2 + 5);
    esc3.write(n3 + 5);
    esc4.write(n4);
  }
  else if (inbyte == 'z') {
    //speedX = 30;
    esc1.write(n1 + 10);
    esc2.write(n2 + 10);
    esc3.write(n3 + 10);
    esc4.write(n4 + 10);
  }
  else if (inbyte == 'x') {
    //speedX = 30;
    esc1.write(n1 - 10);
    esc2.write(n2 - 10);
    esc3.write(n3 - 10);
    esc4.write(n4 - 10);
  }
  else if (inbyte == 'm') {
    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);
    while (1);
  }
  else {
    esc1.write(n1);
    esc2.write(n2);
    esc3.write(n3);
    esc4.write(n4);
  }

}
