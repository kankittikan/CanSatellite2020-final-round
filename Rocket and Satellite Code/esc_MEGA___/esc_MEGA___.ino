#include<Servo.h>
Servo esc;
void setup() {
  esc.attach(7,1000,2000);
  esc.write(0);
}

void loop() {
  esc.write(0);
  delay(1000);
  esc.write(180);
  delay(1000);
  esc.write(0);

}
