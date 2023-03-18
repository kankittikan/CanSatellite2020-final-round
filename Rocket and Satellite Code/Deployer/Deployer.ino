#include<Servo.h>

Servo deploy;
int apogee;
int num;
int numloop;
void setup() {
  pinMode(7,OUTPUT);
  Serial.begin(9600);
  deploy.attach(6);
  deploy.write(0);
}

void loop() {
  Serial.println(apogee = digitalRead(3));
 if(apogee == 0){
  numloop += 1;
  if(numloop == 1){
    deploy.write(180);
    do{
      digitalWrite(7,HIGH);
      delay(50);
      digitalWrite(7,LOW);
      delay(50);
      num +=1;
    }while(num < 50);
    deploy.write(0);
  }
  }
}
