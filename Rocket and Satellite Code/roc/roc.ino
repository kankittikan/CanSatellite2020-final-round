#include<Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "BMP280.h"
I2Cdev   I2C_M;
Servo gotnahoh;
float temperature;
float pressure;
float atm;
float altitude;
float a, b;
BMP280 bmp280;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  bmp280.init();
  gotnahoh.attach(5);
  gotnahoh.write(0);
}

void loop() {
  temperature = bmp280.getTemperature(); //Get the temperature, bmp180ReadUT MUST be called first
  pressure = bmp280.getPressure();//Get the temperature
  altitude = bmp280.calcAltitude(pressure);
  a = trunc(altitude);
  Serial.println(a);
  
  delay(2000);
  temperature = bmp280.getTemperature(); //Get the temperature, bmp180ReadUT MUST be called first
  pressure = bmp280.getPressure();//Get the temperature
  altitude = bmp280.calcAltitude(pressure);
  b = trunc(altitude);
  
  b = b+1;
  Serial.println(b);
  if(a>b){
    gotnahoh.write(90);
    Serial.println("Servo 90 !!!");
    delay(3000);
    gotnahoh.write(0);
  }
}
