#include<Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include<SoftwareSerial.h>
#include <Digital_Light_TSL2561.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"
#define sample_num_mdate  5000
File myFile;
SoftwareSerial gps(2, 3);
SoftwareSerial lora(7, 8);
const int chipSelect = 4;
MPU9250 accelgyro;
I2Cdev   I2C_M;
const int distance = A6;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;
String pass;
BMP280 bmp280;

uint8_t buffer_m[6];
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void setup() {
  esc1.attach(5, 1000, 2000);
  esc1.attach(6, 1000, 2000);
  esc1.attach(9, 1000, 2000);
  esc1.attach(10, 1000, 2000);
  Wire.begin();
  gps.begin(9600);
  lora.begin(9600);
  Serial.begin(9600);
  Wire.setClock(400000);
  Serial.write("Welcome to RobotKPAOS Cansat please send me a password");
  while (pass != "hnoonnahoh") {
    pass = Serial.readString();
    if (pass != "hnoonnahoh" && pass != "") {
      Serial.println("");
      Serial.println("Go away fucking pirate!!!");
      Serial.println("Go away fucking pirate!!!");
      Serial.println("Go away fucking pirate!!!");
    }
  }
  Serial.println("");
  Serial.println("Hello my Team Prepare for log");
  Serial.println("Hello my Team Prepare for log");
  Serial.println("Hello my Team Prepare for log");
  // lora.write("Welcome to RobotKPAOS Cansat please send me a password");
  // while (!lora.find("hnoonnahoh"));

  /////////////////////////////////////////////////////////////////////////// SD SETUP /////////////////////////////////////////////////////////
  Serial.print("Initializing SD card...");
  lora.print("Initializing SD card...");
  pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    lora.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  lora.println("initialization done.");
  /////////////////////////////////////////////////////////////////////////// SD SETUP /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////// CALIBRATE ////////////////////////////////////////////////////////
  myFile = SD.open("log.txt", FILE_WRITE);
  Serial.println("Initializing I2C devices...");
  lora.println("Initializing I2C devices...");
  myFile.println("Initializing I2C devices...");
  accelgyro.initialize();
  bmp280.init();
  TSL2561.init();

  Serial.println("Testing device connections...");
  lora.println("Testing device connections...");
  myFile.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  lora.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  myFile.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  delay(1000);
  Serial.println("     ");
  lora.println("     ");
  myFile.println("     ");
  ///////////////////////////////////////////////////////////////////////// CALIBRATE////////////////////////////////////////////////
}

void loop() {

}
void getHeading(void) {
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void) {
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated () {
  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  while (!Serial.find("ready"));
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  get_calibration_Data ();

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}

void get_calibration_Data () {
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();
    /*
      Serial.print(mx_sample[2]);
      Serial.print(" ");
      Serial.print(my_sample[2]);                            //you can see the sample data here .// Must see it
      Serial.print(" ");
      Serial.println(mz_sample[2]);
    */

    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];

  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;

}

void get_one_sample_date_mxyz() {
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void) {
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated () {
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}

void getgpsdata () {
  while (gps.available() > 0) {
    byte gpsData = gps.read();
    Serial.write(gpsData);
    lora.write(gpsData);
    myFile.write(gpsData);
  }
}

void gpsprep () {
  Serial.write("wait gps a second...");
  lora.write("wait gps a second...");
  while (gps.available() == 0);
  Serial.write("Gps signal is coming...");
  lora.write("Gps signal is coming...");
}
