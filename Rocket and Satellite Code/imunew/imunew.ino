#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>
File myFile;
const int chipSelect = 4;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
TinyGPS gps;
SoftwareSerial ss(3, 2);
SoftwareSerial lora(8, 7);
int t, x;
int a, b, c, d, e, f, g, h, i, j;
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;



float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];


#define sample_num_mdate  5000

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
BMP280 bmp280;

void setup() {
  esc1.attach(5, 1000, 2000);
  esc2.attach(6, 1000, 2000);
  esc3.attach(9, 1000, 2000);
  esc4.attach(10, 1000, 2000);
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(9600);
  ss.begin(9600);
  lora.begin(9600);
  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("log.txt", FILE_WRITE);

  lora.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  bmp280.init();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  delay(1000);
  Serial.println("     ");
  //Mxyz_init_calibrated ();
  while (!Serial.find("g"));
  esc1.write(40);
  esc2.write(40);
  esc3.write(40);
  esc4.write(40);
  delay(500);
}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    lora.print("LAT=");
    myFile.print("LAT=");
    lora.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    lora.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    myFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    lora.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    myFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    lora.print(" LON=");
    myFile.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    lora.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    myFile.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    lora.print(" SAT=");
    myFile.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    lora.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    myFile.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    lora.print(" PREC=");
    myFile.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    lora.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    myFile.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  lora.print(" CHARS=");
  myFile.print(" CHARS=");
  Serial.print(chars);
  lora.print(chars);
  myFile.print(chars);
  Serial.print(" SENTENCES=");
  lora.print(" SENTENCES=");
  myFile.print(" SENTENCES=");
  Serial.print(sentences);
  lora.print(sentences);
  myFile.print(sentences);
  Serial.print(" CSUM ERR=");
  myFile.print(" CSUM ERR=");
  lora.print(" CSUM ERR=");
  Serial.println(failed);
  lora.println(failed);
  myFile.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  /*getCompassDate_calibrated();
    getHeading();
    getTiltHeading();

    Serial.println("Acceleration(g) of X,Y,Z:");
    Serial.print(Axyz[0]);
    Serial.print(",");
    Serial.print(Axyz[1]);
    Serial.print(",");
    Serial.println(Axyz[2]);
    Serial.println("heading :");
    Serial.print(heading);
    Serial.println(" ");
    Serial.println(x);



    temperature = bmp280.getTemperature();
    pressure = bmp280.getPressure();
    altitude = bmp280.calcAltitude(pressure);

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print("deg C |");

    Serial.print("Pressure: ");
    Serial.print(pressure, 0);
    Serial.print(" Pa |");

    Serial.print("Altitude: ");
    Serial.print(altitude, 2);
    Serial.println(" m");

    Serial.println();
  */
  if (lora.available()) {
    if (lora.find(" ")) {
      esc1.write(0);
      esc2.write(0);
      esc3.write(0);
      esc4.write(0);
      while (1);

    }
  }
  
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

    Serial.print(mx_sample[2]);
    Serial.print(" ");
    Serial.print(my_sample[2]);
    Serial.print(" ");
    Serial.println(mz_sample[2]);


    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2];
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2];
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
  Axyz[0] = ax / 100;
  Axyz[1] = ay / 100;
  Axyz[2] = az / 100;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Accelx_control(void) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = ax / 100;
  /*accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    b = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    c = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    d = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    e = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    f = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    g = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    h = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    i = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    j = ax / 100;
    Axyz[1] = (a + b + c + d + e + f + g + h + i + j) / 10;*/
  x = Axyz[0];
  if (x < 0) {
    x = x * -1;
    if (x > 40) {
      x = 40;
    }
    t = map(x, 0, 40, 80, 90);
    esc1.write(t);
    esc3.write(t);
    Serial.println(t);
  }
  if (x > 0) {
    if (x > 40) {
      x = 40;
    }
    t = map(x, 0, 40, 80, 90);
    esc2.write(t);
    esc4.write(t);
    Serial.println(t);
  }
}

void Accely_control(void) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[1] = ay / 100;
  /*accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    b = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    c = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    d = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    e = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    f = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    g = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    h = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    i = ax / 100;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    j = ax / 100;
    Axyz[1] = (a + b + c + d + e + f + g + h + i + j) / 10;*/
  x = Axyz[1];
  if (x < 0) {
    x = x * -1;
    if (x > 40) {
      x = 40;
    }
    t = map(x, 0, 40, 80, 90);
    esc1.write(t);
    esc2.write(t);
    Serial.println(t);
  }
  if (x > 0) {
    if (x > 40) {
      x = 40;
    }
    t = map(x, 0, 40, 80, 90);
    esc3.write(t);
    esc4.write(t);
    Serial.println(t);
  }
}

void Heading_control(void) {
  getCompassDate_calibrated();
  getHeading();
}
///////////////////////////////////////////////////////////////////////////////////////////////////
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
