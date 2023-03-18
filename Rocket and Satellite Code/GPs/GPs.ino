#include <TinyGPS++.h>
#include <SoftwareSerial.h>

SoftwareSerial lora (8,7);

// กำหนดขา 8 เป็น RX และขา 9 เป็น TX
static const int RXPin = 3, TXPin = 2;
// กำหนดค่า Baud Rate ของโมดูล GPS = 9600 (ค่า Default)
static const uint32_t GPSBaud = 9600;
 
// สร้าง TinyGPS++ object ชื่อ myGPS
TinyGPSPlus myGPS;
 
// สร้าง Software Serial object ชื่อ mySerial
SoftwareSerial mySerial(RXPin, TXPin);

 
void setup(){
// เริ่ม Serial สำหรับใช้งาน Serial Monitor
  Serial.begin(9600);
  lora.begin(9600);
// เริ่มใช้งาน Software Serial
  mySerial.begin(GPSBaud);

  lora.println("GPS Module Tutorial");
  lora.println("This tutorial base on NEO-6M device");
  lora.println();
}
 
 
void loop()
{
// ถ้า mySerial มีการสื่อสารข้อมูล ให้ library ถอดรหัสข้อมูลแล้วเรียกใช้ฟังก์ชั่น GPSinfo
  while (mySerial.available() > 0)
    if (myGPS.encode(mySerial.read()))
      GPSinfo();

// ถ้ารอ 5 วินาทีแล้วยังไม่มีข้อมูล ให้แสดงข้อความผิดพลาด
  if (millis() > 5000 && myGPS.charsProcessed() < 10) {
    lora.println("No GPS detected: check wiring.");
    while(true);
  }
}

/*
 * ฟังก์ชั่น GPSinfo
 */
void GPSinfo(){
  lora.print("Location: "); 
  // ถ้ามีข้อมูลตำแหน่ง
  if (myGPS.location.isValid()) {
    lora.print(myGPS.location.lat(), 6);      // lattitude เป็นองศา ทศนิยม 6 ตำแหน่ง
    lora.print(", ");
    lora.print(myGPS.location.lng(), 6);      // longitude เป็นองศา ทศนิยม 6 ตำแหน่ง
    lora.print("\t");                         // เคาะวรรค 1 tab
  } else {                                      // กรณีผิดพลาดแสดงข้อความผิดพลาด
    lora.print("INVALID");
  }

  // ถ้ามีข้อมูลวันที่
  lora.print("  Date/Time: ");
  if (myGPS.date.isValid()) {
    lora.print(myGPS.date.day());             // แสดงวันที่
    lora.print("/");
    lora.print(myGPS.date.month());           // แสดงเดือน
    lora.print("/");
    lora.print(myGPS.date.year());            // แสดงปี
  } else {                                      // กรณีผิดพลาดแสดงข้อความผิดพลาด
    lora.print("INVALID");
  }

  // ถ้ามีข้อมูลเวลา (แสดงเป็นเวลา UTC)
  lora.print("\t");                                 // เคาะวรรค 1 tab
  if (myGPS.time.isValid()) {
    if (myGPS.time.hour() < 10) lora.print("0");    // แสดงค่าชั่วโมง ถ้ามีหลักเดียวเติม 0 ด้านหน้า
    lora.print(myGPS.time.hour());
    lora.print(":");
    if (myGPS.time.minute() < 10) lora.print("0");  // แสดงค่านาที ถ้ามีหลักเดียวเติม 0 ด้านหน้า
    lora.print(myGPS.time.minute());
    lora.print(":");
    if (myGPS.time.second() < 10) lora.print("0");  // แสดงค่าวินาที ถ้ามีหลักเดียวเติม 0 ด้านหน้า
    lora.print(myGPS.time.second());
  } else {                                      // กรณีผิดพลาดแสดงข้อความผิดพลาด
    lora.print("INVALID");
  }

  // ขึ้นบรรทัดใหม่รอไว้
  lora.println();

}
