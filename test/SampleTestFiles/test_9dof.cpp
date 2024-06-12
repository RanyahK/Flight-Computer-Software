#include "Arduino_PortentaBreakout.h"
#include <arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_H3LIS331.h>
#include "FlashIAPLimits.h"
#include <math.h>
#include <serial.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "SocketHelpers.h"
#include <stdio.h>
#define PIN_TMP A7

//#define CLEAN_SDCARD
//#define RESET_COUNTER

//SD CARD CONFIG
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");

using namespace mbed;

//9DoF
Adafruit_LSM9DS1 MidIMU = Adafruit_LSM9DS1();
bool MidIMUFlag = true;
float acc[3];
float gyr[3];
float mag[3];
int16_t temp_1;

//Communication
BreakoutCarrierClass Breakout;
UART myUART = Breakout.UART1;
UART mySerial2(PG_14, PG_9, NC, NC);

uint32_t counter; 

//Sevenseg setup
breakoutPin A = PWM9;
breakoutPin B = PWM8;
breakoutPin C = PWM7;
breakoutPin D = PWM6;
breakoutPin E = PWM5;
breakoutPin Z = PWM4;
breakoutPin G = PWM3;
breakoutPin DP = PWM2;

breakoutPin GP5 = GPIO_5; //Sweep Trigger
breakoutPin GP6 = GPIO_6; //Sweep Trigger

char hex[5];
char data;

int seg[]{ A, B, C, D, E, Z, G, DP };
byte Chars[17][7]{
  { 1, 1, 1, 1, 1, 1, 0 },  //0
  { 0, 1, 1, 0, 0, 0, 0 },  //1
  { 1, 1, 0, 1, 1, 0, 1 },  //2
  { 1, 1, 1, 1, 0, 0, 1 },  //3
  { 0, 1, 1, 0, 0, 1, 1 },  //4
  { 1, 0, 1, 1, 0, 1, 1 },  //5
  { 1, 0, 1, 1, 1, 1, 1 },  //6
  { 1, 1, 1, 0, 0, 0, 0 },  //7
  { 1, 1, 1, 1, 1, 1, 1 },  //8
  { 1, 1, 1, 1, 0, 1, 1 },  //9
  { 1, 1, 1, 0, 1, 1, 1 },  //A/10
  { 0, 0, 1, 1, 1, 1, 1 },  //b/11
  { 1, 0, 0, 1, 1, 1, 0 },  //C/12
  { 0, 1, 1, 1, 1, 0, 1 },  //d/13
  { 1, 0, 0, 1, 1, 1, 1 },  //E/14
  { 1, 0, 0, 0, 1, 1, 1 },  //F/15
  { 0, 0, 0, 0, 0, 0, 0 }   //blank
};

void displayPrint(char n) {
  for (char i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[n][i]);
  }
}


bool accelCheck() {  //accelArr
  uint16_t accelSum = 0;
  for (int i = 0; i < 10; i++) {
    MidIMU.read();
    sensors_event_t a, m, g, temp;
    MidIMU.getEvent(&a, &m, &g, &temp);
    accelSum += a.acceleration.z;
  }
  float accelAvg = accelSum / 10;
  //int16_t acceleration = a.acceleration.z; //in m/s^2, pos
  //Serial.println( accelAvg );
  if (accelAvg > 30) {  //3 g's, in m/s^2
    return true;
  } else {
    return false;
  }
}

void setup() {
  pinMode(seg[0], OUTPUT);
  pinMode(seg[1], OUTPUT);
  pinMode(seg[2], OUTPUT);
  pinMode(seg[3], OUTPUT);
  pinMode(seg[4], OUTPUT);
  pinMode(seg[5], OUTPUT);
  pinMode(seg[6], OUTPUT);
  pinMode(seg[7], OUTPUT);
  pinMode(GP5, OUTPUT);
  pinMode(GP6, OUTPUT);

  analogWriteResolution(12);  // Limits ADCS to 12-bit resolution on portenta
  analogReadResolution(12);   // Same thing

  Serial.begin(115200);
  Breakout.I2C_0.begin();

  // I2C Scan
  Serial.println("Scanning I2C bus...");
  for (byte i = 1; i < 127; i++) {
    Breakout.I2C_0.beginTransmission(i);
    if (Breakout.I2C_0.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
    }
  }
  Serial.println("I2C scan done.");

  // 9 DoF Sensor Setup
  Serial.println("Initializing IMU...");
  uint8_t attempts = 0;
  while (!MidIMU.begin() && attempts < 5) {
    Serial.print("Attempt ");
    Serial.print(attempts + 1);
    Serial.println(" to initialize IMU failed.");
    attempts++;
    delay(1000); // Wait 1 second before retrying
  }
  if (attempts == 5) {
    Serial.println("Failed to initialize IMU after 5 attempts.");
    MidIMUFlag = false;
    displayPrint(0x05);
  } else {
    Serial.println("IMU initialized successfully.");
    displayPrint(0x00);
    MidIMU.setupAccel(MidIMU.LSM9DS1_ACCELRANGE_16G);
    MidIMU.setupMag(MidIMU.LSM9DS1_MAGGAIN_4GAUSS);
    MidIMU.setupGyro(MidIMU.LSM9DS1_GYROSCALE_2000DPS);
  }
}

void loop() {
  if (MidIMUFlag) {
    MidIMU.read();
    sensors_event_t a, m, g, temp;
    MidIMU.getEvent(&a, &m, &g, &temp);
    long t1 = millis();
    long t2 = 0;
    int count = 0;
    while(t2 - t1 < 1000){
      count++;
      acc[0] = a.acceleration.x + acc[0] ; 
      acc[1] = a.acceleration.y + acc[1];
      acc[2] = a.acceleration.z + acc[2];
      gyr[0] = g.gyro.x + gyr[0];
      gyr[1] = g.gyro.y + gyr[1];
      gyr[2] = g.gyro.z + gyr[2];
      mag[0] = g.magnetic.x + mag[0];
      mag[1] = g.magnetic.y + mag[1];
      mag[2] = g.magnetic.z + mag[2];
      t2 = millis();
    }
    //95.43, 936.25, 409.6
    Serial.print("Acceleration x: ");
    Serial.println((acc[0] / count));
    Serial.print("Acceleration y: ");
    Serial.println((acc[1] / count));
    Serial.print("Acceleration z: ");
    Serial.println((acc[2] / count));
    Serial.print("Gyro x: ");
    Serial.println((gyr[0] / count));
    Serial.print("Gyro y: ");
    Serial.println((gyr[1] / count));
    Serial.print("Gyro z: ");
    Serial.println((gyr[2] / count));
    Serial.print("Magnet x: ");
    Serial.println((mag[0] / count));
    Serial.print("Magnet y: ");
    Serial.println((mag[1] / count));
    Serial.print("Magnet z: ");
    Serial.println((mag[2] / count));

    acc[0] = 0;
    acc[1] = 0;
    acc[2] = 0;
    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;
    gyr[0] = 0; 
    gyr[1] = 0; 
    gyr[2] = 0; 

  } else {
    acc[0] = 0xffff;
    acc[1] = 0xffff;
    acc[2] = 0xffff;
    gyr[0] = 0xffff;
    gyr[1] = 0xffff;
    gyr[2] = 0xffff;
    mag[0] = 0xffff;
    mag[1] = 0xffff;
    mag[2] = 0xffff;
  }
}