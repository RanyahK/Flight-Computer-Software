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

  analogWriteResolution(12);  //Limits ADCS to 12-bit resolution on portenta
  analogReadResolution(12);   //Same thing

  Serial.begin(115200);
  myUART.begin(115200);
  mySerial2.begin(115200);

}

void loop(){

  while(!mySerial2.available())
  {
    ;
  } //once SB starts sweeps, waits until there is data in buffer to start reading

  mySerial2.readBytes(hex, 5);
  data = hex[0]; // Assuming the display will use the first character in the array

  // Flush the input buffer
  mySerial2.read();

  // Display the data on the seven-segment display
  displayPrint(data - '0'); // Convert ASCII character to corresponding integer
  Serial.println(hex);


  digitalWrite(GP5, HIGH);
  digitalWrite(GP6, HIGH);

}