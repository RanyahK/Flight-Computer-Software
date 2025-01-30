#include "Arduino_PortentaBreakout.h"
#include <arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_H3LIS331.h>
#include <math.h>
#include <serial.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "SocketHelpers.h"
#include <stdio.h>

using namespace mbed;
breakoutPin GP3 = GPIO_3;  //Sweep Trigger

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
}