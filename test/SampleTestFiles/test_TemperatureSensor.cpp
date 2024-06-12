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

//SD CARD CONFIG
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");

using namespace mbed;


uint16_t getTemp() {  
  uint16_t reading = analogRead(PIN_TMP);
  float voltage = reading * (3.3 / 4095.0);
  
  // Convert the voltage to temperature in Celsius
  // TMP36 has a scale factor of 10 mV/°C and 500 mV offset for 0°C
  float temperatureC = (voltage - 0.5) * 100;
  
  float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;
  
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" °C / ");
  Serial.print(temperatureF);
  Serial.println(" °F");
  return reading;
}

void setup() {
  analogWriteResolution(12);  //Limits ADCS to 12-bit resolution on portenta
  analogReadResolution(12);   //Same thing

  Serial.begin(115200);

}

void loop(){
  getTemp();
  delay(1000);
}