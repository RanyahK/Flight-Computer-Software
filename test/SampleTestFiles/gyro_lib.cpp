#include "Arduino_PortentaBreakout.h"
#include <arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <serial.h>
#include "SocketHelpers.h"
#include <stdio.h>
#include <MPU6500_WE.h>
#define LAST_ARDUINO_PIN_NUMBER LEDB + 1

static const uint8_t MOSI_PIN = PC_3;
static const uint8_t MISO_PIN = PC_2;
static const uint8_t SCLK_PIN = PI_1;
static const uint8_t CS_PIN  = LAST_ARDUINO_PIN_NUMBER + PD_4;
static const uint8_t INT_PIN = PG_3;
static const uint8_t LED0_PIN = LED_BUILTIN;
static const uint8_t LED1_PIN = LEDG;

const int csPin = LAST_ARDUINO_PIN_NUMBER + PD_4;  // Chip Select Pin
const int mosiPin = LAST_ARDUINO_PIN_NUMBER + PC_3;  // "MOSI" Pin
const int misoPin = LAST_ARDUINO_PIN_NUMBER + PC_2;  // "MISO" Pin
const int sckPin = LAST_ARDUINO_PIN_NUMBER + PI_1;  // SCK Pin
bool useSPI = true;    // SPI use flag


/* There are two constructors for SPI: */
//MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, useSPI);

/* Use this one if you want to change the default SPI pins (only for ESP32 so far): */
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, mosiPin, misoPin, sckPin, useSPI);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }

  /* Choose the SPI clock speed, default is 8 MHz 
     This function must be used only after init(), not before */
  myMPU6500.setSPIClockSpeed(500000);

  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
    
  myMPU6500.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  myMPU6500.setGyrOffsets(45.0, 145.0, -105.0);
  myMPU6500.enableGyrDLPF();
  myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  //myMPU6500.enableAccAxes(MPU6500_ENABLE_XYZ);
  //myMPU6500.enableGyrAxes(MPU6500_ENABLE_XYZ);
}

void loop() {
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");

  delay(1000);
}
