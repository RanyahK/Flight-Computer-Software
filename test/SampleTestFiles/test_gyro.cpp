#include "Arduino_PortentaBreakout.h"
#include <arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <serial.h>
#include "SocketHelpers.h"
#include <stdio.h>

#define LAST_ARDUINO_PIN_NUMBER LEDB + 1

static uint8_t constexpr REGISTER_PWR_MGMT_1             = 0x6B;
static uint8_t constexpr REGISTER_VALUE_RESET            = 0x80;
static uint8_t constexpr REGISTER_INT_PIN_CFG            = 0x37;
static uint8_t constexpr REGISTER_VALUE_BYPASS_EN        = 0x02;

static const uint8_t MOSI_PIN = PC_3;
static const uint8_t MISO_PIN = PC_2;
static const uint8_t SCLK_PIN = PI_1;
static const uint8_t CS_PIN  = LAST_ARDUINO_PIN_NUMBER + PD_4;
static const uint8_t INT_PIN = PG_3;
static const uint8_t LED0_PIN = LED_BUILTIN;
static const uint8_t LED1_PIN = LEDG;

uint8_t gyroWHOAMI(uint8_t thisRegister) {
   thisRegister |= 0x80;
     // Start the SPI transaction
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW); 

  SPI.transfer(thisRegister);
  // Response will be in ret
  auto status = SPI.transfer(0x00);
  // End the transaction
  SPI.endTransaction();
  // Deselect the remote SPI device
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  Serial.print("Status Exit: 0x");
  Serial.println(status, HEX);

  return status;
}

void gyroWriteReg(uint8_t reg, uint8_t val) {

     // Start the SPI transaction
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW); 

  SPI.transfer(reg);
  SPI.transfer(val);

  // End the transaction
  SPI.endTransaction();
  // Deselect the remote SPI device
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delay(10);
}


void gyroRST(uint8_t reg, uint8_t val) {

     // Start the SPI transaction
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW); 

  SPI.transfer(reg);
  SPI.transfer(val);

  // End the transaction
  SPI.endTransaction();
  // Deselect the remote SPI device
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delay(10);
}

void setup(void) {
    //  Pin Configuration
    pinMode(INT_PIN, INPUT);
    pinMode(LED0_PIN, OUTPUT);
    pinMode(LED1_PIN, OUTPUT);

    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);
    delay(100);
    // Initialise SPI and the MPU6500 IMU
    SPI.begin();
}

void loop(void) {
    gyroRST(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
    delay(10);
    gyroWriteReg(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);
    delay(10);
    gyroWHOAMI(0x75);
    delay(5000);

}

