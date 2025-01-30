#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "mbed.h"
#include "Arduino_PortentaBreakout.h"
#include <Wire.h>

void writeTCA9539(uint8_t reg, uint8_t value);
uint8_t readTCA9539(uint8_t reg);

#define TCA9539_ADDR 0x74

// TCA9539 register addresses
#define CONFIG_PORT0 0x06
#define CONFIG_PORT1 0x07
#define OUTPUT_PORT0 0x02
#define OUTPUT_PORT1 0x03
#define LAST_ARDUINO_PIN_NUMBER LEDB + 1
#define RESET_PIN LAST_ARDUINO_PIN_NUMBER + PE_3

void setup() {
  Wire.begin();  // Initialize I2C
  Serial.begin(9600);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(10);  // Hold reset low for 10ms
  digitalWrite(RESET_PIN, HIGH);
  delay(10);  // Wait for the device to initialize
  
}

void loop() {
    // Writing 0x00 to CONFIG_PORT0 and CONFIG_PORT1 makes all pins outputs
  writeTCA9539(CONFIG_PORT0, 0x00);  // Set all Port 0 pins as outputs
  writeTCA9539(CONFIG_PORT1, 0x00);  // Set all Port 1 pins as outputs

  // Set specific pins high
  //Set the first two pins and fourth pin of Port 0 high (0b00001011).
  writeTCA9539(OUTPUT_PORT0, 0b11111111);  // Set P0_0 and P0_1 high on Port 0
  writeTCA9539(OUTPUT_PORT1, 0b00000001);  // Set all pins low on Port 1

  Serial.println("GPIO pins set high on TCA9539");
  Serial.print("CONFIG_PORT0: ");
  Serial.println(readTCA9539(CONFIG_PORT0), BIN);
  Serial.print("CONFIG_PORT1: ");
  Serial.println(readTCA9539(CONFIG_PORT1), BIN);

  delay(5000);
}

// Function to write data to a register in the TCA9539
void writeTCA9539(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9539_ADDR);
  Wire.write(reg);      // Register address
  Wire.write(value);    // Value to write
  Wire.endTransmission();
}

uint8_t readTCA9539(uint8_t reg) {
  Wire.beginTransmission(TCA9539_ADDR);
  Wire.write(reg);
  Wire.requestFrom(TCA9539_ADDR, 1);
  return Wire.read();
}
