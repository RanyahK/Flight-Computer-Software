#include "Arduino.h"
#include "mbed.h"
#include "Arduino_PortentaBreakout.h"
#include <Wire.h>
#include <SPI.h>

void writeTCA9539(uint8_t reg, uint8_t value);
#define TCA9539_ADDR 0x74

// TCA9539 register addresses
#define CONFIG_PORT0 0x06
#define CONFIG_PORT1 0x07
#define OUTPUT_PORT0 0x02
#define OUTPUT_PORT1 0x03

const int WRITE = 0b0001000000000000;   
const int WRITEB = 0b1001000000000000;   

const int mask = 0x0FFF;  


void setup() {
  Wire.begin();  // Initialize I2C
  Serial.begin(9600);
  
  // Writing 0x00 to CONFIG_PORT0 and CONFIG_PORT1 makes all pins outputs
  writeTCA9539(CONFIG_PORT0, 0x00);  // Set all Port 0 pins as outputs
  writeTCA9539(CONFIG_PORT1, 0x00);  // Set all Port 1 pins as outputs

  // Set specific pins high
  //Set the first two pins and fourth pin of Port 0 high (0b00001011).
  writeTCA9539(OUTPUT_PORT0, 0b11111111);  // Set P0_0 and P0_1 high on Port 0
  writeTCA9539(OUTPUT_PORT1, 0b11111111);  // Set all pins low on Port 1

  Serial.println("GPIO pins set high on TCA9539");

  SPI.begin();

  pinMode(PI_0, OUTPUT);

}

void writeRegister(unsigned int thisRegister) {
  int dataToSend =  (thisRegister | WRITE);
 // Serial.println((unsigned int)dataToSend, HEX);

  digitalWrite(PI_0, LOW);
  SPI.transfer16(dataToSend);
  digitalWrite(PI_0, HIGH);
}
void writeRegisterB(unsigned int thisRegister) {
  int dataToSend =  (thisRegister | WRITEB);
 // Serial.println((unsigned int)dataToSend, HEX);

  digitalWrite(PI_0, LOW);
  SPI.transfer16(dataToSend);
  digitalWrite(PI_0, HIGH);
}


void loop() {
    writeRegisterB(1710);
  
    for(int i = 0; i<255; i++){
      int x = i * 16;
      writeRegister(x);
      delay(10);
    }
}

// Function to write data to a register in the TCA9539
void writeTCA9539(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9539_ADDR);
  Wire.write(reg);      // Register address
  Wire.write(value);    // Value to write
  Wire.endTransmission();
}

