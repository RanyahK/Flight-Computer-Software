#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "mbed.h"

UART mySerial2(PA_0, PI_9, NC, NC);


String sendMessage;
String receivedMessage;

void setup() {
  Serial.begin(9600);    // Initialize the Serial monitor for debugging
  mySerial2.begin(9600);   // Initialize Serial1 for sending data
}

void loop() {
  if (mySerial2.available() > 0) { // Check if data is available to read
    String receivedMessage = mySerial2.readStringUntil('\n'); // Read the message
    Serial.print("Received: ");
    Serial.println(receivedMessage); // Print the received message
  }  
  else{
    mySerial2.println("hello");
  }
  delay(1000);
}