// Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
// License: MIT License (https://en.wikipedia.org/wiki/MIT_License)
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Spi camera, and can run on Arduino platform.
// This demo was made for ArduCAM Spi Camera.
// It needs to be used in combination with PC software.
// It can test ArduCAM Spi Camerafunctions

#include "ArducamLink.h"
#include "Arducam_Mega.h"
#include "SPI.h"
#include "Wire.h"

const int MEGA_CS = PIN_SPI_SS;
Arducam_Mega myCAM(MEGA_CS);
ArducamLink myUart;
uint8_t temp             = 0xff;
uint8_t sendFlag         = TRUE;
uint8_t commandBuff[20]  = {0};
uint8_t commandLength    = 0;
uint32_t readImageLength = 0;
uint8_t jpegHeadFlag     = 0;
uint8_t readBuffer(uint8_t* imagebuf, uint8_t length)
{
    if (imagebuf[0] == 0xff && imagebuf[1] == 0xd8) {
        jpegHeadFlag    = 1;
        readImageLength = 0;
        Serial.write(0xff);
        Serial.write(0xAA);
        Serial.write(0x01);

        Serial.write((uint8_t)(myCAM.getTotalLength() & 0xff));
        Serial.write((uint8_t)((myCAM.getTotalLength() >> 8) & 0xff));
        Serial.write((uint8_t)((myCAM.getTotalLength() >> 16) & 0xff));
        Serial.write((uint8_t)((myCAM.getTotalLength() >> 24) & 0xff));
        Serial.write(((CAM_IMAGE_PIX_FMT_JPG & 0x0f) << 4) | 0x01);
    }
    if (jpegHeadFlag == 1) {
        readImageLength += length;
        for (uint8_t i = 0; i < length; i++) {
            Serial.write(imagebuf[i]);
        }
    }
    if (readImageLength == myCAM.getTotalLength()) {
        jpegHeadFlag = 0;
        Serial.write(0xff);
        Serial.write(0xBB);
    }
    return sendFlag;
}

void stop_preivew()
{
    readImageLength = 0;
    jpegHeadFlag    = 0;
    uint32_t len    = 9;

    Serial.write(0xff);
    Serial.write(0xBB);
    Serial.write(0xff);
    Serial.write(0xAA);
    Serial.write(0x06);
    myUart.arducamUartWriteBuff((uint8_t*)&len, 4);
    Serial.print("streamoff");
    Serial.write(0xff);
    Serial.write(0xBB);
}

void setup()
{
    SPI.begin();
    Serial.begin(115200);
    Serial.write("Hello Arduino UNO!", 7);
    myCAM.begin();
    Serial.write("Mega start!", 8);
    myCAM.registerCallBack(readBuffer, 200, stop_preivew);
}

void loop()
{
    if (Serial.available()) {
        temp = Serial.read();
        delay(5);
        if (temp == 0x55) {
            while (Serial.available()) {
                commandBuff[commandLength] = Serial.read();
                if (commandBuff[commandLength] == 0xAA) {
                    break;
                }
                commandLength++;
            }
            Serial.flush();
            myUart.uartCommandProcessing(&myCAM, commandBuff);
            commandLength = 0;
        }
    }
    myCAM.captureThread();
}