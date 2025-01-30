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
uint32_t counter; 

//Sevenseg setup
breakoutPin A = PWM8;
breakoutPin B = PWM9;
breakoutPin C = PWM7;
breakoutPin D = PWM3;
breakoutPin E = PWM4;
breakoutPin Z = PWM5;
breakoutPin G = PWM6;
breakoutPin DP = PWM2;


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


void deleteAllFiles(const char *path) {
  DIR *dir;
  struct dirent *ent;

  Serial.println("Deleting all files in SDCARD: ");
  if ((dir = opendir(path)) != NULL) {
    // Iterate over all the files and directories within directory
    while ((ent = readdir(dir)) != NULL) {
      // Form the full path to the file
      char fullPath[256];
      snprintf(fullPath, sizeof(fullPath), "%s/%s", path, ent->d_name);

      // Delete the file
      if (remove(fullPath) == 0) {
        Serial.print("Deleted: ");
        Serial.println(ent->d_name);
      } else {
        Serial.print("Failed to delete: ");
        Serial.println(ent->d_name);
      }
    }
    closedir(dir);
  } else {
    // Could not open directory
    Serial.println("Error opening SDCARD");
    while (1)
      ;
  }
}
// Initialize SD CARD
void sd_init() {
  //Serial.println("Mounting SDCARD...");
  int err = fs.mount(&block_device);
  if (err) {
    // Reformat if we can't mount the filesystem
    // this should only happen on the first boot
    //Serial.println("No filesystem found, formatting... ");
    err = fs.reformat(&block_device);
  }
  if (err) {
    //Serial.println("Error formatting SDCARD ");
    displayPrint(0x00);
    while (1)
      ;
  }

  DIR *dir;
  struct dirent *ent;
  int dirIndex = 0;
  Serial.println("List SDCARD content: ");
  if ((dir = opendir("/fs")) != NULL) {
    // Print all the files and directories within directory (not recursively)
    while ((ent = readdir(dir)) != NULL) {
      Serial.println(ent->d_name);
      dirIndex++;
    }
  }
  closedir(dir);

  mkdir("fs/UDIP2024", 0777);  // 0777 full access permissions linux style

  Serial.println("Closed Dir");
#ifdef CLEAN_SDCARD
  deleteAllFiles("/fs");
#endif
}

/*everything will be saved to the same file, just put in if you're trying to save sweep pckt or sensor pckt*/

void sd_write(byte *packet) {

  char filename[256];
  snprintf(filename, sizeof(filename), "/fs/UDIP2024/UDIP%d.dat", counter);
  FILE *mf = fopen(filename, "ab");
  if (mf != NULL) {
    size_t bytesWritten = fwrite(packet, sizeof(uint8_t), 39, mf);

    fclose(mf);
    //Serial.println("File created and written successfully");
    //Serial.println(counter);
  } else {
    //Serial.println("Failed to open file for writing");
    displayPrint(0x00);
  }
}

void sd_read(int fileCountNumber) {

  char filename[32];
  snprintf(filename, sizeof(filename), "/fs/UDIP2024/UDIP%d.dat", fileCountNumber);

  FILE *fp = fopen(filename, "rb");
  if (fp != NULL) {
    int c;
    while ((c = fgetc(fp)) != EOF) {
      //Serial.println((char)c, BIN);
      //Serial.print((char)c, HEX);
    }
    fclose(fp);
    //Serial.println("\nFile read successfully");
  } else {
    //Serial.println("Failed to open file for reading");
  }
}

void flash_init() {
  auto [flashSize, startAddress, iapSize] = getFlashIAPLimits();

  // Create a block device on the available space of the flash
  FlashIAPBlockDevice blockDevice(startAddress, iapSize);

  // Initialize the Flash IAP block device and print the memory layout
  blockDevice.init();

  const auto eraseBlockSize = blockDevice.get_erase_size();
  const auto programBlockSize = blockDevice.get_program_size();
  const unsigned int requiredEraseBlocks = ceil(sizeof(counter) / (float)eraseBlockSize);
  const unsigned int requiredProgramBlocks = ceil(sizeof(counter) / (float)programBlockSize);
  const auto dataSize = requiredProgramBlocks * programBlockSize;


  // Read the current counter value from flash memory
  if (blockDevice.read(reinterpret_cast<char *>(&counter), 0, dataSize != 0)) { Serial.println("Error with reading counter"); }

#ifdef RESET_COUNTER
  // Increment the counter
  counter = 0;
#else
  counter = counter + 1;
#endif;

  // Erase a block starting at the offset 0 relative
  // to the block device start address
  blockDevice.erase(0, requiredEraseBlocks * eraseBlockSize);

  // Write the updated counter value back to flash memory
  blockDevice.program(reinterpret_cast<char *>(&counter), 0, dataSize);

  // Print the updated counter value
  Serial.print("Counter value: ");
  Serial.println(counter);

  // Deinitialize the device
  blockDevice.deinit();
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

  analogWriteResolution(12);  //Limits ADCS to 12-bit resolution on portenta
  analogReadResolution(12);   //Same thing

  Serial.begin(115200);

  flash_init();
  sd_init();

}

void loop(){

  byte testPacket[39] = { 'T', 'e', 's', 't', ' ', 'P', 'a', 'c', 'k', 'e', 't', ' ', 'D', 'a', 't', 'a', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J' };
    displayPrint(0x01);
    sd_write(testPacket);

  Serial.print("File Counter: ");
  Serial.println(counter);

  // Wait for a while before repeating the test
  delay(10000);
  
}