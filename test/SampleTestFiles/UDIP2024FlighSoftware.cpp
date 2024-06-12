//Ranyah K, Matthew W, Alex B.
//UDIP 2024

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

//SD CARD CONFIG
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");

using namespace mbed;
//Define Packet Types
//Packet Header Positions
#define HEDR_POS_SYNC 0
#define HEDR_POS_COUNT 2
#define HEDR_POS_T_INITIAL 4
#define HEDR_POS_T_FINAL 8
#define HEDR_POS_TYPE 12
#define HEDR_POS_PYLD_LEN 13

#define HEDR_LEN 15

#define PCKT_SYNC_0 0x55  //ASCII 'U'
#define PCKT_SYNC_1 0x44  //ASCII 'D'

//Sweep Reference Payload Positions
#define SWP_POS_VPOS 0         //two byte unsigned int for reading of +12 voltage line
#define SWP_POS_VNEG 2         //two byte unsigned int for reading of -12 voltage line
#define SWP_POS_PD1_INITIAL 4  //two byte unsigned int for reading of PD1 at start of measurement
#define SWP_POS_PD1_FINAL 6    //two byte unsigned int for reading of PD1 at end of measurement

#define SWP_REF_LEN 8

//Constant Sweep Reference Payload Positions
#define SWP_CST_POS_VPOS 0  //two byte unsigned int for reading of +12 voltage line
#define SWP_CST_POS_VNEG 2  //two byte unsigned int for reading of -12 voltage line
#define SWP_CST_POS_DAC 4   //two byte unsigned int for reading of -12 voltage line

#define SWP_CST_REF_LEN 6

//Sweep ADC Postions
#define SWP_POS_DAC 0   //two byte unsigned int for reading of the voltage from DAC
#define SWP_POS_ADC0 2  //two byte unsigned int for reading of the 0 gain ADC
#define SWP_POS_ADC1 4  //two byte unsigned int for reading of the 1 gain ADC
#define SWP_POS_ADC2 6  //two byte unsigned int for reading of the 2 gain ADC

#define SWP_ADC_LEN 8





// Sweep Packet Header Definitions
#define TYPE_SENS 0x01
#define TYPE_MED_SWP 0x30       //standard sweep standard GND
#define TYPE_MED_SWP_RGND 0x50  // Rocket GND
#define TYPE_SPL_SWP 0x10       //special sweep standard GND
#define TYPE_SPL_SWP_RGND 0x70  // Rocket GND

//Sensor Payload Positions
#define SENS_POS_ACCEL_M 0  //3 two byte signed ints. Mid range accels
#define SENS_POS_ACCEL_H 6  //1 two byte signed int. High range accels
#define SENS_POS_GYRO 8     //3 two byte signed ints. Gyroscopes
#define SENS_POS_MAG 14     //3 two byte signed ints. Magnetometers
#define SENS_POS_TMP 20     //1 two byte unsigned int. Digital board temp
#define SENS_POS_PD1 22     //1 two byte unsigned int. Photodiode 1

const int senLen = 24;  //Length of Sensor Payload

#define PCKT_SYNC_0 0x55  //ASCII 'U'
#define PCKT_SYNC_1 0x44  //ASCII 'D'

//Sweep packet pos
#define SWP_POS_PD1_INITIAL 4  //two byte unsigned int for reading of PD1 at start of measurement
#define SWP_POS_PD1_FINAL 6    //two byte unsigned int for reading of PD1 at end of measurement


//9DoF
Adafruit_LSM9DS1 MidIMU = Adafruit_LSM9DS1();
bool MidIMUFlag = true;
int16_t acc[3];
int16_t gyr[3];
int16_t mag[3];
int16_t temp_1;

#define ADC_PD1 A3  //Port photodiode 1

//Temp
#define PIN_TMP A7
uint16_t tmpInput;

//Acceleration
Adafruit_H3LIS331 HighA = Adafruit_H3LIS331();
bool HighAFlag = true;
int16_t acc_h;

//Packet Functions
void makeHedr(byte *, uint16_t *);

//Sensor Packet Functions
void makeSensPckt(byte *, uint16_t *);
void makeSensPyld(byte *);
void sd_writeSweep();

//SD CARD FUNCTIONS & Variables
void sd_init();
void sd_write(byte *packet);
void flash_init();
void deleteAllFiles(const char *path);
void printPacket(byte *packet, int length);
void printPacketHex(byte *packet, int length);
void sd_read(int fileCountNumber);
void sweepInit();
void sd_write_sweep(byte *packet);
void sweepWriteSD();
bool accelCheck();
uint16_t getADC(int ADCpin);
uint16_t getTemp();

int tstart;
int tend;

int tstart1;
int tend1;

//Checking which CPU Portenta is using
String currentCPU() {
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    return "M7";
  } else {
    return "M4";
  }
}


//Communication
BreakoutCarrierClass Breakout;
UART myUART = Breakout.UART1;
UART mySerial2(PG_14, PG_9, NC, NC);


//Global Variables
uint16_t count = 0;
uint16_t PD_initial;
byte sensPckt[HEDR_LEN + senLen];
bool rcktGND = false;
unsigned long tInitial;
unsigned long tFinal;
int phase = 1;
int lcount = 0;
uint32_t launchTime;  //constant
uint32_t measurement_time;
byte swpPckt[2071];  //Array for storing sweep packets
uint32_t counter;    //counter for UDIP Files - new file every boot up for packet storage (stored in flash)
bool relay_pos = 0;   //Keeping Track of Ground Relay Switch State


void GroundCheck();  // Ground Switching

//Sevenseg setup
breakoutPin A = PWM9;
breakoutPin B = PWM8;
breakoutPin C = PWM7;
breakoutPin D = PWM6;
breakoutPin E = PWM5;
breakoutPin Z = PWM4;
breakoutPin G = PWM3;
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
void displayPrint(char n);
breakoutPin GP5 = GPIO_5; //Sweep Trigger
//#define CLEAN_SDCARD //Uncomment if want to delete all files in SDCARD
//#define RESET_COUNTER //Uncomment if want to reset counter back to 0


void setup() {
  //Serial & Flashcard Setup
  Serial.begin(115200);
  myUART.begin(115200);
  mySerial2.begin(115200);
  flash_init();
  sd_init();
  Breakout.I2C_0.begin();
  //Display setup
  pinMode(seg[0], OUTPUT);
  pinMode(seg[1], OUTPUT);
  pinMode(seg[2], OUTPUT);
  pinMode(seg[3], OUTPUT);
  pinMode(seg[4], OUTPUT);
  pinMode(seg[5], OUTPUT);
  pinMode(seg[6], OUTPUT);
  pinMode(seg[7], OUTPUT);
  //Temp sens
  pinMode(PIN_TMP, INPUT);
  pinMode(GP5, OUTPUT);
  pinMode(PWM0, OUTPUT);
  pinMode(PWM1, OUTPUT);
  analogWrite(PWM0, 0); // Make sure relay is OPEN
  analogWrite(PWM1, 0); // Make sure relay is OPEN
  analogWriteResolution(12);  //Limits ADCS to 12-bit resolution on portenta
  analogReadResolution(12);   //Same thing
  digitalWrite(GP5, LOW); //Set default state of Sweep Trigger

  //9 DoF Sensor Setup
  if (!MidIMU.begin())  // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    //Serial.println("Mid Range IMU Failed to start.");
    //displayPrint(0x04);
    uint8_t i = 0;
    while (!MidIMU.begin() & i < 5) {
      i++;
    }
    if (i == 5) {
      MidIMUFlag = false;
        displayPrint(0x05);
        delay(10000);
    }
  }
  if (MidIMUFlag) {
    MidIMU.setupAccel(MidIMU.LSM9DS1_ACCELRANGE_16G);
    MidIMU.setupMag(MidIMU.LSM9DS1_MAGGAIN_4GAUSS);
    MidIMU.setupGyro(MidIMU.LSM9DS1_GYROSCALE_2000DPS);
  }
  //High range accel setup
  if (!HighA.begin_I2C()) {
    //Serial.println("High Range accel fail to initialize");
    //displayPrint(0x05);
    uint8_t i = 0;
    while (!HighA.begin_I2C() & i < 5) {
      i++;
    }
    if (i == 5) {
      HighAFlag = false;
    }
  }
  if (HighAFlag) {
    HighA.setRange(H3LIS331_RANGE_100_G);
    HighA.setDataRate(LIS331_DATARATE_1000_HZ);
  }
}


void loop() {
  if (phase == 1) {  //Power on, Collect High Rate Sensor Data
    displayPrint(0x01);
    makeSensPckt(sensPckt, &count);
    sd_write(sensPckt);
    if (accelCheck()) {
      launchTime = millis();
      phase = 2;
    }
  } else if (phase == 2) {  //Accel threshold reached, Timer starts for Sweep conduction
    measurement_time = (millis() - launchTime);
    displayPrint(0x02);
    makeSensPckt(sensPckt, &count);
    sd_write(sensPckt);
    //Serial.println("To phase 3!");
    if (measurement_time >= 10 * 1000) {
      phase = 3;
    }

  } else if (phase == 3) {  //Sweeps begin 50 seconds after phase 2
    measurement_time = (millis() - launchTime);
    displayPrint(0x03);
    makeSensPckt(sensPckt, &count);
    sd_write(sensPckt);
    sweepInit();
    PD_initial = getADC(ADC_PD1);  //Grab initial PD reading before sweep
    //insert instruct scienceboard function
     GroundCheck();
    sweepWriteSD();
    if (measurement_time >= 400 * 1000) {
      phase = 4;
      //Serial.println("To phase 4!");
    }
  } else if (phase == 4) {  //After 350 seconds, stop sweeps, low rate sensor measurements
    analogWrite(PWM1, 0);
    digitalWrite(GP5, LOW);
    analogWrite(PWM0, 0); // Make sure relay is OPEN
    makeSensPckt(sensPckt, &count);
    sd_write(sensPckt);
    displayPrint(0x04);
    delay(10);
  }
}

void sweepWriteSD() {
  //Now reassign the type if needed
  if (swpPckt[HEDR_POS_TYPE] == 0x30) {  //Linear
    if (rcktGND) {
      swpPckt[HEDR_POS_TYPE] = 0x50;            //Rocket GND
    }                                           //if not, remain unchanged
  } else if (swpPckt[HEDR_POS_TYPE] == 0x10) {  //High Density
    if (rcktGND) {
      swpPckt[HEDR_POS_TYPE] = 0x70;  //Rocket GND
    }
  }
  //Now put in the PD values

  
  uint16_t ADC_Final = getADC(ADC_PD1);
  memcpy(&swpPckt[HEDR_LEN + SWP_POS_PD1_FINAL], &ADC_Final, 2);
  memcpy(&swpPckt[HEDR_LEN + SWP_POS_PD1_INITIAL], &PD_initial, 2);
  
  /*
  uint16_t mn = (swpPckt[HEDR_LEN + SWP_POS_PD1_FINAL+1]) << 8 | swpPckt[HEDR_LEN + SWP_POS_PD1_FINAL];
  uint16_t xyz = (swpPckt[HEDR_LEN + SWP_POS_PD1_INITIAL+1]) << 8 | swpPckt[HEDR_LEN + SWP_POS_PD1_INITIAL];
  Serial.print("PD FINAL: "); Serial.println((mn / 4) * (4.76/1023) );
  Serial.print("PD FINAL: "); Serial.println((xyz / 4) * (4.76/1023) );
  
  swpPckt[HEDR_LEN + SWP_POS_PD1_FINAL] = getADC(ADC_PD1);
  swpPckt[HEDR_LEN + SWP_POS_PD1_INITIAL] = PD_initial;
  */
  
  sd_write_sweep(swpPckt);
}



bool accelCheck() {  //accelArr
  uint16_t accelSum = 0;
  for (int i = 0; i < 10; i++) {
    MidIMU.read();
    sensors_event_t a, m, g, temp;
    MidIMU.getEvent(&a, &m, &g, &temp);
    accelSum += a.acceleration.z;
  }
  float accelAvg = accelSum / 10;
  //int16_t acceleration = a.acceleration.z; //in m/s^2, pos
  //Serial.println( accelAvg );
  if (accelAvg > 30) {  //3 g's, in m/s^2
    return true;
  } else {
    return false;
  }
}

void makeSensPckt(byte *pckt, uint16_t *count) {
  makeHedr(pckt, count);
  pckt[HEDR_POS_TYPE] = TYPE_SENS;
  memcpy(&pckt[HEDR_POS_PYLD_LEN], &senLen, 2);
  makeSensPyld(pckt);
  tFinal = millis();

  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal, 4);

  return;
}

void makeHedr(byte *pckt, uint16_t *count) {  //only making headers for Temp packets
  //Serial.println("Making Header");
  tInitial = millis();
  pckt[HEDR_POS_SYNC] = PCKT_SYNC_0;
  pckt[HEDR_POS_SYNC + 1] = PCKT_SYNC_1;
  memcpy(&pckt[HEDR_POS_COUNT], count, 2);
  (*count)++;
  memcpy(&pckt[HEDR_POS_T_INITIAL], &tInitial, 4);
  pckt[HEDR_POS_TYPE] = TYPE_SENS;
  memcpy(&pckt[HEDR_POS_PYLD_LEN], &senLen, 2);
  return;
}

void makeSensPyld(byte *pckt) {
  //Read mid range IMU
  if (MidIMUFlag) {
    MidIMU.read();
    sensors_event_t a, m, g, temp;
    MidIMU.getEvent(&a, &m, &g, &temp);
    //95.43, 936.25, 409.6
    acc[0] = int16_t(a.acceleration.x * 95.43);
    acc[1] = int16_t(a.acceleration.y * 95.43);
    acc[2] = int16_t(a.acceleration.z * 95.43);
    gyr[0] = int16_t(g.gyro.x * 936.25);
    gyr[1] = int16_t(g.gyro.y * 936.25);
    gyr[2] = int16_t(g.gyro.z * 936.25);
    mag[0] = int16_t(m.magnetic.x * 409.6);
    mag[1] = int16_t(m.magnetic.y * 409.6);
    mag[2] = int16_t(m.magnetic.z * 409.6);
  } else {
    acc[0] = 0xffff;
    acc[1] = 0xffff;
    acc[2] = 0xffff;
    gyr[0] = 0xffff;
    gyr[1] = 0xffff;
    gyr[2] = 0xffff;
    mag[0] = 0xffff;
    mag[1] = 0xffff;
    mag[2] = 0xffff;
  }

  //Read high range accel
  if (HighAFlag) {
    sensors_event_t high_a_event;
    HighA.getEvent(&high_a_event);
    //Library returns a float. This scales it properly and makes it a int16_t.
    acc_h = int16_t(high_a_event.acceleration.z * 33.4);
  } else {
    acc_h = 0xffff;
  }

  //Read port digital systems
  uint16_t pd1 = getADC(ADC_PD1);
  uint16_t tr1 = getTemp();

  //Populate Sensor Packet
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_M], &acc, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_H], &acc_h, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_GYRO], &gyr, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_MAG], &mag, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP], &tr1, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_PD1], &pd1, 2);


  return;
}

uint16_t getADC(int ADCpin) {
  uint16_t val = 0;
  uint32_t val_sum = 0;
  uint16_t val_max = 0;
  uint16_t val_min = 0xffff;
  for (int i = 0; i < 18; i++) {
    val = analogRead(ADCpin);
    if (val < val_min) {
      val_min = val;
    }
    if (val > val_max) {
      val_max = val;
    }
    val_sum += val;
  }
  val_sum -= (val_max + val_min);
  return (uint16_t(val_sum));
}

uint16_t getTemp() {  //analogReadResolution of 12
  uint16_t reading = analogRead(PIN_TMP);
  float voltage = reading * (3300 / 4096.0);  //ADC conversion
  uint16_t temperature = (voltage - 500) / 10;

  /*
  Serial.print("Temperature: ");
  Serial.println( (voltage - 500) / 10);
  Serial.print("Reading: ");
  Serial.println(reading);
  */

  return reading;
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
// Sevenseg Display function
void displayPrint(char n) {
  for (char i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[n][i]);
  }
}

//Checks to see if high density sweep was conducted, indicating end of sweep cycle.
//Flips ground state to opposite state
void GroundCheck() {
  //Everytime a high density sweep is detected, flip ground on OR off
   if (swpPckt[HEDR_POS_TYPE] == 0x10) {
    if (relay_pos == 1) {
      analogWrite(PWM0, 0);
      analogWrite(PWM1, 0);
      delay(15);
      rcktGND = false;
      relay_pos= !relay_pos;
    } else {
      analogWrite(PWM0, 4095);
      analogWrite(PWM1, 800);
      rcktGND = true;
      delay(15);
      relay_pos= !relay_pos;
    }
}
}
//Instruct SB To Start Sweeps
void sweepInit() {
  digitalWrite(GP5, HIGH);
  while(!mySerial2.available())
  {
    ;
  } //once SB starts sweeps, waits until there is data in buffer to start reading
  digitalWrite(GP5, LOW);
  mySerial2.readBytes(swpPckt, 2071);  //reads the whole packet (2072 bytes)
  mySerial2.read();  // to flush
  /*
  Serial.print("VPOS: "); Serial.println(((swpPckt[HEDR_LEN + SWP_POS_VPOS + 1] << 8 | swpPckt[HEDR_LEN + SWP_POS_VPOS]) / 4) * (4.76/1023) );
  Serial.print("VNEG: "); Serial.println(((swpPckt[HEDR_LEN + SWP_POS_VNEG + 1] << 8 | swpPckt[HEDR_LEN + SWP_POS_VNEG]) / 4) * (4.76/1023) );
  Serial.print("Count: "); Serial.println(swpPckt[HEDR_POS_COUNT + 1] << 8 | swpPckt[HEDR_POS_COUNT]);*/
  
}

//Write sweeps to SD CARD. Different length than sens payload so has its own function
void sd_write_sweep(byte *packet) {

  char filename[256];
  snprintf(filename, sizeof(filename), "/fs/UDIP2024/UDIP%d.dat", counter);

  FILE *mf = fopen(filename, "ab");
  if (mf != NULL) {
    size_t bytesWritten = fwrite(packet, sizeof(uint8_t), 2071, mf);

    fclose(mf);
    //Serial.println("File created and written successfully");
  } else {
    //Serial.println("Failed to open file for writing");
  }
}
