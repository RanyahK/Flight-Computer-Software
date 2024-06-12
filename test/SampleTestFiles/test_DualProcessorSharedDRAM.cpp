/*
 * STM32H7M7 M4 and M7 using L3 SDRAM to share memory 
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include "Arduino_PortentaBreakout.h"
#include "SocketHelpers.h"
#include "RPC.h"
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include <FlashIAPBlockDevice.h>
#include "FlashIAPLimits.h"
#include "QSPIFBlockDevice.h"
#include "MBRBlockDevice.h"
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_flash_ex.h>
#include <stm32h7xx_hal_flash.h>
#include <serial.h>
#include <math.h>

using namespace rtos;

int tstart;
int tend;

int tstart1;
int tend1;


#define NOR_FLASH_SIZE 8 * 1024 // 16 MB
#define PARTITION_TYPE 0x0B // FAT 32
#define BUFFER_SIZE 4096

using namespace mbed;


#ifndef BUFF_CORES_SIZE			// Defines the buffer size to transfer data
	#define BUFF_CORES_SIZE	32
#endif

unsigned int buffer[BUFFER_SIZE];

typedef struct {
	// Flags to lock reading or writing
	unsigned char status_CM4_nreading:1;	// CM4 NOT reading flag
	unsigned char status_CM4_nwriting:1;	// CM4 NOT writing flag
	unsigned char status_CM7_nreading:1;	// CM7 NOT reading flag
	unsigned char status_CM7_nwriting:1;	// CM7 NOT writing flag
//	unsigned char status_CM7toCM4_has_data:1;
//	unsigned char status_CM4toCM7_has_data:1;

	unsigned int buff4to7[BUFF_CORES_SIZE];	// Buffer to transfer from core 4 to 7
	unsigned int buff7to4[BUFF_CORES_SIZE];	// Buffer to transfer from core 7 to 4

	// Stored buffer sizes. MUST BE LESS THAN BUFF_CORES_SIZE
	unsigned int buff4to7_size;
	unsigned int buff7to4_size;
} shared_data_TypeDef;


/*
 * Shared data struct
 *
 * It is configured at the beginning of D3 domain, AHB SRAM @ 0x38000000
 */
#define shared_data		((shared_data_TypeDef *) 0x38000000)


SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");
FlashIAPLimits flash;

void core_share_init();
void get_from_M4(unsigned int *buffer);	// Get data from M4 to M7
void get_from_M7(unsigned int *buffer); // Get data from M7 to M4
void put_to_M4(unsigned int buffer[], unsigned int buffer_size);	// put data from M7 to M4
void put_to_M7(unsigned int buffer[], unsigned int buffer_size);	// put data from M4 to M7

/**
 * Returns the CPU that's currently running the sketch (M7 or M4)
 * Note that the sketch has to be uploaded to both cores. 
 **/
String currentCPU() {
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    return "M7";
  } else {
    return "M4";
  }
}

volatile int sharedVariable = 0;

int readSharedVariable() {
    return sharedVariable;
}

void writeSharedVariable(int value) {
    sharedVariable = value;
}

UART mySerial2(PG_14, PG_9, NC, NC);

void ARTtoggle() {

		/* enable the ART accelerator */
		/* enable prefetch buffer */
		FLASH->ACR |= FLASH_ACR_PRFTEN;
		/* Enable flash instruction cache */
		FLASH->ACR |= FLASH_ACR_ICEN;
		/* Enable flash data cache */
		FLASH->ACR |= FLASH_ACR_DCEN;
		asm("wfi"); //wait for a systick interrupt i.e. delay(1)
	
}

void enablefpu() {
	  __asm volatile
	  (
	    "  ldr.w r0, =0xE000ED88    \n"  /* The FPU enable bits are in the CPACR. */
	    "  ldr r1, [r0]             \n"  /* read CAPCR */
	    "  orr r1, r1, #( 0xf << 20 )\n" /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
	    "  str r1, [r0]              \n" /* Write back the modified value to the CPACR */
	    "  dsb                       \n" /* wait for store to complete */
	    "  isb"                          /* reset pipeline now the FPU is enabled */
	  );
}

void setup() {
tstart = micros();
  enablefpu();
  //ARTtoggle();
  // Initialize RPC library; this also boots the M4 core
  core_share_init();
  Serial.begin(9600);
  mySerial2.begin(9600);

  // Both CPUs will execute this instruction, just at different times
  randomSeed(analogRead(A0)); // Initializes the pseudo-random number generator

  #ifdef CORE_CM7
    bootM4();
  #endif
tend = micros();
}

void printBuffer(unsigned int *buffer, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        Serial.print(buffer[i]);
        Serial.print(" ");
    }
    Serial.println(); // Print a newline at the end
}


const int numChars = 256;
char receivedChars[numChars];   // an array to store the received data
static byte ndx = 0;
boolean newData = false;
int ind;

void showNewData() {
    if (newData == true) {
        //Serial.print("This just in ... ");
        //Serial.println(receivedChars);
        //Serial.println(receivedChars[ndx]);
        newData = false;
    }
}
void recvWithEndMarker() {
    char endMarker = '\n';
    char rc;
   
    while (mySerial2.available() > 0 && newData == false) {
        rc = mySerial2.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

long temp;
unsigned int out;
void loop() {

  tstart1 = micros();
  while(ind < BUFFER_SIZE){
    buffer[ind] = random(256);
    ind++;
  }
    recvWithEndMarker();
    showNewData();
/*
  int t1;
  int t2;
  int t3;
  int t4;
  int t5;
*/
  /*while (mySerial2.available() > 0 && index < BUFFER_SIZE) {
      *receivedData = mySerial2.read();
      temp = atol(receivedData);
      out = (unsigned int)temp;
  }
*/
 // unsigned long start_time = micros();

  // Buffer is full, call put_to_M4 and pass the buffer
  put_to_M4(buffer, BUFFER_SIZE);

  //unsigned long end_time = micros();
  //Serial.print("Time to send data from M7 to M4: ");
 // Serial.println(end_time - start_time);

  // Test receiving data from M4 to M7
  //start_time = micros();
  get_from_M7(buffer);
 // printBuffer(buffer, shared_data->buff4to7_size);
  //end_time = micros();
  //Serial.print("Time to receive data from M4 to M7: ");
  //Serial.println(end_time - start_time);

  // Additional debug prints
 
  //Serial.println();

tend1 = micros();
Serial.print("Time for Loop Execution (us): ");
Serial.println(tend1 - tstart1);
Serial.print("Time for Setup Execution (us): ");
Serial.println(tend - tstart);


delay(1000);
ind = 0;
}

static unsigned int buffer_size_limited_4to7, buffer_size_limited_7to4;


/*
 * core_share_init()
 * Set nreading and nwriting flags to 1
 */
void core_share_init() {
	shared_data->status_CM4_nwriting = ~0;
	shared_data->status_CM4_nreading = ~0;
	shared_data->status_CM7_nwriting = ~0;
	shared_data->status_CM7_nreading = ~0;
}


/*
 * Get data from M4 to M7
 */
void get_from_M4(unsigned int *buffer) {
	if (shared_data->status_CM4_nwriting) {		// if M4 to M7 buffer has data
		shared_data->status_CM7_nreading = 0;	// Lock buffer

		for(unsigned int n = 0; n < shared_data->buff4to7_size; ++n) {
			*(buffer+n) = shared_data->buff4to7[n];	// Transfer data
		}
		shared_data->status_CM7_nreading = 1;	// Unlock buffer
	}
}


/*
 * Send data from M7 to M4
 */
void put_to_M4(unsigned int buffer[], unsigned int buffer_size) {

	if (shared_data->status_CM4_nreading) {	// if M7 to M4 buffer is not locked
		shared_data->status_CM7_nwriting = 0;	// Lock buffer

		buffer_size_limited_7to4 = (buffer_size > BUFF_CORES_SIZE) ? BUFF_CORES_SIZE : buffer_size;

		shared_data->buff7to4_size = buffer_size_limited_7to4;
		for (unsigned int n = 0; n < buffer_size_limited_7to4; ++n) {
			shared_data->buff7to4[n] = buffer[n];	// Transfer data
      //Serial.print(shared_data->buff7to4[n]);
		}
    //Serial.println();
		shared_data->status_CM7_nwriting = 1;

	}
}


/*
 * Get data from M7 to M4
 */
void get_from_M7(unsigned int *buffer) {
	if (shared_data->status_CM7_nwriting) {	// if M7 to M4 buffer has data
		shared_data->status_CM4_nreading = 0;		// Lock buffer

		for(unsigned int n = 0; n < shared_data->buff7to4_size; ++n) {
			*(buffer+n) = shared_data->buff7to4[n];	// Transfer data
//			shared_data->buff7to4[n] = 0;			// Clear buffer
      //Serial.print(*(buffer+n));
		}
     // Serial.println();
		shared_data->status_CM4_nreading = 1;	// Unlock buffer
	}
}


/*
 * Send data from M4 to M7
 */
void put_to_M7(unsigned int buffer[], unsigned int buffer_size) {

	if (shared_data->status_CM7_nreading) {	// if M4 to M7 buffer is not locked
		shared_data->status_CM4_nwriting = 0;		// Lock buffer

		buffer_size_limited_4to7 = (buffer_size > BUFF_CORES_SIZE) ? BUFF_CORES_SIZE : buffer_size;

		shared_data->buff4to7_size = buffer_size_limited_4to7;
		for (unsigned int n = 0; n < buffer_size_limited_4to7; ++n) {
			//shared_data->buff4to7[n] = buffer[n];	// Transfer data

    		}		
      //shared_data->status_CM4_nwriting = 1;

	}


}


