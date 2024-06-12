#include <Arduino.h>
#include "Arduino_PortentaBreakout.h"
#include "SocketHelpers.h"
#include "RPC.h"
#include <FlashIAPBlockDevice.h>
#include "FlashIAPLimits.h"

using namespace rtos;
UART mySerial2(PG_14, PG_9, NC, NC);

int t1, t2, t3, t4, t5, t6; // Define these variables outside the loop()
Thread subtractThread;
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

volatile char sharedVariable;

char readSharedVariable() {
    return sharedVariable;
}

void writeSharedVariable(char value){
    sharedVariable = value;
}

void setup() {

    t1 = micros();
  enablefpu();
  ARTtoggle();
  // Initialize RPC library; this also boots the M4 core
  RPC.begin();
  Serial.begin(9600);
  mySerial2.begin(9600);
  
  //while (!Serial) {} // Uncomment this to wait until the Serial connection is ready
  //Serial.println(currentCPU());

  // Both CPUs will execute this instruction, just at different times
  randomSeed(analogRead(A0)); // Initializes the pseudo-random number generator

  if (currentCPU() == "M7") {
    RPC.bind("readSharedVariable", readSharedVariable);
    RPC.bind("writeSharedVariable", writeSharedVariable);
  } 
   t3 = micros();

}

void loop() {
  t4 = micros(); // Record the start time
  while (mySerial2.available() > 0) {
      char receivedData = mySerial2.read();
  if (currentCPU() == "M4") {
      RPC.call("writeSharedVariable", receivedData).as<int>();
      auto res = RPC.call("readSharedVariable", receivedData).as<int>();
      RPC.println(res);
    }
  }
    //RPC.println(currentCPU() + ": Time for calling variable " + String(t2 - t1));
    //RPC.println(currentCPU() + ": Time for printing over RPC " + String(t3 - t1));
  
  if (currentCPU() == "M7") {
    // On M7, let's print everything that is received over the RPC stream interface
    // Buffer it, otherwise all characters will be interleaved by other prints
    String buffer = "";
    while (RPC.available()) {
      buffer += (char)RPC.read(); // Fill the buffer with characters
    }

    if (buffer.length() > 0) {
      Serial.print(buffer);
      //Serial.println(currentCPU() + ": Time to print to Serial Monitor " + String(t5 - t4));
      //Serial.println(currentCPU() + ": Time since Initialization " + String(t5 - t6));
    }
  }
  t2 = micros();
  Serial.print("Time for Setup Execution (us): ");
  Serial.println(t3-t1);
  Serial.print("Time for Loop Execution (us): ");
  Serial.println(t2-t4);
  delay(1000);
}