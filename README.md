6/14/2024 RK

Wallops Flight Tests UDIP 2024: 

test_blink.cpp: simple blink program for initial testing of PlatformIO setup environment using inbuilt LED on Portenta H7. 
Requires only Portenta H7.

test_9dof: averages data collected from magnetometer, accelerometer, and gyroscope every second on the x, y and z axis and outputs to serial monitor. 
Requires Flight Computer with LSM9DS1 soldered on, and 3.3V external power rail to power the LSM9DS1. 

test_TemperatureSensor: prints out Temperature in C and F over Serial Monitor at 115200 baud rate and 12 bit ADC resolution every second. Pin A7 on Portenta H7 is designated for the temperature sensor. 
Requires Flight Computer board with TMP36 soldered on, and 3.3V external power rail to power the TMP36. 

test_SevenSegmentDisplay: SSD should increment from 0-9 every second then loop around. SSD Pins are connected to PWM pins on Portenta H7 with 10k resistors.
Requires Flight Computer Board with onboard SSD.

test_SDCard: 1. Ensure #define CLEAN_SDCARD and  #define RESET_COUNTER are defined in line 17 and 18 respectively of sdCardTests. This will reset the counter to 0 and clean out all files under UDIP2024 in existing SD Card.
             2. Upload to Flight Computer. File Counter: 0 should print out in Serial Monitor every 10 seconds, and TestPacket... should be appended to UDIP0.dat accordingly.  
             3. Comment out CLEAN_SDCARD and RESET_COUNTER. Upload to Flight Computer. 
             4. File Counter should increment to 1 in Serial Monitor, and TestPacket... should be appended accordingly every 10 seconds to UDIP1.dat.  
             5. Press Reset Button on Portenta. File Counter should increment to 2 in Serial Monitor.  
             6. Check SD Card, should be 3 files, UDIP0.dat, UDIP1.dat, UDIP2.dat. Written as characters so can use notepad to open file and verify data.  
Requires Flight Computer board with onboard SD Card. 

test_SBCommFlightComputer: Science Board sends hex value to Flight Computer over UART every second, Flight Computer displays value on Seven Segment Display. 
Both Science Board and Flight Computer also print value in each of their respective Serial Monitors. 
SSD should successfully increment every second from 0-9.  
              1. Upload sbCommsTestScienceBoard to Science Board
              2. Upload sbCommsTestFlightComputer to Flight Computer
              3. Check GPIO 5 and GPIO 6 through PC104 with multimeter to see if set high to 3.3V using jumper cables
              4. Remove usb cables to observe SSD increment with exteral power supply.  
Requires Flight Computer with onboard SSD and Science Board, as well as 3.3V and 5V power supply rails. 

UDIP2024FlightSoftware: Requires full stack: EPS (or emulator with active 3.3V/5V/12V power rails), Flight Computer, and Science Board. 

Miscellaneous:

Note: M7 is master processsor and M4 is more of a coprocessor or slave processor. M7 has to boot up the M4 in order for M4 to be active. M7 has ownership over Serial0 (whatever prints out in the Serial Monitor)
Refer to Shared vs Distributed Memory Documentation. 

test_DualProcessorSharedDRAM: Uses L3 Domain of DRAM for M7 and M4 cores to store and share data. This domain in the cache is active even when in low power mode, therefore ideal when using for sensors.
Uses semaphores for cache coherence. M4 stores data in DRAM, and M7 fetches it. Serial Monitor prints out loop and setup time in microseconds to store and fetch 256 values.

test_DualProcessorRemoteProcedureCall: Uses RPC messaging to communicate between the two cores by binding and calling functions and over RPC messaging serial stream. M4 waits for data from Science Board over UART2, then calls function that is bound by M7 to write to a shared data variable. M4 also messages data to M7 over RPC stream. M7 waits until it receives data over RPC stream to then print out to Serial if needed. Serial Monitor prints out setup and loop execution time for data received from Science Board.


