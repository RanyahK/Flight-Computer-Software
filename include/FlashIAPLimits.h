/**
Helper functions for calculating FlashIAP block device limits
**/

// Ensures that this file is only included once
#pragma once 

#include <Arduino.h>
#include <FlashIAP.h>
#include <FlashIAPBlockDevice.h>

using namespace mbed;

#define 	FLASH_ACR_WRHF_VOS1_70MHZ   (0 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS1_185MHZ   (1 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS1_225MHZ   (2 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS2_55MHZ   (0 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS2_165MHZ   (1 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS2_225MHZ   (2 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS3_45MHZ   (0 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS3_135MHZ   (1 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_WRHF_VOS3_225MHZ   (2 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define 	FLASH_ACR_PRFTEN   (1 << 8)
#define 	FLASH_ACR_DCRST   (1 << 12)
#define 	FLASH_ACR_ICRST   (1 << 11)
#define 	FLASH_ACR_DCEN   (1 << 10)
#define 	FLASH_ACR_ICEN   (1 << 9)

// A helper struct for FlashIAP limits
struct FlashIAPLimits {
  size_t flash_size;
  uint32_t start_address;
  uint32_t available_size;
};

// Get the actual start address and available size for the FlashIAP Block Device
// considering the space already occupied by the sketch (firmware).
FlashIAPLimits getFlashIAPLimits()
{
  // Alignment lambdas
  auto align_down = [](uint64_t val, uint64_t size) {
    return (((val) / size)) * size;
  };
  auto align_up = [](uint32_t val, uint32_t size) {
    return (((val - 1) / size) + 1) * size;
  };

  size_t flash_size;
  uint32_t flash_start_address;
  uint32_t start_address;
  FlashIAP flash;

  auto result = flash.init();
  if (result != 0)
    return { };

  // Find the start of first sector after text area
  int sector_size = flash.get_sector_size(FLASHIAP_APP_ROM_END_ADDR);
  start_address = align_up(FLASHIAP_APP_ROM_END_ADDR, sector_size);
  flash_start_address = flash.get_flash_start();
  flash_size = flash.get_flash_size();

  result = flash.deinit();

  int available_size = flash_start_address + flash_size - start_address;
  if (available_size % (sector_size * 2)) {
    available_size = align_down(available_size, sector_size * 2);
  }

  return { flash_size, start_address, available_size };
}