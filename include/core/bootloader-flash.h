#ifndef INC_BOOTLOADER_FLASH_H
#define INC_BOOTLOADER_FLASH_H

#include "common-defines.h"

void bootloader_flash_erase_main_application(void);
void bootloader_flash_write(const uint32_t address, const uint8_t* data, const uint32_t length);

#endif/* INC_BOOTLOADER_FLASH_H */
