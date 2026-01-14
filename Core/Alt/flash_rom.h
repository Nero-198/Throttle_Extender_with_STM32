/*
 * flash_rom.h
 *
 *  Created on: April 18, 2024
 *      Author: iroen
 */

#ifndef INC_FLASH_ROM_H_
#define INC_FLASH_ROM_H_

#include "main.h"
#include "alt_main.h"

class Flash_rom
{
public:
    Flash_rom();
    ~Flash_rom();

    HAL_StatusTypeDef write(uint32_t address, uint8_t *data, uint32_t size);
    uint32_t read(uint32_t address, uint32_t size);
    void erase(uint32_t address, uint32_t size);
    private:

};

#endif /* INC_FLASH_ROM_H_ */
