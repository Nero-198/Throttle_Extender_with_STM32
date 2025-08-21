/*
 * flash_rom.cpp
 *
 *  Created on: April 18, 2024
 *      Author: iroen
 */

#include "flash_rom.h"
#include "main.h"

flash_rom::flash_rom()
{
    // TODO Auto-generated constructor stub
}

flash_rom::~flash_rom()
{
    // TODO Auto-generated destructor stub
}

HAL_StatusTypeDef flash_rom::write(uint32_t address, uint8_t *data, uint32_t size)
{
	HAL_StatusTypeDef status = HAL_FLASH_Unlock();
	if(status == HAL_OK){
	    uint32_t* pData = (uint32_t*)data;  // ポインタをuint32_t型にキャスト
	    for (uint32_t i = 0; i < size / sizeof(uint32_t); i++) // sizeはバイト単位なので、uint32_tのサイズで割る
	    {
	        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4, pData[i]) == HAL_ERROR)
	        {
	            return HAL_ERROR;
	        }
	    }
	}
	else{
	    return HAL_ERROR;
	}
	HAL_FLASH_Lock();
	return HAL_OK;
}

uint32_t flash_rom::read(uint32_t address, uint32_t size)
{
    uint32_t data = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        data = *(uint32_t *)(address + i);
    }
    return data;
}

void flash_rom::erase(uint32_t address, uint32_t size)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.PageAddress = address;
    EraseInitStruct.NbPages = size / FLASH_PAGE_SIZE;//STM32F103の場合、1ページは400なのでsize=400で1ページ消去
    uint32_t PageError;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Lock();
}
