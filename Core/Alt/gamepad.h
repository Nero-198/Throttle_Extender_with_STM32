/*
 * gamepad.h
 *
 *  Created on: January 27, 2024
 *      Author: iroen
 */
#ifndef INC_GAMEPAD_H_
#define INC_GAMEPAD_H_

#include "main.h"
#include "alt_main.h"
#include "usbd_customhid.h"
#include <vector>
#include "adc.h"

/*declaration Macro*/
#define __UINT16_TO_UINT8_LOW(NUMuint16) ((uint8_t)(NUMuint16))
#define __UINT16_TO_UINT8_HIGH(NUMuint16) ((uint8_t)((NUMuint16) >> 8))
#define    cbi( addr, bit)       addr &= ~(1 << bit)    // addrのbit目を'0'にする。
#define    sbi( addr, bit)       addr |= (1 << bit)       // addrのbit目を'1'にする。
/*------Setting Device Inputs------*/
#define NUM_of_ADC_12bit 2
#define NUM_of_Buttons 21
/*------Buffer Size Definite------*/
#define ADC_CONVERTED_DATA_BUFFER_SIZE	((uint32_t)	NUM_of_ADC_12bit)
#define BUTTONS_DATA_BUFFER_SIZE (((uint32_t)NUM_of_Buttons / 8)+1)     //FIXME 割り切れる場合は+1しないようにする。(現状は割り切れる場合も+1している。)
#define GAMEPAD_HID_AXIS_DATA_BUFFER_SIZE (NUM_of_ADC_12bit * 2)        //8bit長に区切ったときのサイズ(8bit*2=16bit=2byte
#define GAMEPAD_HID_BUTTONS_DATA_BUFFER_SIZE ((NUM_of_Buttons / 8)+1)   //8bit長に区切ったときのサイズ
/*------ KeyMatrix INPUT and OUTPUT Pin Counts ------*/
//#define NUM_of_IN 4
//#define NUM_of_OUT 6  //今回はキーマトリクスを使わないのでコメントアウト
#ifdef CUSTOM_HID_EPIN_SIZE
#undef CUSTOM_HID_EPIN_SIZE
#define CUSTOM_HID_EPIN_SIZE 7      //EPIN_SIZEとは、HIDのデータを送るときの1パケットのサイズ。このサイズは、HIDのディスクリプタで定義されている。この値を変更すると、HIDのディスクリプタも変更する必要がある。gamepadHIDの配列サイズはEPIN_SIZEと同一のサイズである。
#endif
/*------ Flash DATA Adress -------*/
#define FLASH_DATA_START_ADDR 0x0800FC00
#define FLASH_DATA_END_ADDR 0x08007FFF
/*------ ADC Calibrate Data Adress -------*/
#define FLASH_ADC_CALIBRATE_CENTER_DATA_X_AXIS_ADDR 0x0800FC00
#define FLASH_ADC_CALIBRATE_MAX_DATA_X_AXIS_ADDR 0x0800FC04
#define FLASH_ADC_CALIBRATE_MIN_DATA_X_AXIS_ADDR 0x0800FC08
#define FLASH_ADC_CALIBRATE_CENTER_DATA_Y_AXIS_ADDR 0x0800FC0C
#define FLASH_ADC_CALIBRATE_MAX_DATA_Y_AXIS_ADDR 0x0800FC10
#define FLASH_ADC_CALIBRATE_MIN_DATA_Y_AXIS_ADDR 0x0800FC14


class gamepad
{
public:
    gamepad();
    ~gamepad();

    typedef struct{
        uint8_t     buttons[BUTTONS_DATA_BUFFER_SIZE];
        uint8_t      axis[NUM_of_ADC_12bit * 2];
    }gamepadHID_t;

    gamepadHID_t gamepadHID;

    void readButtons();
    void readAxis();

    void Initialize();  //initializeにも使う

    int ADCcalibrate();

private:
    uint32_t* ADC_FLASH_cal_data;
    typedef struct{
        int32_t ADC_center;
        int32_t ADC_max;
        int32_t ADC_min;
    }ADCcalibrate_t;
    ADCcalibrate_t ADCcal[NUM_of_ADC_12bit];

    uint32_t ADC_read(ADC_HandleTypeDef *hadc);
    int FLASH_If_Erase(uint32_t StartSector, uint32_t EndSector);

};

#endif /* INC_GAMEPAD_H_ */
