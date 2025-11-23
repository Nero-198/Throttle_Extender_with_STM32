/*
 * gamepad.cpp
 *
 *  Created on: Oct 17, 2023
 *      Author: iroen
 */
/*このコードはgamepadで用いる各種読み込みやピン操作をまとめた関数である。
 *gamepadHID_t型の構造体に読み込んだ値を格納し、それを他のファイルへ渡すものにする。
 */

#include "gamepad.h"
#include "alt_main.h"
#include "main.h"
#include "flash_rom.h"

//マクロ
#define READ_BIT_AND_PLACE(GPIO, PIN, POSITION) (((GPIO) & (1U << (PIN))) ? (1U << (POSITION)) : 0)

//インスタンス
//gamepad::ADCcalibrate_t ADCcalibrate[NUM_of_ADC_12bit];
flash_rom flash_rom;

//クラス
gamepad::gamepad()
{
    //インスタンス


}

gamepad::~gamepad()
{
    // TODO !
}

/*
GPIOA
PA2	    CAST_UP
PA3	    CAST_DOWN
PA4	    CAST_LEFT
PA5	    CAST_RIGHT
PA6	    CAST_PRESS
PA7	    COOL_UP
PA8	    PINKY
PA9	    TRIG_1
PA10	TRIG_2
PA15	TDC_PRESS
GPIOB
PB0	    COOL_DOWN
PB1 	COOL_LEFT
PB3	    ACQ_DOWN
PB4	    ACQ_UP
PB5	    ACQ_LEFT
PB8	    ACQ_RIGHT
PB9	    ACQ_PRESS
PB10	COOL_RIGHT
PB11	COOL_PRESS
PB14	SLIDE_FWD
PB15	SLIDE_AFT

	7	            6	            5	                4	            3	            2	            1	            0
0	CAST_R(PA5)	    CAST_L(PA4)	    CAST_DOWN(PA3)	    CAST_UP(PA2)	TDC_PRESS(PA15) PINKY(PA8)	    TRIG_2(PA10)    RIG_1(PA9)	    gamepadHID.button[0]
1	ACQ_DOWN(PB3)	ACQ_UP(PB4)	    COOL_PRESS(PB11)	COOL_R(PB10)	COOL_L(PB1)     COOL_DOWN(PB0)	COOL_UP(PA7)	CAST_PRESS(PA6)	gamepadHID.button[1]
2	24	            23	            22	                SLIDE_FWD(PB14)	SLIDE_AFT(PB15)	ACQ_PRESS(PB9)	ACQ_R(PB8)	    ACQ_L(PB5)	    gamepadHID.button[2]
3								                                                                                        TDC_X	        gamepadHID.axis[0]
4									                                                                                                    gamepadHID.axis[1]
5								                                                                                        TDC_Y	        gamepadHID.axis[2]
6									                                                                                                    gamepadHID.axis[3]

USBD_EPIN_SIZE = 7byte

*/
void gamepad::readButtons()
{
    uint32_t GPIOA_temp = 0;
    uint32_t GPIOB_temp = 0;
    uint32_t button_states = 0;
    GPIOA_temp = GPIOA->IDR;
    GPIOB_temp = GPIOB->IDR;
    //下のコードは、USBのボタン番号とGPIOのピン番号を対応させるためのものである。トリガーをボタン１に設定したいな～って時に使う。
    //GPIOA
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 9, 0);    // TRIGGER 1st(PA9) -> bit0
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 10, 1);   // TRIGGER 2nd(PA10) -> bit1
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 8, 2);    // PINKY(PA8) -> bit2
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 15, 3);   // TDC_depress(PA15) -> bit3
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 2, 4);    // Castle_UP(PA2) -> bit4
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 3, 5);    // Castle_DOWN(PA3) -> bit5
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 4, 6);    // Castle_LEFT(PA4) -> bit6
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 5, 7);    // Castle_RIGHT(PA5) -> bit7
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 6, 8);    // Castle_depress(PA6) -> bit8
    button_states |= READ_BIT_AND_PLACE(GPIOA_temp, 7, 9);    // COOL_UP(PA7) -> bit9
    //GPIOB
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 0, 10);   // COOL_DOWN(PB0) -> bit10
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 1, 11);   // COOL_LEFT(PB1) -> bit11
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 10, 12);  // COOL_RIGHT(PB10) -> bit12
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 11, 13);  // COOL_depress(PB11) -> bit13
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 4, 14);   // ACQ_UP(PB4) -> bit14
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 3, 15);   // ACQ_DOWN(PB3) -> bit15
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 5, 16);   // ACQ_LEFT(PB5) -> bit16
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 8, 17);   // ACQ_RIGHT(PB8) -> bit17
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 9, 18);   // ACQ_depress(PB9) -> bit18
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 15, 19);  // SLIDE_AFT(PB15) -> bit19
    button_states |= READ_BIT_AND_PLACE(GPIOB_temp, 14, 20);  // SLIDE_FWD(PB14) -> bit20
    /*
    --上のコードが動いたら下のコードは消す--
    GPIO_temp = ((GPIOA->IDR) & (1<<9)) >> 9-0 ;   //TRIGGER 1st(PA9) >> bit0
    GPIO_temp = ((GPIOA->IDR) & (1<<10)) >> 10-1 ; //TRIGGER 2nd(PA10) >> bit1
    GPIO_temp = ((GPIOA->IDR) & (1<<8)) >> 8-2 ; //PINKY(PA8) >> bit2
    GPIO_temp = ((GPIOA->IDR) & (1<<15)) >> 15-3 ; //TDC_depress(PA15) >> bit3
    GPIO_temp = ((GPIOA->IDR) & (1<<2)) << 4-2 ; //Castle_UP(PA2) >> bit4
    GPIO_temp = ((GPIOA->IDR) & (1<<3)) << 5-3 ; //Castle_DOWN(PA3) >> bit5
    GPIO_temp = ((GPIOA->IDR) & (1<<4)) << 6-4 ; //Castle_LEFT(PA4) >> bit6
    GPIO_temp = ((GPIOA->IDR) & (1<<5)) << 7-5 ; //Castle_RIGHT(PA5) >> bit7
    GPIO_temp = ((GPIOA->IDR) & (1<<6)) << 8-6 ; //Castle_depress(PA6) >> bit8
    GPIO_temp = ((GPIOA->IDR) & (1<<7)) << 9-7 ; //COOL_UP(PA7) >> bit9
    //GPIOB
    GPIO_temp = ((GPIOB->IDR) & (1<<0)) << 10-0 ; //COOL_DOWN(PB0) >> bit10
    GPIO_temp = ((GPIOB->IDR) & (1<<1)) << 11-1 ; //COOL_LEFT(PB1) >> bit11
    GPIO_temp = ((GPIOB->IDR) & (1<<10)) << 12-10 ; //COOL_RIGHT(PB10) >> bit12
    GPIO_temp = ((GPIOB->IDR) & (1<<11)) << 13-11 ; //COOL_depress(PB11) >> bit13
    GPIO_temp = ((GPIOB->IDR) & (1<<4)) << 14-4 ; //ACQ_UP(PB4) >> bit14
    GPIO_temp = ((GPIOB->IDR) & (1<<3)) << 15-3 ; //ACQ_DOWN(PB3) >> bit15
    GPIO_temp = ((GPIOB->IDR) & (1<<5)) << 16-5 ; //ACQ_LEFT(PB5) >> bit16
    GPIO_temp = ((GPIOB->IDR) & (1<<8)) << 17-8 ; //ACQ_RIGHT(PB8) >> bit17
    GPIO_temp = ((GPIOB->IDR) & (1<<9)) << 18-9 ; //ACQ_depress(PB9) >> bit18
    GPIO_temp = ((GPIOB->IDR) & (1<<15)) << 19-15 ; //SLIDE_AFT(PB15) >> bit19
    GPIO_temp = ((GPIOB->IDR) & (1<<14)) << 20-14 ; //SLIDE_FWD(PB14) >> bit20
    */
// button_statesを全ビット反転
    button_states = ~button_states;
//gamepadHIDインスタンスに値を渡す
    gamepadHID.buttons[0] = (uint8_t)(button_states & 0xFF);
    gamepadHID.buttons[1] = (uint8_t)((button_states >> 8) & 0xFF);
    gamepadHID.buttons[2] = (uint8_t)((button_states >> 16) & 0xFF);

    return;
}

void gamepad::readAxis() //ADCの値をgamepadHID.axisに格納する関数。ADCの値をそのまま格納するのではなく、16bitに変換して格納する。
{
    //ADC1
    __IO uint32_t ADC_val[NUM_of_ADC_12bit];
    __IO int32_t ADC_val_signed[NUM_of_ADC_12bit];
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    ADC_val[0] = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    //ADC2
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 100);
    ADC_val[1] = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    //ADC値を12bitから16bitに変換
    ADC_val[0] = ADC_val[0] << 4;
    ADC_val[1] = ADC_val[1] << 4;
    //int32_tに一旦変換する。
    ADC_val_signed[0] = (int32_t)ADC_val[0] - 0x8000; //32bitで計算しているから、オーバーフローとかは起きないはず。//uint32_tの値から中央の値(32768)を引く。
    ADC_val_signed[1] = (int32_t)ADC_val[1] - 0x8000;
    //calibration値を読み込む。


    //calibration値で補正
    ADC_val_signed[0] = ADC_val_signed[0] - ADCcal[0].ADC_center;
    ADC_val_signed[1] = ADC_val_signed[1] - ADCcal[1].ADC_center;
    float ADC_calibrate_coeficient_X_max = 0x7FFF / (float)ADCcal[0].ADC_max; //変換用係数を作る。//floatじゃないとダメ
    float ADC_calibrate_coeficient_X_min = -0x8000 / (float)ADCcal[0].ADC_min;
    float ADC_calibrate_coeficient_Y_max = 0x7FFF / (float)ADCcal[1].ADC_max;
    float ADC_calibrate_coeficient_Y_min = -0x8000 / (float)ADCcal[1].ADC_min;
//---debug用---
    //係数を1000倍してuintで表示する
    printf("ADC_calibrate_coeficient_X_max: %ld\r\n", (uint32_t)(ADC_calibrate_coeficient_X_max * 1000));
    printf("ADC_calibrate_coeficient_X_min: %ld\r\n", (uint32_t)(ADC_calibrate_coeficient_X_min * 1000));
    printf("ADC_calibrate_coeficient_Y_max: %ld\r\n", (uint32_t)(ADC_calibrate_coeficient_Y_max * 1000));
    printf("ADC_calibrate_coeficient_Y_min: %ld\r\n", (uint32_t)(ADC_calibrate_coeficient_Y_min * 1000));
    printf("ADC_val_signed[0]: %ld\r\n", ADC_val_signed[0]);
    printf("ADC_val_signed[1]: %ld\r\n", ADC_val_signed[1]);
//---end debug用---
    //係数を用いて値を補正する。0より上の値と0より下の値にそれぞれ係数を適応する。
    if (ADC_val_signed[0] > 0)
    {
        ADC_val_signed[0] = ADC_val_signed[0] * ADC_calibrate_coeficient_X_max;
    }
    else
    {
        ADC_val_signed[0] = ADC_val_signed[0] * ADC_calibrate_coeficient_X_min;
    }
    if (ADC_val_signed[1] > 0)
    {
        ADC_val_signed[1] = ADC_val_signed[1] * ADC_calibrate_coeficient_Y_max;
    }
    else
    {
        ADC_val_signed[1] = ADC_val_signed[1] * ADC_calibrate_coeficient_Y_min;
    }
    printf("ADC_val_signed[0]: %ld\r\n", ADC_val_signed[0]);
    printf("ADC_val_signed[1]: %ld\r\n", ADC_val_signed[1]);
    //値が16bitを超えた場合に、上限/下限に抑える。//(これはキャリブレーションで最大値が正しく取れてない場合、係数が大きくなりすぎるので、それに対応するため）
    if (ADC_val_signed[0] > 0x7FFF)
    {
        ADC_val_signed[0] = 0x7FFF;
    }
    if (ADC_val_signed[1] > 0x7FFF)
    {
        ADC_val_signed[1] = 0x7FFF;
    }
    if (ADC_val_signed[0] < -0x8000)
    {
        ADC_val_signed[0] = -0x8000;
    }
    if (ADC_val_signed[1] < -0x8000)
    {
        ADC_val_signed[1] = -0x8000;
    }
    //USBに送るためにuint32_tに変換する。
    ADC_val[0] = (uint32_t)(ADC_val_signed[0]);
    ADC_val[1] = (uint32_t)(ADC_val_signed[1]);
    //ADC_Val[1]は値が反転していたので反転させる
    ADC_val[1] = 0xFFFF - ADC_val[1];

    //gamepadHIDインスタンスに値を渡す
    //この時渡す値を2つの8bitに分ける
    gamepadHID.axis[0] = (int8_t)(ADC_val[0] & 0x000000FF);
    gamepadHID.axis[1] = (int8_t)((ADC_val[0] & 0x0000FF00) >> 8);
    gamepadHID.axis[2] = (int8_t)(ADC_val[1] & 0x000000FF);
    gamepadHID.axis[3] = (int8_t)((ADC_val[1] & 0x0000FF00) >> 8);
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop(&hadc2);
    printf("\033[2J");
    printf("ADC1: %lu\r\n", ADC_val[0]);
    printf("ADC2: %lu\r\n", ADC_val[1]);
    return;
}

int gamepad::ADCcalibrate()
{
    uint8_t fanc_err_status = 0;
    //uint32_t status = 0;
    //status = FLASH_If_Erase(11, 11);  //FIXME: ADCcalibration機能を先に完成させる。保存動作は後。
    ADCcal[0].ADC_center = ((int32_t)(gamepad::ADC_read(&hadc1) << 4)) - 0x8000;
    ADCcal[1].ADC_center = ((int32_t)(gamepad::ADC_read(&hadc2) << 4)) - 0x8000;
    ADCcal[0].ADC_max = 0;
    ADCcal[1].ADC_max = 0;
    ADCcal[0].ADC_min = 0;
    ADCcal[1].ADC_min = 0;
    flash_rom.erase(0x08007c00, 0x0400); //キャリブレーション値を消去する。

    while (HAL_GPIO_ReadPin(TDC_CALBRATION_GPIO_Port,TDC_CALBRATION_Pin) == GPIO_PIN_RESET)
    {
        int32_t ADCval[] = {(((int32_t)(gamepad::ADC_read(&hadc1) << 4)) - 0x8000), (((int32_t)(gamepad::ADC_read(&hadc2) << 4)) - 0x8000)};  //FIXME, ADCが2個である事を前提としている。
        ADCval[0] = ADCval[0] - ADCcal[0].ADC_center;
        ADCval[1] = ADCval[1] - ADCcal[1].ADC_center;
        if (ADCcal[0].ADC_max < ADCval[0])
        {
            ADCcal[0].ADC_max = ADCval[0];
        }
        if (ADCcal[1].ADC_max < ADCval[1])
        {
            ADCcal[1].ADC_max = ADCval[1];
        }
        if (ADCcal[0].ADC_min > ADCval[0])
        {
            ADCcal[0].ADC_min = ADCval[0];
        }
        if (ADCcal[1].ADC_min > ADCval[1])
        {
            ADCcal[1].ADC_min = ADCval[1];
        }
        printf("\033[2J");
        printf("ADC_val_X: %ld\r\n", ADCval[0]);
        printf("ADC_val_Y: %ld\r\n", ADCval[1]);
        printf("ADC_center_X: %ld\r\n", ADCcal[0].ADC_center);
        printf("ADC_center_Y: %ld\r\n", ADCcal[1].ADC_center);
        printf("ADC_max_X: %ld\r\n", ADCcal[0].ADC_max);
        printf("ADC_min_X: %ld\r\n", ADCcal[0].ADC_min);
        printf("ADC_max_Y: %ld\r\n", ADCcal[1].ADC_max);
        printf("ADC_min_Y: %ld\r\n", ADCcal[1].ADC_min);
    }
    //最大値と最小値が0のままだった場合、のちの処理で0除算が発生するので、エラーを返す。
    if (ADCcal[0].ADC_max == 0 || ADCcal[1].ADC_max == 0 || ADCcal[0].ADC_min == 0 || ADCcal[1].ADC_min == 0)
    {
        fanc_err_status = 1;
        ADCcal[0].ADC_max = 0x0001;
        ADCcal[1].ADC_max = 0x0001;
        ADCcal[0].ADC_min = -0x0001;
        ADCcal[1].ADC_min = -0x0001;
    }
    //キャリブレーション値を保存する。
    flash_rom.write(0x08007c00, (uint8_t *)&ADCcal[0], sizeof(ADCcal[0]));
    flash_rom.write(0x08007c00 + sizeof(ADCcal[0]), (uint8_t *)&ADCcal[1], sizeof(ADCcal[1]));

    printf("\033[2J");
    printf("ADC_center_X: %ld\r\n", ADCcal[0].ADC_center);
    printf("ADC_center_Y: %ld\r\n", ADCcal[1].ADC_center);
    printf("ADC_max_X: %ld\r\n", ADCcal[0].ADC_max);
    printf("ADC_min_X: %ld\r\n", ADCcal[0].ADC_min);
    printf("ADC_max_Y: %ld\r\n", ADCcal[1].ADC_max);
    printf("ADC_min_Y: %ld\r\n", ADCcal[1].ADC_min);
    return fanc_err_status;
}

uint32_t gamepad::ADC_read(ADC_HandleTypeDef *hadc) //ADCの値を読む関数。readAxis()とことなり、ADCの値をそのままreturnする。
{
    uint32_t ADC_val = 0;
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    ADC_val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return ADC_val;
}
void gamepad::Initialize(){
    memcpy(&(ADCcal[0].ADC_center), (const void*)0x08007c00, sizeof(uint32_t));
    memcpy(&(ADCcal[0].ADC_max), (const void*)(0x08007c00 + 4), sizeof(uint32_t));
    memcpy(&(ADCcal[0].ADC_min), (const void*)(0x08007c00 + 4*2), sizeof(uint32_t));
    memcpy(&(ADCcal[1].ADC_center), (const void*)(0x08007c00 + 4*3), sizeof(uint32_t));
    memcpy(&(ADCcal[1].ADC_max), (const void*)(0x08007c00 + 4*4), sizeof(uint32_t));
    memcpy(&(ADCcal[1].ADC_min), (const void*)(0x08007c00 + 4*5), sizeof(uint32_t));
}
