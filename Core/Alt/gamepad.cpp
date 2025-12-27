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
#include "stm32f1xx_hal_def.h"

//マクロ
#define READ_BIT_AND_PLACE(GPIO, PIN, POSITION) (((GPIO) & (1U << (PIN))) ? (1U << (POSITION)) : 0)

//インスタンス
//gamepad::ADCcalibrate_t ADCcalibrate[NUM_of_ADC_12bit];
Flash_rom flash_rom;

//クラス
gamepad::gamepad()
{
}

gamepad::~gamepad()
{
    // TODO !
}

/*
GPIOA
PA4     TDC1_PUSH
PA5     TDC2_PUSH
GPIOB
PB0	    BRK_RET
PB1 	BRK_EXT
PB3	    SLIDER_DOWN
PB4	    SLIDER_HALF_DOWN
PB5	    SLIDER_PUSH
PB6	    SLIDER_HALF_UP
PB7	    SLIDER_UP
PB8     TDC_CALIBRATION
PB10	DOGFIGHT
PB11	MISSILE
PB12    PINKY_AFT
PB13    PINKY_FWD

---ADC---
GPIOA
PA0     TDC1_X
PA1     TDC1_Y
PA2     TDC2_X
PA3     TDC2_Y

	7	            6	                5	                    4                   3	                    2                   1	                0
0	BRK_RET(PB0)	SLIDER_DOWN(PB3)	SLIDER_HALF_DOWN(PB4)   SLIDER_PUSH(PB5)    SLIDER_HALF_UP(PB6) 	SLIDER_UP(PB7)      TDC2_PUSH(PA5)	    TDC1_PUSH(PA4)      gamepadHID.button[0]
1	PADDING     	PADDING	            PADDING     	        PINKY_FWD(PB13)     PINKY_AFT(PB12)         MISSILE(PB11)       DOGFIGHT(PB10)      BRK_EXT(PB1) 	    gamepadHID.button[1]
2								                                                                                                                    TDC1_X              gamepadHID.axis[0]
3									                                                                                                                                    gamepadHID.axis[1]
4								                                                                                                                    TDC1_Y	            gamepadHID.axis[2]
5									                                                                                                                                    gamepadHID.axis[3]
6								                                                                                                                    TDC2_X	            gamepadHID.axis[4]
7									                                                                                                                                    gamepadHID.axis[5]
8								                                                                                                                    TDC2_Y	            gamepadHID.axis[6]
9									                                                                                                                                    gamepadHID.axis[7]

USBD_EPIN_SIZE = 10byte

---GPIO - HID button num 対応付け---
gamepadHID.button[0] bits (LSB→MSB):

bit0: TDC1_PUSH (PA4)
bit1: TDC2_PUSH (PA5)
bit2: SLIDER_UP (PB7)
bit3: SLIDER_HALF_UP (PB6)
bit4: SLIDER_PUSH (PB5)
bit5: SLIDER_HALF_DOWN (PB4)
bit6: SLIDER_DOWN (PB3)
bit7: BRK_RET (PB0)
gamepadHID.button[1] bits (LSB→MSB):

bit0: BRK_EXT (PB1)
bit1: DOGFIGHT (PB10)
bit2: MISSILE (PB11)
bit3: PINKY_AFT (PB12)
bit4: PINKY_FWD (PB13)
bit5: PADDING
bit6: PADDING
bit7: PADDING

軸 (gamepadHID.axis の 16bit扱いなら [lo,hi] の順で 2バイトずつ):
axis[0..1]: TDC1_X (PA0)
axis[2..3]: TDC1_Y (PA1)
axis[4..5]: TDC2_X (PA2)
axis[6..7]: TDC2_Y (PA3)

*/
void gamepad::readButtons()
{
    uint32_t GPIOA_temp = 0;
    uint32_t GPIOB_temp = 0;
    uint32_t button_status = 0;
    GPIOA_temp = GPIOA->IDR;
    GPIOB_temp = GPIOB->IDR;

    //プロジェクトごとに変更する所
    //下のコードは、USBのボタン番号とGPIOのピン番号を対応させるためのものである。トリガーをボタン１に設定したいな～って時に使う。
    //uint8_t gamepadHID.button[0]
    button_status |= READ_BIT_AND_PLACE(GPIOA_temp, 4, 0);  // TDC1_PUSH(PA4) -> bit0
    button_status |= READ_BIT_AND_PLACE(GPIOA_temp, 5, 1);  // TDC2_PUSH(PA5) -> bit1
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 7, 2);  // SLIDER_UP(PB7) -> bit2
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 6, 3);  // SLIDER_HALF_UP(PB6) -> bit3
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 5, 4);  // SLIDER_PUSH(PB5) -> bit4
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 4, 5);  // SLIDER_HALF_DOWN(PB4) -> bit5
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 3, 6);  // SLIDER_DOWN(PB3) -> bit6
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 0, 7);  // BRK_RET(PB0) -> bit7

    //uint8_t gamepadHID.button[1]
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 1, 8);   // BRK_EXT(PB1) -> bit8
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 10, 9);  // DOGFIGHT(PB10) -> bit9
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 11, 10); // MISSILE(PB11) -> bit10
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 12, 11); // PINKY_AFT(PB12) -> bit11
    button_status |= READ_BIT_AND_PLACE(GPIOB_temp, 13, 12); // PINKY_FWD(PB13) -> bit12

    //ボタンの論理反転処理。押されたときに1になるようにする。
    button_status = ~button_status;
//gamepadHIDインスタンスに値を渡す
    for(uint32_t i = 0; i < BUTTONS_DATA_BUFFER_SIZE; i++)
    {
        USB_HID_Report.buttons[i] = (uint8_t)(button_status >> (i * 8) & 0xFF);
    }

    return;
}

void gamepad::readAxis() //ADCの値をgamepadHID.axisに格納する関数。ADCの値をそのまま格納するのではなく、16bitに変換して格納する。
{
    __IO int32_t ADC_val_signed_12bit_to_16bit[NUM_of_ADC_12bit]; //オーバーフロー防止のためにint32_tで確保

    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        ADC_val_signed_12bit_to_16bit[i] = (int32_t)(ADC_DMA_val[i] << 4); //元のデータが12bitなので、16bitにするために4bitシフト
    }

    //calibration値で補正
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        ADC_val_signed_12bit_to_16bit[i] = ADC_val_signed_12bit_to_16bit[i] - gamepadHID.axis[i].cal_center; //この処理で中心値を0にする
    }
    

    //calibration値で補正
    // ADC_val_signed[0] = ADC_val_signed[0] - ADCcal[0].ADC_center;
    // ADC_val_signed[1] = ADC_val_signed[1] - ADCcal[1].ADC_center;
    // float ADC_calibrate_coeficient_X_max = 0x7FFF / (float)ADCcal[0].ADC_max; //変換用係数を作る。//floatじゃないとダメ
    // float ADC_calibrate_coeficient_X_min = -0x8000 / (float)ADCcal[0].ADC_min;
    // float ADC_calibrate_coeficient_Y_max = 0x7FFF / (float)ADCcal[1].ADC_max;
    // float ADC_calibrate_coeficient_Y_min = -0x8000 / (float)ADCcal[1].ADC_min;

    //係数を用いて値を補正する。0より上の値と0より下の値にそれぞれ係数を適応する。//0はそのまま0
    for (int i = 0; i < NUM_of_ADC_12bit; i++){
        if (ADC_val_signed_12bit_to_16bit[i] > 0)
        {
            ADC_val_signed_12bit_to_16bit[i] = (int32_t)(ADC_val_signed_12bit_to_16bit[i] * gamepadHID.axis[i].cal_pos_coeficient);
        }
        else
        {
            ADC_val_signed_12bit_to_16bit[i] = (int32_t)(ADC_val_signed_12bit_to_16bit[i] * gamepadHID.axis[i].cal_neg_coeficient);
        }
    }

    //値が16bitを超えた場合に、上限/下限に抑える。//(これはキャリブレーションで最大値が正しく取れてない場合、係数が大きくなりすぎるので、それに対応するため）
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        if (ADC_val_signed_12bit_to_16bit[i] > 0x7FFF)
        {
            ADC_val_signed_12bit_to_16bit[i] = 0x7FFF;
        }
        if (ADC_val_signed_12bit_to_16bit[i] < -0x8000)
        {
            ADC_val_signed_12bit_to_16bit[i] = -0x8000;
        }
    }

    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        USB_HID_Report.axis[i*2] = (uint8_t)(ADC_val_signed_12bit_to_16bit[i] & 0x00FF);          //下位8bit
        USB_HID_Report.axis[i*2 + 1] = (uint8_t)((ADC_val_signed_12bit_to_16bit[i] & 0xFF00) >> 8);   //上位8bit
    }
    return;
}

int gamepad::ADCcalibrate()
{
    uint8_t fanc_err_status = 0;
    int32_t ADCmax[NUM_of_ADC_12bit]= {0};
    int32_t ADCmin[NUM_of_ADC_12bit]= {0};
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        gamepadHID.axis[i].cal_center = (uint16_t)(ADC_DMA_val[i] << 4); //中心値だけは初回に設定しておく。
        gamepadHID.axis[i].cal_pos_coeficient = 0.0f;
        gamepadHID.axis[i].cal_neg_coeficient = 0.0f;
    }
    flash_rom.erase(0x08007c00, 0x0400); //キャリブレーション値を消去する。

//calibrationスイッチが押されている間、ADCの最大値と最小値を取得する。
    while (HAL_GPIO_ReadPin(TDC_CALBRATION_GPIO_Port,TDC_CALBRATION_Pin) == GPIO_PIN_RESET)
    {
        for (int i = 0; i < NUM_of_ADC_12bit; i++)
        {
            int32_t ADC_val = (((int32_t)(ADC_DMA_val[i] << 4))) - gamepadHID.axis[i].cal_center;//元のデータが12bitなので、16bitにするために4bitシフト
            if(ADCmax[i] < ADC_val)
            {
                ADCmax[i] = ADC_val;
            }
            if(ADCmin[i] > ADC_val)
            {
                ADCmin[i] = ADC_val;
            }
        }
        #ifdef DEBUG
        printf("\033[2J");
        printf("ADC_val_X: %ld\r\n", ADCval[0]);
        printf("ADC_val_Y: %ld\r\n", ADCval[1]);
        printf("ADC_center_X: %ld\r\n", ADCcal[0].ADC_center);
        printf("ADC_center_Y: %ld\r\n", ADCcal[1].ADC_center);
        printf("ADC_max_X: %ld\r\n", ADCcal[0].ADC_max);
        printf("ADC_min_X: %ld\r\n", ADCcal[0].ADC_min);
        printf("ADC_max_Y: %ld\r\n", ADCcal[1].ADC_max);
        printf("ADC_min_Y: %ld\r\n", ADCcal[1].ADC_min);
        #endif //DEBUG
    }
    //最大値と最小値が0のままだった場合、のちの処理で0除算が発生するので、エラーを返す。
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        if(ADCmax[i] == 0||ADCmin[i] == 0)
        {
            fanc_err_status = 1;
            ADCmax[i] = 0x0001; //0除算防止のために1を代入
            ADCmin[i] = -0x0001;
        }
    }
    //ADCの値から補正係数を計算する。
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        gamepadHID.axis[i].cal_pos_coeficient = 32767.0f / (float)ADCmax[i]; //変換用係数を作る。
        gamepadHID.axis[i].cal_neg_coeficient = -32768.0f / (float)ADCmin[i];
    }
    //キャリブレーション値を保存する。
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        flash_rom.write(0x08007c00 + (sizeof(uint32_t))*3*i, (uint8_t *)&(gamepadHID.axis[i].cal_center), sizeof(uint32_t));
        flash_rom.write(0x08007c00 + (sizeof(float))*3*i + sizeof(float), (uint8_t *)&(gamepadHID.axis[i].cal_pos_coeficient), sizeof(float));
        flash_rom.write(0x08007c00 + (sizeof(float))*3*i + sizeof(float)*2, (uint8_t *)&(gamepadHID.axis[i].cal_neg_coeficient), sizeof(float));
    }
    return fanc_err_status;
}


void gamepad::Initialize(){
    for (int i = 0; i < NUM_of_ADC_12bit; i++)
    {
        //wip castが起こられているので後にreinterpret_castに変更すること
        memcpy(&(gamepadHID.axis[i].cal_center), (const void*)(0x08007c00 + 4*3*i), sizeof(uint16_t));
        memcpy(&(gamepadHID.axis[i].cal_pos_coeficient), (const void*)(0x08007c00 + 4*3*i + 4), sizeof(float));
        memcpy(&(gamepadHID.axis[i].cal_neg_coeficient), (const void*)(0x08007c00 + 4*3*i + 8), sizeof(float));
    }
    //assert_param(0);
}

HAL_StatusTypeDef gamepad::DMA_ADC_Start() {
    return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_val[0], NUM_of_ADC_12bit);
}
