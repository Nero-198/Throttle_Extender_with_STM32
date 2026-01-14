/*
 * alt_main.cpp
 *
 *  Created on: Oct 17, 2023
 *      Author: iroen
 */
/*このファイルはcppをCubeMXによるコード生成に対応させるためのコードである。
 * CubeMXはmain.cファイルの生成にしか対応していないため、main.cppを作って運用してしていても途中でペリフェラルの変更等が行いずらい。
 * main.cppに名前を変えたままコード生成を行うと、main.cppの他に新たなmain.cが生まれてしまう。
 * これを避けるため、cppで記述する部分を本ファイルで実装し、main.cでは本ファイルのコードを読み出す形とする。
 * 参照:https://kleinembedded.com/cpp-and-stm32cubemx-code-generation/
 */
/*設定の類はgamepad.hに記述している。*/

/*main.c と同じIncludeを行う。*/
#include "main.h"
#include "adc.h"
#include "usb_device.h"
#include "gpio.h"
#include "usart.h"

/*include*/
#include "alt_main.h"
#include <stdio.h>
#include <usbd_customhid.h>
#include "usb_device.h"
#include "gamepad.h"

#ifdef CUSTOM_HID_EPIN_SIZE
#undef CUSTOM_HID_EPIN_SIZE
#define CUSTOM_HID_EPIN_SIZE 11 //byte
#endif

//#define DEBUG  //debug用のprintfを有効にするならコメントアウトを外す
//#define USB_SEND_DUMMY_REPORT  // USB疎通確認用のダミーデータ送信を有効にするならコメントアウトを外す
#define DEBUG_USB_REPORT 1  // USB送信データをUARTに出す

/*declaration Grobal Valiable*/
//ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig;

/*Instance*/
gamepad g_gamepad;
/*declarattion extern Valiable*/
extern USBD_HandleTypeDef hUsbDeviceFS;

#ifdef USB_SEND_DUMMY_REPORT
// USBが正しく動作しているかを確認するためのダミーレポート送信
static inline void WriteLe16(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void SendDummyHidReport(void)
{
    static uint8_t dummyReport[CUSTOM_HID_EPIN_SIZE] = {0};
    static uint16_t val01 = 0;  // [0],[1] : 0 〜 0x1FFF を1ずつインクリメント
    static uint16_t val23 = 0;  // [2],[3] : 100ずつ増加
    static uint16_t val45 = 0;  // [4],[5] : 100ずつ増加
    static uint16_t val67 = 0;  // [6],[7] : 100ずつ増加
    static uint16_t val89 = 0;  // [8],[9] : 100ずつ増加

    WriteLe16(&dummyReport[0], val01);
    WriteLe16(&dummyReport[2], val23);
    WriteLe16(&dummyReport[4], val45);
    WriteLe16(&dummyReport[6], val67);
    WriteLe16(&dummyReport[8], val89);

    // 次回送信用に値を更新
    val01 = static_cast<uint16_t>((val01 + 1U) % 0x2000U);  // 0〜0x1FFF の範囲で循環
    val23 = static_cast<uint16_t>(val23 + 100U);
    val45 = static_cast<uint16_t>(val45 + 100U);
    val67 = static_cast<uint16_t>(val67 + 100U);
    val89 = static_cast<uint16_t>(val89 + 100U);

    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, dummyReport, CUSTOM_HID_EPIN_SIZE);
}
#endif

static void DebugPrintHidReport(const uint8_t *data, uint16_t len)
{
#if DEBUG_USB_REPORT
    printf("USB HID Report (%u bytes):", len);
    for (uint16_t i = 0; i < len; i++)
    {
        printf(" %02X", data[i]);
    }
    printf("\r\n");
#else
    (void)data;
    (void)len;
#endif
}


int alt_main(){
	/*Variable*/
	setbuf(stdout, NULL);	//printf用

	/* initialize ここで初期化処理等を書く  */
    for (uint8_t i = 0; i < BUTTONS_DATA_BUFFER_SIZE; i++)
    {
        g_gamepad.gamepadHID.buttons[i] = 0;
	}
	for (uint8_t i = 0; i < (NUM_of_ADC_12bit); i++)
    {
		g_gamepad.gamepadHID.axis[i].value = 0;
    }

    // F1系はキャリブレーションを一度しておく
    if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        // キャリブレーションエラー処理
        printf("ADC Calibration Error\r\n");
        //assert_param(0); //define USE_FULL_ASSERTが有効な場合にエラーを報告
    }
    if(g_gamepad.DMA_ADC_Start() != HAL_OK)
    {
        printf("ADC DMA Start Error\r\n");
    }

    g_gamepad.Initialize();

	while(1){
#ifdef USB_SEND_DUMMY_REPORT
        SendDummyHidReport();
        HAL_Delay(10);
        continue;  // ダミー送信モード中は通常処理をスキップ
#endif
		/*alt_main loop ここにメイン関数のループを書く。*/
    /*gamepadHIDデバッグ出力*/
    /*debug config  ---End point data
    printf("\033[2J");
    printf("\033[10A");
    printf("Value: %x\r\n", g_gamepad.gamepadHID.buttons[0]);


    printf("Value: %x\r\n", g_gamepad.gamepadHID.buttons[1]);
    printf("Value: %x\r\n", g_gamepad.gamepadHID.buttons[2]);
    printf("Value: %d\r\n", g_gamepad.gamepadHID.axis[0]);
    printf("Value: %d\r\n", g_gamepad.gamepadHID.axis[1]);
    printf("Value: %d\r\n", g_gamepad.gamepadHID.axis[2]);
    printf("Value: %d\r\n", g_gamepad.gamepadHID.axis[3]);
    */
   //キャリブレーション開始操作。ボタンを押すとキャリブレーションが始まる。
    if (HAL_GPIO_ReadPin(TDC_CALBRATION_GPIO_Port, TDC_CALBRATION_Pin) == GPIO_PIN_RESET)
    {
    	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
    	printf("Calibrate Start\r\n");
        g_gamepad.ADCcalibrate();
        g_gamepad.Initialize();
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    }
    /*dataゲット*/
    g_gamepad.readButtons();
    //printf("Read Buttons\r\n");
    g_gamepad.readAxis();
    //printf("Read Axis\r\n");
    g_gamepad.Axis_to_TDC_buttons();
        /*USB送信*/
        UART_puts("Sending USB HID Report\r\n");
        DebugPrintHidReport(&g_gamepad.USB_HID_Report.buttons[0], CUSTOM_HID_EPIN_SIZE);
            if(USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &g_gamepad.USB_HID_Report.buttons[0], CUSTOM_HID_EPIN_SIZE) != USBD_OK){
                // BUSY/FAILの時は次フレームで再送。過度な連続送信でハングするのを防ぐ。
            }
        HAL_Delay(1); // 1ms程度待ち、USBフレーム間隔に合わせて送信を間引く
	}
}

/* func */
/*
 * //printf用 printfの内部でこの処理が要る。参考:https://yukblog.net/stm32cubeide-printf-uart/
 *今回はUSBデバッグ用
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
    return len;
}

