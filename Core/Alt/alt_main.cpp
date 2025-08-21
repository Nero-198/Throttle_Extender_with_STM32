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
#define CUSTOM_HID_EPIN_SIZE 7
#endif

/*declaration Grobal Valiable*/
//ADC_HandleTypeDef hadc1;
//UART_HandleTypeDef huart1;
ADC_ChannelConfTypeDef sConfig;

/*Instance*/
gamepad gamepad;
/*declarattion extern Valiable*/
extern USBD_HandleTypeDef hUsbDeviceFS;


int alt_main(){
	/*Variable*/
	setbuf(stdout, NULL);	//printf用

	/* initialize ここで初期化処理等を書く  */
    for (uint8_t i = 0; i < BUTTONS_DATA_BUFFER_SIZE; i++)
    {
        gamepad.gamepadHID.buttons[i] = 0;
	}
	for (uint8_t i = 0; i < (NUM_of_ADC_12bit * 2); i++)
    {
		gamepad.gamepadHID.axis[i] = 0;
    }

      if (HAL_ADCEx_Calibration_Start(&hadc1) !=  HAL_OK)	//ADCを自動でキャリぶれーとしてくれる？
  {
    Error_Handler();
  }
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
      gamepad.Initialize();

	while(1){
		/*alt_main loop ここにメイン関数のループを書く。*/
    /*gamepadHIDデバッグ出力*/
    /*debug config  ---End point data
    printf("\033[2J");
    printf("\033[10A");
    printf("Value: %x\r\n", gamepad.gamepadHID.buttons[0]);
    printf("Value: %x\r\n", gamepad.gamepadHID.buttons[1]);
    printf("Value: %x\r\n", gamepad.gamepadHID.buttons[2]);
    printf("Value: %d\r\n", gamepad.gamepadHID.axis[0]);
    printf("Value: %d\r\n", gamepad.gamepadHID.axis[1]);
    printf("Value: %d\r\n", gamepad.gamepadHID.axis[2]);
    printf("Value: %d\r\n", gamepad.gamepadHID.axis[3]);
    */
   //キャリブレーション開始操作。ボタンを押すとキャリブレーションが始まる。
    if (HAL_GPIO_ReadPin(TDC_Calibrate_GPIO_Port,TDC_Calibrate_Pin) == GPIO_PIN_RESET)
    {
    	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
    	printf("Calibrate Start\r\n");
        gamepad.ADCcalibrate();
        gamepad.Initialize();
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    }
    /*dataゲット*/
    gamepad.readButtons();
    //printf("Read Buttons\r\n");
    gamepad.readAxis();
    //printf("Read Axis\r\n");
		/*USB送信*/
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &gamepad.gamepadHID.buttons[0], CUSTOM_HID_EPIN_SIZE);  //buttonsとaxisを別々に送信して良いのかは怪しい。
    //printf("Send USB\r\n");
	}
}

/* func */
/*
 * //printf用 printfの内部でこの処理が要る。参考:https://yukblog.net/stm32cubeide-printf-uart/
 *
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
    return len;
}

