/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beep.h"
#include "u8g2.h"
#include "Data.h"
/* USER CODE END Includes */

extern SemaphoreHandle_t g_xSemKey; 
extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;
extern u8g2_t u8g2;

uint16_t WfisTaskRuning;

void ShowWoodenFishTask(void *params)
{
	//xSemaphoreTake(g_xSemKey, portMAX_DELAY);
	
	/* system sound */
	buzzer_init();
	/* 创建队列 */
	g_xQueueMenu = xQueueCreate(4, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	/* u8g2 Start */
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2,U8G2_R0, u8x8_byte_hw_i2c, u8g2_stm32_delay);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_ClearDisplay(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_wqy16_t_chinese1);
//	u8g2_SetFont(&u8g2, u8g2_font_spleen32x64_mf);	
//	u8g2_SetFont(&u8g2, u8g2_font_fur35_tf);
	
	struct Key_data	key_data;
	uint8_t wooden_fish_flag = 0, seclect_flag = 0, add_flag = 0;
	uint8_t num1 = 0, num2 = 0, num3 = 0; 
	while(1)
	{
		WfisTaskRuning = 1;
		u8g2_ClearBuffer(&u8g2);

		u8g2_DrawXBMP(&u8g2, 80, 0, 6, 8, Num_6x8[num1]);		
		u8g2_DrawXBMP(&u8g2, 88, 0, 6, 8, Num_6x8[num2]);		
		u8g2_DrawXBMP(&u8g2, 96, 0, 6, 8, Num_6x8[num3]);		

		switch(seclect_flag)
		{
			case 0: u8g2_DrawXBMP(&u8g2, 8, 15, 63, 48, wooden_flsh[0]);u8g2_DrawXBMP(&u8g2, 5, 0, 30, 8, hammer);break;
            case 1: u8g2_DrawXBMP(&u8g2, 8, 15, 49, 38, wooden_flsh[1]);u8g2_DrawXBMP(&u8g2, 10, 8, 30, 8, hammer);seclect_flag = 0;break;
		}
		u8g2_DrawXBMP(&u8g2, 75, 37, 32, 16, gongde);
		u8g2_DrawStr(&u8g2, 110, 47, ":");		
		
		u8g2_SendBuffer(&u8g2);
		
		/* 读按键中断队列 */
		if(wooden_fish_flag == 0 && seclect_flag == 0)
		{
			pdPASS == xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
		}
		/* processing data */
		if(key_data.rdata == 1)
		{
			buzzer_buzz(3000, 200);
			num3++;
			if(num3>9){num3=0;num2++;}
			if(num2>9){num2=0;num1++;}
			seclect_flag = 1;
			add_flag = 100;
			key_data.rdata = 0;
			key_data.exdata = 0;
		}		
		if(key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
		}
		
		u8g2_ClearBuffer(&u8g2);
		if(add_flag!=0){add_flag--;u8g2_DrawStr(&u8g2, 105, 40, "+1");}
		switch(seclect_flag)
		{
			case 0: u8g2_DrawXBMP(&u8g2, 8, 15, 63, 48, wooden_flsh[0]);u8g2_DrawXBMP(&u8g2, 5, 0, 30, 8, hammer);break;
            case 1: u8g2_DrawXBMP(&u8g2, 8, 15, 49, 38, wooden_flsh[1]);u8g2_DrawXBMP(&u8g2, 10, 8, 30, 8, hammer);seclect_flag = 0;break;
		}			
		u8g2_SendBuffer(&u8g2);
	}
}
