/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beep.h"
#include "driver_dht11.h"
#include "oled.h"
#include "Data.h"
#include "ShowCalendar.h"

/* USER CODE END Includes */

extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;

void ShowDHT11Task(void *params)
{
	DHT11_Init();
	buzzer_init();
	/* 创建队列 */
	//g_xQueueMenu = xQueueCreate(1, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	OLED_Init();
	OLED_Clear();
	
	struct Key_data	key_data = {0};
	int hum, temp;
    int hum1, hum2, hum3, temp1, temp2 ,temp3;

    int max;
	int g_max[] = {20, 30, 40, 50, 60, 70};
	
	while(1)
	{
		OLED_Clear();
		if (DHT11_Read(&hum, &temp) !=0 ){
			DHT11_Init();
			OLED_ShowString(10, 2, "Temp:", 16, 0);
			OLED_ShowString(10, 4, "Humi:", 16, 0);
		}
		else{
			temp1 = temp%10;	//low bit
			temp2 = temp/10;    //high bit
			
			hum1 = hum%10;		//low bit	
			hum2 = hum/10;      //high bit
			
			OLED_ShowString(10, 2, "Temp:", 16, 0);
			OLED_ShowNum(50, 2, temp2, 1, 16, 0);
			OLED_ShowNum(58, 2, temp1, 1, 16, 0);
			
			OLED_ShowString(10, 4, "Humi:", 16, 0);
			OLED_ShowNum(50, 4, hum2, 1, 16, 0);
			OLED_ShowNum(58, 4, hum1, 1, 16, 0);
		}
		
		/* 读按键中断队列，阻塞等待1s，既实现了延时，又实现了按键响应 */
		if(xQueueReceive(g_xQueueMenu, &key_data, 1000) == pdPASS)
		{
			if(key_data.exdata == 1)
			{
				buzzer_buzz(2500, 100);
				vTaskResume(xShowMenuTaskHandle);
				vTaskSuspend(NULL);
				key_data.exdata = 0;
			}
		}
	}
}
