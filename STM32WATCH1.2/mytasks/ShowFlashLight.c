/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beep.h"
#include "oled.h"
#include "Data.h"
/* USER CODE END Includes */

extern QueueHandle_t g_xQueueMenu;
extern TaskHandle_t xShowMenuTaskHandle;

void ShowFlashLightTask(void *params)
{
	buzzer_init();
	/* �������� */
	//g_xQueueMenu = xQueueCreate(1, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	OLED_Init();
	OLED_Clear();
	
	OLED_ShowString(20, 3, "Light: OFF", 16, 0);

	uint8_t light_flag = 0;
	struct Key_data	key_data;
	
	while(1)
	{
		// OLED_Clear(); // Don't clear if we want to keep the light on in case 0
		/* �������ж϶��� */
		xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
		
		if(key_data.updata == 1)
		{
			buzzer_buzz(2500, 100);
			switch(light_flag)
			{
				case 0: 
					OLED_On(); // Full bright
					light_flag++;
					break;
				case 1: 
					OLED_Clear(); // Clear screen (turn off light)
					OLED_ShowString(20, 3, "Light: OFF", 16, 0);
					light_flag--;
					break;
			}
		}		
		if(key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);
			OLED_Clear(); // Ensure screen is cleared before leaving
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
                        light_flag = 0;
                        OLED_Clear();
                        OLED_ShowString(20, 3, "Light: OFF", 16, 0);
                        key_data.exdata = 0;
		}		
	}
}

