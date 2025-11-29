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
#include "oled.h"
#include "Data.h"
/* USER CODE END Includes */

/* other task handle */
extern QueueHandle_t g_xQueueMenu;
extern TaskHandle_t xShowMenuTaskHandle;
extern TaskHandle_t xShowTimeTaskHandle;
extern TaskHandle_t xShowWoodenFishTaskHandle;
extern TaskHandle_t xShowFlashLightTaskHandle;
extern TaskHandle_t xShowSettingTaskHandle;
extern TaskHandle_t xShowCalendarTaskHandle;
extern TaskHandle_t xShowClockTaskHandle;
extern TaskHandle_t xShowDHT11TaskHandle;
extern TaskHandle_t xShowBMP280TaskHandle;
extern TaskHandle_t xShowStepTaskHandle;

/* some data */
#define BOX_R 1
uint8_t time_flag = 0;

uint8_t sec_unit, sec_decade, min_unit, min_decade, hour_unit, hour_decade;
typedef struct Time_param{
    int x[4];
	int y;
	int w;
	int h;
	int x_arg;
}T;
T time = { {8, 35, 71, 98}, 15, 20, 40, 98};
Image Box1 = {62, 22, 4, 4,};
Image Box2 = {62, 39, 4, 4,};

void ShowTimeTask(void *params)
{
	buzzer_init();
	//xSemaphoreTake(g_xSemTicks, portMAX_DELAY);

	/* suspend_other_task */
	vTaskSuspend(xShowMenuTaskHandle);
	//vTaskSuspend(xShowWoodenFishTaskHandle);
	vTaskSuspend(xShowFlashLightTaskHandle);
	vTaskSuspend(xShowSettingTaskHandle);
    vTaskSuspend(xShowClockTaskHandle);
    vTaskSuspend(xShowCalendarTaskHandle);
    vTaskSuspend(xShowDHT11TaskHandle);
	vTaskSuspend(xShowBMP280TaskHandle);
	vTaskSuspend(xShowStepTaskHandle);

	/* create_queue */
	//g_xQueueMenu = xQueueCreate(1, 4);
	
	OLED_Init();
	OLED_Clear();
	
	/* receive queue */
	struct Key_data	key_data;

	while(1)
	{	
		OLED_Clear();
		/* draw */
		/* draw time */
		OLED_ShowString(30, 1, "Time", 16, 0);

		OLED_ShowNum(20, 3, hour_decade, 1, 16, 0);
		OLED_ShowNum(28, 3, hour_unit, 1, 16, 0);
		OLED_ShowString(36, 3, ":", 16, 0);
		OLED_ShowNum(44, 3, min_decade, 1, 16, 0);
		OLED_ShowNum(52, 3, min_unit, 1, 16, 0);
		OLED_ShowString(60, 3, ":", 16, 0);
		OLED_ShowNum(68, 3, sec_decade, 1, 16, 0);
		OLED_ShowNum(76, 3, sec_unit, 1, 16, 0);
				
		vTaskDelay(250);
		/* handle queue data */
		if(time_flag == 0)
		{
			pdPASS == xQueueReceive(g_xQueueMenu, &key_data, 0);
		}
		/* task scheduling */
		if(key_data.updata == 1)
		{	
			buzzer_buzz(2500, 100);			                     
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
			key_data.updata = 0;
		}			
	}
}

/******************TimerCallBackFun*******************/
void TimerCallBackFun(TimerHandle_t xTimer)
{  
	/* handle time data */
	sec_unit++;		
	if(sec_unit>9){sec_unit = 0; sec_decade++;}
	if(sec_decade>5){sec_decade = 0; min_unit++;}
	if(min_unit>9){min_unit = 0; min_decade++;}
	if(min_decade>5){min_decade = 0; hour_unit++;}
	if(hour_unit>5){hour_unit = 0; hour_decade++;}
	
}
