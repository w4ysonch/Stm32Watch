/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#include "gpio.h"
#include "beep.h"
#include "oled.h"
#include "Data.h"

extern QueueHandle_t g_xQueueMenu;
//extern SemaphoreHandle_t g_xSemMenu; 

extern TaskHandle_t xShowMenuTaskHandle;
extern TaskHandle_t xShowTimeTaskHandle;
extern TaskHandle_t xShowWoodenFishTaskHandle;
extern TaskHandle_t xShowFlashLightTaskHandle;
extern TaskHandle_t xShowSettingTaskHandle;
extern TaskHandle_t xShowClockTaskHandle;
extern TaskHandle_t xShowCalendarTaskHandle;
extern TaskHandle_t xShowDHT11TaskHandle;
extern TaskHandle_t xShowBMP280TaskHandle;

const char str[6][10] = {"cleder", "torch", "hum", "clock", "press", "more"};

/* app's name */
str1 fly1 = {"fly1", NULL};
str1 dino1 = {"hum", NULL};
str1 test1 = {"torch", NULL};
str1 block1 = {"clock", NULL};
str1 setting1 = {"setting", NULL};

/* some data */
Image Left = {0, 0, 23, 10};
Image Right = {104, 0, 23, 10};
Image String = {53, 10, 0, 0};
Image Rec_select = {49, 16, 32, 32};
uint8_t dock_pos = 2;
uint8_t dock[6] = {45, 55, 65, 75, 85, 95};
uint8_t dock_y = 58, dock_r = 3;  
int str_flag = 2;
int8_t R_move_pos[6] = {-1, 9, 49, 89, 129, 169};
BaseType_t select = 3;

uint32_t end_flag = 1;
uint32_t seclect_end = 0;//showsetting

/* draw app's icon */
void ShowUI(void)
{
	/* show_gameui */
	OLED_ShowString(0, 0, "Menu", 16, 0);
	
	// Show arrows
	OLED_ShowString(0, 3, "<", 16, 0);
	OLED_ShowString(120, 3, ">", 16, 0);

	// Show current selection
	OLED_ShowString(30, 3, (char*)str[str_flag], 16, 0);
};

void ShowMenuTask(void *params)
{
//	xSemaphoreTake(g_xQueueMenu, portMAX_DELAY);
	
	/* system sound */
	buzzer_init();
   
	/* create queue */
	//g_xQueueMenu = xQueueCreate(4, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	
	
	OLED_Init();
	OLED_Clear();
	OLED_Display_On();
	
	struct Key_data	key_data;
	
	while(1)
	{
		OLED_Clear();
		ShowUI();
		
		/* receive queue data and keep waitting */
		xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
		
		/* handle data */
		if(key_data.rdata == 1)
		{	
			if(dock_pos != 0)
			{
				dock_pos--;
				str_flag--;
				buzzer_buzz(2000, 100);
			}
			key_data.rdata = 0;
		}
		else if(key_data.ldata == 1)
		{
			if(dock_pos < 5)
			{		
				dock_pos++;
				str_flag++;
				buzzer_buzz(2000, 100);
			}
			key_data.ldata = 0;
		}
		/* ststus machine : task scheduling  */
		else if(key_data.exdata == 1)
		{
			buzzer_buzz(2000, 100);
			switch(dock_pos)
			{
				case 0: vTaskResume(xShowCalendarTaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
				case 1: vTaskResume(xShowFlashLightTaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
				case 2: vTaskResume(xShowDHT11TaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
				case 3: vTaskResume(xShowClockTaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
				case 4: vTaskResume(xShowBMP280TaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
				case 5: vTaskResume(xShowSettingTaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
			}
		}
		else if(key_data.updata == 1)
		{
			/* SysSound */
			buzzer_buzz(2000, 100);
			vTaskResume(xShowTimeTaskHandle);
			vTaskSuspend(NULL);
			key_data.updata = 0;
		}
	}
}

