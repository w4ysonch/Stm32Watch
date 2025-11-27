/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
//#include "semphr.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beep.h"
#include "oled.h"
#include "Data.h"

/* some extern data */
extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;
extern BaseType_t end_flag;
extern BaseType_t seclect_end;

/* some data */
const char strs[5][10] = {"<<<", "record", "Sound", "Power", "About"}; 
BaseType_t str_x_pos = 1, str_y_pos[] = {11, 23, 36, 49, 62};
BaseType_t about_x_pos = 55, about_y_pos[] = {13, 27, 41, 55};

int32_t seclect = 0, seclect_h = 13;
int32_t seclect_y[6] = {0, 13, 25, 38, 51, 0};
int32_t seclect_w[6] = {24, 44, 37, 37, 37, 24}; 

int width[5] = {0};

/* control system sound */
int power_button = 0;
	
void ShowSetiing(void)
{
	for(int i = 0; i<5; i++)
	{
		OLED_ShowString(str_x_pos, str_y_pos[i]/8, (char*)strs[i], 16, 0);
	}
}
void ShowAbout(void)
{
	OLED_ShowString(about_x_pos, about_y_pos[0]/8,  "thank you", 16, 0);
	OLED_ShowString(about_x_pos, about_y_pos[1]/8,  "following", 16, 0);
	OLED_ShowString(about_x_pos, about_y_pos[2]/8,  "my project", 16, 0);
	OLED_ShowString(about_x_pos, about_y_pos[3]/8,  "@chs&mzk", 16, 0);
}

void ShowSwitch(int switch_status)
{
	if(switch_status == 0)
	{
		OLED_ShowString(100, 34/8,  "on", 16, 0);  
	}
	if(switch_status == 1)
	{
		OLED_ShowString(69, 36/8,  "off", 16, 0); 
	}
}

void ShowSetting_Task(void)
{
	buzzer_init();
	/* 创建队列 */
	//g_xQueueMenu = xQueueCreate(4, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	OLED_Init();
	OLED_Clear();
	OLED_Display_On();

	struct Key_data	key_data;

	while(1)
	{
		OLED_Clear();
		 		
		switch(seclect)
		{
			case 0: OLED_ShowString(64, 25/8,"@chs&mzk", 16, 0);OLED_ShowString(61, 50/8,"2025/11", 16, 0);break;
			case 1: ShowSwitch(power_button);break;
			case 2: ShowSwitch(power_button);break;
			case 3: ShowSwitch(power_button);break;
			case 4: ShowAbout();break;
		}
		ShowSetiing();
		
		if(seclect_end == 0)
		{
			pdPASS == xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
        }
		/* move_down */
		if(key_data.rdata == 1)
		{
			seclect_end++;
			if(seclect!=4)
			{
				/* status_machine */
				switch(seclect)
				{
					case 0: ui_run(&seclect_w[0], &seclect_w[1], 1);ui_run(&seclect_y[0], &seclect_y[1], 1);break;
					case 1: ui_run(&seclect_w[0], &seclect_w[2], 1);ui_run(&seclect_y[0], &seclect_y[2], 1);break;
					case 2: ui_run(&seclect_w[0], &seclect_w[3], 1);ui_run(&seclect_y[0], &seclect_y[3], 1);break;
					case 3: ui_run(&seclect_w[0], &seclect_w[4], 1);ui_run(&seclect_y[0], &seclect_y[4], 1);break;
					case 4: ui_run(&seclect_w[0], &seclect_w[5], 1);ui_run(&seclect_y[0], &seclect_y[5], 1);break;				
				}
			}
			if(seclect_end == 20)
			{
				if(seclect!=4)seclect++;
				seclect_end = 0;
				key_data.rdata = 0;
			}
			if(seclect_end == 0)buzzer_buzz(2500, 100);			                     
		}
		/* move_up */
		else if(key_data.ldata == 1)
		{
			seclect_end++;
			if(seclect!=0)
			{
				switch(seclect)
				{
					case 0: break;
					case 1: ui_run(&seclect_w[0], &seclect_w[5], 1);ui_run(&seclect_y[0], &seclect_y[5], 1);break;
					case 2: ui_run(&seclect_w[0], &seclect_w[1], 1);ui_run(&seclect_y[0], &seclect_y[1], 1);break;
					case 3: ui_run(&seclect_w[0], &seclect_w[2], 1);ui_run(&seclect_y[0], &seclect_y[2], 1);break;
					case 4: ui_run(&seclect_w[0], &seclect_w[3], 1);ui_run(&seclect_y[0], &seclect_y[3], 1);break;				
				}				
			}
			if(seclect_end == 20)
			{
				if(seclect!=0)seclect--;
				seclect_end = 0;
				key_data.ldata = 0;
			}
			if(seclect_end == 0)buzzer_buzz(2500, 100);			                     			
		}
		if(key_data.updata == 1)
		{
			buzzer_buzz(2500, 100);			                     
			power_button++;
			power_button = power_button%2;
		}		
		/* task scheduling */
		if(key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);			                     
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
		}
	}
}
