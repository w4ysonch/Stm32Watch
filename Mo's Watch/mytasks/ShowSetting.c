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
#include "u8g2.h"
#include "Data.h"

/* some extern data */
extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;
extern u8g2_t u8g2;
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
		u8g2_DrawStr(&u8g2, str_x_pos, str_y_pos[i], strs[i]);
		width[i] = u8g2_GetStrWidth(&u8g2, &strs[i][10]);
//		u8g2_GetMaxCharHeight();		
	}
	u8g2_DrawFrame(&u8g2, 0, seclect_y[0], seclect_w[0], seclect_h);		
	u8g2_DrawBox(&u8g2, 48, seclect_y[0], 5, 12);
	u8g2_DrawFrame(&u8g2, 48, 0, 5, 63);	
}
void ShowAbout(void)
{
	u8g2_DrawStr(&u8g2, about_x_pos, about_y_pos[0],  "thank you");
	u8g2_DrawStr(&u8g2, about_x_pos, about_y_pos[1],  "following");
	u8g2_DrawStr(&u8g2, about_x_pos, about_y_pos[2],  "my project");
	u8g2_DrawStr(&u8g2, about_x_pos, about_y_pos[3],  "@moyiji");
}

void ShowSwitch(int switch_status)
{
	u8g2_DrawHLine(&u8g2, 70, 22, 40);
	u8g2_DrawHLine(&u8g2, 70, 40, 40);
	if(switch_status == 0)
	{
		u8g2_DrawDisc(&u8g2, 70, 31, 9, U8G2_DRAW_ALL);
		u8g2_DrawStr(&u8g2, 100, 34,  "on");  
		u8g2_DrawCircle(&u8g2, 110, 31, 9, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_LOWER_RIGHT);		
	}
	if(switch_status == 1)
	{
		u8g2_DrawDisc(&u8g2, 110, 31, 9, U8G2_DRAW_ALL);
		u8g2_DrawStr(&u8g2, 69, 36,  "off"); 
		u8g2_DrawCircle(&u8g2, 70, 31, 9, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_LOWER_LEFT);
	}
}

void ShowSetting_Task(void)
{
	buzzer_init();
	/* 创建队列 */
	g_xQueueMenu = xQueueCreate(4, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	u8g2_config();
	u8g2_SetFont(&u8g2, u8g2_font_7x13_mf);	
    u8g2_FirstPage(&u8g2);
	do {
	ShowSetiing();
	u8g2_SendBuffer(&u8g2);
   	} while (u8g2_NextPage(&u8g2));
	for(int i = 0; i<5; i++)
	{
		width[i] = u8g2_GetStrWidth(&u8g2, &strs[i][10]);
	}
	struct Key_data	key_data;

	while(1)
	{
		u8g2_ClearBuffer(&u8g2);
		 		
		switch(seclect)
		{
			case 0: u8g2_DrawStr(&u8g2, 64, 25,"@moyiji");u8g2_DrawStr(&u8g2, 61, 50,"2021/05/27");break;
			case 1: ShowSwitch(power_button);break;
			case 2: ShowSwitch(power_button);break;
			case 3: ShowSwitch(power_button);break;
			case 4: ShowAbout();break;
		}
		ShowSetiing();
		u8g2_SendBuffer(&u8g2);
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
