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
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

extern TaskHandle_t xShowMenuTaskHandle;
extern TimerHandle_t g_Clock_Timer;
extern QueueHandle_t g_xQueueMenu;

uint16_t millisecond;
uint8_t len1, len2;
uint8_t clock_flag = 0;
int8_t seclect_flag = 0;
uint16_t g_num_x[] = {1, 9, 25, 33, 41, 17}, g_num_y[] = {22, 52}, num_w = 6, num_h = 8;
uint16_t g_seclect_x[] = {1, 9, 25, 33, 41};
uint16_t g_clock_num[] = {0, 0, 0, 0};
uint16_t g_real_time[] = {0, 0, 0, 0};

void ShowClock(void)
{
	OLED_ShowString(0, 2, "Set:", 16, 0);
	OLED_ShowString(0, 6, "Ret:", 16, 0);

	/* draw_time */
	// Set time
	OLED_ShowNum(40, 2, g_clock_num[0], 1, 16, 0);
	OLED_ShowNum(48, 2, g_clock_num[1], 1, 16, 0);
	OLED_ShowString(56, 2, ":", 16, 0);
	OLED_ShowNum(64, 2, g_clock_num[2], 1, 16, 0);
	OLED_ShowNum(72, 2, g_clock_num[3], 1, 16, 0);

	// Real time
	OLED_ShowNum(40, 6, g_real_time[0], 1, 16, 0);
	OLED_ShowNum(48, 6, g_real_time[1], 1, 16, 0);
	OLED_ShowString(56, 6, ":", 16, 0);
	OLED_ShowNum(64, 6, g_real_time[2], 1, 16, 0);
	OLED_ShowNum(72, 6, g_real_time[3], 1, 16, 0);

	/* draw_clock */
	// Millisecond
	OLED_ShowNum(94, 4, millisecond, 1, 16, 0);
}
void ShowClockTimeTask(void *params)
{
	/* system sound */
	buzzer_init();

	/* 创建队列 */
	//g_xQueueMenu = xQueueCreate(1, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	OLED_Init();
	OLED_Clear();
	
	struct Key_data	key_data;	
	while(1)
	{
		OLED_Clear();
		ShowClock();		
		
		/* 读按键中断队列 */
		if(clock_flag == 0)
		{
			pdPASS == xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
		}
		/* seclect */
		if(key_data.rdata == 1)
		{
			buzzer_buzz(2500, 100);
			seclect_flag++;
			if(seclect_flag>4)seclect_flag=0;
		}
		if(key_data.ldata == 1)
		{
			buzzer_buzz(2500, 100);
			seclect_flag--;
			if(seclect_flag<0)seclect_flag=4;
		}
		/* handle_data */
		if(key_data.updata == 1)
		{
			buzzer_buzz(2500, 100);
			if(seclect_flag == 4)
			{
				/*启动定时器*/
				if(g_Clock_Timer != NULL)
				{
					xTimerStart(g_Clock_Timer, 0);
					clock_flag = 1;
					key_data.updata = 0;
				}
			}
			else{
				g_clock_num[seclect_flag]++;
				if(g_clock_num[seclect_flag]>9)g_clock_num[seclect_flag]=0;				
			}
		}		
		if(key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);
			clock_flag = 0;
			if(g_Clock_Timer != NULL)
			{
				xTimerStop(g_Clock_Timer, 0);
			}
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
		}	
		/* circle_run */
		if(clock_flag == 1)
		{
			/* time_stop */
			if(g_clock_num[0]==g_real_time[0]&&g_clock_num[1]==g_real_time[1]&&g_clock_num[2]==g_real_time[2]&&g_clock_num[3]==g_real_time[3])
			{
				clock_flag = 0;
				if(g_Clock_Timer != NULL)
				{
					xTimerStop(g_Clock_Timer, 0);
				}
				/* music */
				buzzer_buzz(2500, 1000);
			}
		}
	}
}

/******************TimerCallBackFun*******************/
void ClockTimerCallBackFun(TimerHandle_t xTimer)
{  
	millisecond++;
	if(millisecond>9)
	{
		millisecond = 0;		
		g_real_time[3]++;
		if(g_real_time[3]>9)
		{
			g_real_time[2]++;
			g_real_time[3]=0;
			if(g_real_time[2]>5)
			{
				g_real_time[1]++;
				g_real_time[2] = 0;
				if(g_real_time[1]>9)
				{
					g_real_time[0]++;
					g_real_time[1] = 0;
				}
			}
		}	
	}
}
