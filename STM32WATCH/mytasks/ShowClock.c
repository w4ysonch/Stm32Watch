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
#include "u8g2.h"
#include "Data.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

extern TaskHandle_t xShowMenuTaskHandle;
extern TimerHandle_t g_Clock_Timer;
extern QueueHandle_t g_xQueueMenu;
extern u8g2_t u8g2;

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
	len1 = u8g2_DrawStr(&u8g2, 0, 30, "Set:");
	len2 = u8g2_DrawStr(&u8g2, 0, 60, "Ret:");

	/* draw_time */
	for(int i = 0; i < 4; i++)
	{
		u8g2_DrawXBMP(&u8g2, len1+g_num_x[i], g_num_y[0], num_w, num_h, Num_6x8[g_clock_num[i]]);	
		u8g2_DrawXBMP(&u8g2, len2+g_num_x[i], g_num_y[1], num_w, num_h, Num_6x8[g_real_time[i]]);
	}
	u8g2_DrawXBMP(&u8g2, len1+g_num_x[5], g_num_y[0], num_w, num_h, Num_6x8[10]);/* : */
	u8g2_DrawXBMP(&u8g2, len1+g_num_x[4], g_num_y[0], num_w, num_h, Num_6x8[11]);/* > */
	u8g2_DrawXBMP(&u8g2, len2+g_num_x[5], g_num_y[1], num_w, num_h, Num_6x8[10]);/* : */

	/* draw_clock */
	u8g2_DrawCircle(&u8g2, 104, 31, 22, U8G2_DRAW_ALL);
	u8g2_DrawCircle(&u8g2, 104, 31, 23, U8G2_DRAW_ALL);
	u8g2_DrawDisc(&u8g2, 104, 31, 1, U8G2_DRAW_ALL);
	
	u8g2_DrawXBMP(&u8g2, 94, 12, 20, 40, BigNum[millisecond]);			
	/* seclect */
	u8g2_DrawFrame(&u8g2, len1+g_seclect_x[seclect_flag]-1, g_num_y[0]-2, num_w+2, num_h+3);	
}
void ShowClockTimeTask(void *params)
{
	/* system sound */
	buzzer_init();

	/* 创建队列 */
	g_xQueueMenu = xQueueCreate(1, 4);
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
	while(1)
	{
		u8g2_ClearBuffer(&u8g2);
		ShowClock();		
		//u8g2_DrawXBMP(&u8g2, 0, 0, 20, 40, BigNum[temp]);
		u8g2_SendBuffer(&u8g2);
		
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
			xTimerStop(g_Clock_Timer, 0);
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
				xTimerStop(g_Clock_Timer, 0);
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
