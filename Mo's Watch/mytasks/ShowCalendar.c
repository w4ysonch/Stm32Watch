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
#include "u8g2.h"
#include "Data.h"
#include "ShowCalendar.h"

/* USER CODE END Includes */

extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;

/* 判断闰年 */
int judge_year(int year)
{
	if(year%400==0||(year%4==0&&year%100!=0)){
		return 1;
	}else{
		return 0;
	}
}

/* 判断需要输出的年份的一月一日是星期几 */
int judge_week(int year)
{
	if(year==1){
		return 1;
	}
	
	int sum=0;
	for(int i=1;i<year;i++){
		if(judge_year(i)==1){
			sum=sum+366;
		} else{
			sum=sum+365;
		}
	}
	return (sum+1)%7;
}

/* 闰年各个月份的天数 */
int month_run(int n){
	switch(n){
		case 1:return 31;
		case 2:return 29;
		case 3:return 31;
		case 4:return 30;
		case 5:return 31;
		case 6:return 30;
		case 7:return 31;      
		case 8:return 31;
		case 9:return 30;
		case 10:return 31;
		case 11:return 30;
		case 12:return 31;
	}
}

int month_ping(int n){
	switch(n){
		case 1:return 31;
		case 2:return 28;
		case 3:return 31;
		case 4:return 30;
		case 5:return 31;
		case 6:return 30;
		case 7:return 31;
		case 8:return 31;
		case 9:return 30;
		case 10:return 31;
		case 11:return 30;
		case 12:return 31;
	}
}

void ShowCalendarTask(void *params)
{
	buzzer_init();
	/* 创建队列 */
	g_xQueueMenu = xQueueCreate(1, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2,U8G2_R0, u8x8_byte_hw_i2c, u8g2_stm32_delay);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_ClearDisplay(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_spleen5x8_mf);

	u8g2_SendBuffer(&u8g2);
	
	struct Key_data	key_data;

	const char ucMonthDay[32][3] = {"0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17","18","19","20","21","22","23","24","25","26","27","28","29","30","31"};	
	const char ucWeekHeader[7][3] = {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"};
	uint16_t usWeekX[7] = {0,  17, 34 , 51, 68, 85, 102};
	uint16_t usWeekY[6] = {17, 26, 35 ,44, 53, 62};
	uint16_t usLineY[12] = {0, 11, 22, 33, 44, 55, 66, 77, 88, 99, 110, 121};
	uint8_t line_pos  = 0;

	uint8_t week_pos  = 0;
	uint32_t week_temp, week_temp_temp, month_temp, enter_temp;
	
	uint32_t month = 1;
	uint16_t wee = 0;	//记录当前月的第一天是星期几
	
	while(1)
	{	
		u8g2_ClearBuffer(&u8g2);			
			
		for(int i=0; i<=6; i++){
			u8g2_DrawStr(&u8g2, usWeekX[i], 8, ucWeekHeader[i]);			
		}
		
		/* month对应月份 */
		month_temp = month_run(month);//获取天数		
		week_temp = judge_week(2024);//获取1月1日是星期几				
		wee = week_temp;
		
		for(int m=1; m<month; m++){
			wee = (wee+month_run(m))%7;        //二月记录当前月的第一天是星期几		
		}
		
		week_temp = wee;		
		week_temp_temp = week_temp;
		
		/* 绘制当前月的日历 */
		for(int k=1; k<=month_temp; k++){			
			enter_temp  = week_temp%7;
			week_temp++;		
			if(k<=(7-week_temp_temp)){
				week_pos=0;
			}else if(enter_temp == 0){
				week_pos = week_pos+1;			
			}
			u8g2_DrawStr(&u8g2, usWeekX[enter_temp], usWeekY[week_pos], ucMonthDay[k]);	
		}
		
		u8g2_DrawLine(&u8g2, 115, 0, 115, 62);
		u8g2_DrawStr(&u8g2, 117, 32, ucMonthDay[line_pos+1]);	
		u8g2_DrawHLine(&u8g2, usLineY[line_pos], 63, 11);
		
		u8g2_SendBuffer(&u8g2);

		/* 读按键中断队列 */
		xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);

		if(key_data.rdata == 1)
		{
			buzzer_buzz(2500, 100);
			month++;
			line_pos++; 
			if(line_pos>11)line_pos=0;
			if(month>12)month=1;
			key_data.rdata = 0;
		}
		
		if(key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
			key_data.exdata = 0;
		}
		//u8g2_SendBuffer(&u8g2);
	}
}
