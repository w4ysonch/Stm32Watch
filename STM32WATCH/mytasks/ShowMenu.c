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
#include "u8g2.h"
#include "Data.h"

extern u8g2_t u8g2;
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

const char str[5][10] = {"cleder", "torch", "hum", "clock", "more"};

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
uint8_t dock_status = 10;
uint8_t dock[5] = {45, 55, 65, 75, 85};
uint8_t dock_y = 58, dock_r = 3;  
int str_flag = 2;
int8_t R_move_pos[5] = {-1, 9, 49, 89, 129};
BaseType_t select = 3;

int queue_flag = 0;
uint32_t end_flag = 1;
uint32_t seclect_end = 0;//showsetting

/* draw app's icon */
void ShowUI(void)
{
	/* show_gameui */
	u8g2_DrawXBMP(&u8g2, Left.x, Left.y, Left.w, Left.h, LeftMove);
	u8g2_DrawXBMP(&u8g2, Right.x, Right.y, Right.w, Right.h, RightMove);
	
	u8g2_DrawStr(&u8g2, String.x, String.y, str[str_flag]);

	u8g2_DrawXBMP(&u8g2, cleder.x, cleder.y, cleder.w, cleder.h, cleder.data);
	u8g2_DrawXBMP(&u8g2, torch.x, torch.y, torch.w, torch.h, torch.data);	
	u8g2_DrawXBMP(&u8g2, hum.x, hum.y, hum.w, hum.h, hum.data);
	u8g2_DrawXBMP(&u8g2, clock.x, clock.y, clock.w, clock.h, clock.data);
	u8g2_DrawXBMP(&u8g2, setting.x, setting.y, setting.w, setting.h, setting.data);

	u8g2_DrawDisc(&u8g2, dock[dock_pos], dock_y, dock_r, U8G2_DRAW_ALL);
	for(int i = 0; i<5; i++)
	{
		u8g2_DrawCircle(&u8g2, dock[i], dock_y, dock_r, U8G2_DRAW_ALL);
	}	
	u8g2_DrawFrame(&u8g2, Rec_select.x, Rec_select.y, Rec_select.w, Rec_select.h);
};

void ShowMenuTask(void *params)
{
//	xSemaphoreTake(g_xQueueMenu, portMAX_DELAY);
	
	/* system sound */
	buzzer_init();
   
	/* create queue */
	g_xQueueMenu = xQueueCreate(4, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	
	/*u8g2_config*/
	u8g2_config();
	/* ShowUI */
//	u8g2_SetFont(&u8g2, u8g2_font_wqy16_t_chinese1);
	u8g2_FirstPage(&u8g2);
	do {
	u8g2_SendBuffer(&u8g2);
   	} while (u8g2_NextPage(&u8g2));
	
	struct Key_data	key_data;
	
	while(1)
	{
		u8g2_ClearBuffer(&u8g2);
		ShowUI();
		u8g2_SendBuffer(&u8g2);
		/* receive queue data and keep waitting */
		if(queue_flag == 0)
		{
			pdPASS == xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
		}
		/* handle data */
		if(key_data.rdata == 1)
		{	
			end_flag = 0;
//			if(end_flag!)
			if(dock_pos != 0)
			{
				ui_right(&cleder.x, 2);ui_right(&torch.x, 2);ui_right(&hum.x, 2);ui_right(&clock.x, 2);ui_right(&setting.x, 2);
				/* state_machine */
				if(dock_status==0)dock_status=1;
				dock_status--;
				switch(dock_pos)
				{
					case 2:if(dock_status!=0){ui_up(&cleder.y, 1);ui_up(&torch.y, 1);ui_down(&hum.y, 1);ui_down(&clock.y, 1);}break;
				    case 1:if(dock_status!=0){ui_up(&cleder.y, 1);ui_up(&setting.y, 1);ui_down(&torch.y, 1);ui_down(&hum.y, 1);}break;
					case 4:if(dock_status!=0){ui_up(&clock.y, 1);ui_up(&hum.y, 1);ui_down(&setting.y, 1);ui_down(&cleder.y, 1);}break;
					case 3:if(dock_status!=0){ui_up(&hum.y, 1);ui_up(&torch.y, 1);ui_down(&setting.y, 1);ui_down(&clock.y, 1);}break;
				}
			}
            queue_flag++;	
			if(queue_flag == 20)
			{
				dock_status=10;
				end_flag = 1;
				if(dock_pos != 0){dock_pos--;str_flag--;}
				queue_flag = 0;
				key_data.rdata = 0;
				key_data.ldata = 0;
			}
			if(end_flag == 1)buzzer_buzz(2000, 100);
		}
		else if(key_data.ldata == 1)
		{
            end_flag = 0;
			if(dock_pos < 4)
			{		
				ui_left(&cleder.x, 2);ui_left(&torch.x, 2);ui_left(&hum.x, 2);ui_left(&clock.x, 2);ui_left(&setting.x, 2);
				/* state_machine */
				if(dock_status==0)dock_status=1;
				dock_status--;
				switch(dock_pos)
				{
					case 0:if(dock_status!=0){ui_up(&hum.y, 1);ui_up(&torch.y, 1);ui_down(&cleder.y, 1);ui_down(&setting.y, 1);}break;
				    case 1:if(dock_status!=0){ui_up(&hum.y, 1);ui_up(&clock.y, 1);ui_down(&torch.y, 1);ui_down(&cleder.y, 1);}break;
					case 2:if(dock_status!=0){ui_up(&clock.y, 1);ui_up(&setting.y, 1);ui_down(&hum.y, 1);ui_down(&torch.y, 1);}break;
					case 3:if(dock_status!=0){ui_up(&setting.y, 1);ui_up(&cleder.y, 1);ui_down(&clock.y, 1);ui_down(&hum.y, 1);}break;
				}				
			}
            queue_flag++;	
			if(queue_flag == 20) 
			{
				dock_status = 10;
				end_flag = 1;
				if(dock_pos < 4){dock_pos++;str_flag++;}
				queue_flag = 0;
				key_data.ldata = 0;
				key_data.rdata = 0;
			}
			if(end_flag == 1)buzzer_buzz(2000, 100);
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
				case 4: vTaskResume(xShowSettingTaskHandle);vTaskSuspend(NULL);key_data.exdata = 0;break;
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

