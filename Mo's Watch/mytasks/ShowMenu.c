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
extern TaskHandle_t xShowStepTaskHandle;

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
    OLED_Clear();
    

    OLED_ShowString(40, 0, "MENU", 16, 0);
    

    OLED_ShowString(40, 1, (char*)str[str_flag], 12, 1); 
    
    OLED_ShowString(10, 2, "Cal", 12, dock_pos == 0 ? 1 : 0);
    OLED_ShowString(50, 2, "Light", 12, dock_pos == 1 ? 1 : 0);
    OLED_ShowString(90, 2, "Hum", 12, dock_pos == 2 ? 1 : 0);
    
    // ???  
    OLED_ShowString(10, 4, "Clock", 12, dock_pos == 3 ? 1 : 0);
    OLED_ShowString(50, 4, "Set", 12, dock_pos == 4 ? 1 : 0);
    OLED_ShowString(90, 4, "More", 12, 0);
    
    /* ????????? */
    OLED_ShowString(0, 6, "<", 12, 0);
    OLED_ShowString(120, 6, ">", 12, 0);
    
    // ??????
    for(int i = 0; i < 6; i++)
    {
        if(i == dock_pos)
        {
            OLED_ShowChar(30 + i * 15, 6, '#', 12, 0); // ????
        }
        else
        {
            OLED_ShowChar(30 + i * 15, 6, '.', 12, 0); // ???
        }
    }
    

    OLED_ShowString(10, 7, "Sel:OK", 12, 0);
    OLED_ShowString(70, 7, "Back:Up", 12, 0);
}


void simple_animation_left(void)
{
    for(int i = 0; i < 3; i++)
    {
        ShowUI();
        OLED_ShowString(60, 3, "<--", 12, 0);
        vTaskDelay(50);
    }
}

void simple_animation_right(void)
{
    for(int i = 0; i < 3; i++)
    {
        ShowUI();
        OLED_ShowString(60, 3, "-->", 12, 0);
        vTaskDelay(50);
    }
}

void ShowMenuTask(void *params)
{
    buzzer_init();
    
    g_xQueueMenu = xQueueCreate(4, 4);
    if(NULL != g_xQueueMenu) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    
    
    OLED_Init();
    OLED_Clear();
    OLED_Display_On();
    
    struct Key_data key_data;
    
    while(1)
    {
        ShowUI();
        
        if(xQueueReceive(g_xQueueMenu, &key_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            if(key_data.rdata == 1 && dock_pos > 0)
            {
                simple_animation_right();
                dock_pos--;
                str_flag--;
                buzzer_buzz(2000, 50);
            }
            else if(key_data.ldata == 1 && dock_pos < 5)
            {
                simple_animation_left();
                dock_pos++;
                str_flag++;
                buzzer_buzz(2000, 50);
            }
            else if(key_data.exdata == 1)
            {
                buzzer_buzz(2500, 100);
                switch(dock_pos)
                {
                    case 0: vTaskResume(xShowCalendarTaskHandle); vTaskSuspend(NULL); break;
                    case 1: vTaskResume(xShowFlashLightTaskHandle); vTaskSuspend(NULL); break;
                    case 2: vTaskResume(xShowDHT11TaskHandle); vTaskSuspend(NULL); break;
                    case 3: vTaskResume(xShowClockTaskHandle); vTaskSuspend(NULL); break;
                    case 4: vTaskResume(xShowSettingTaskHandle); vTaskSuspend(NULL); break;
										case 5: vTaskResume(xShowStepTaskHandle); vTaskSuspend(NULL); break;
                }
            }
            else if(key_data.updata == 1)
            {
                buzzer_buzz(2000, 100);
                vTaskResume(xShowTimeTaskHandle);
                vTaskSuspend(NULL);
            }
        }
        
        vTaskDelay(10);
    }
}