/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
#include "Data.h"
#include "semphr.h"

/*  
*
*
*   this file is not used ,you can use this file to change Scheduling mode of tasks
*	
*
*
*/
/* Private includes ----------------------------------------------------------*/
extern QueueHandle_t g_xQueueMenu;

extern TaskHandle_t xShowTimeTaskHandle;
extern TaskHandle_t xShowMenuTaskHandle;
extern TaskHandle_t xShowCalendarTaskHandle;
extern TaskHandle_t xShowClockTaskHandle;
extern TaskHandle_t xShowFlashLightTaskHandle;
extern TaskHandle_t xShowSettingTaskHandle;
extern TaskHandle_t xShowWoodenFishTaskHandle;
//extern TaskHandle_t xShowDht11TaskHandle;

extern uint16_t TimeTaskRunning;
extern uint16_t MenuTaskRunning;
extern uint16_t WfisTaskRuning;
SemaphoreHandle_t g_xSemMenu; 

uint32_t root_flag = 0;
        
void RootTask(void)
{
		g_xSemMenu = xSemaphoreCreateMutex();

	vTaskSuspend(xShowTimeTaskHandle);
	vTaskSuspend(xShowMenuTaskHandle);
//	vTaskSuspend(xShowCalendarTaskHandle);
//	vTaskSuspend(xShowClockTaskHandle);
//	vTaskSuspend(xShowFlashLightTaskHandle);
//	vTaskSuspend(xShowSettingTaskHandle);
//	vTaskSuspend(xShowWoodenFishTaskHandle);
//	vTaskSuspend(xShowDht11TaskHandle);

	/* 创建队列 */
	g_xQueueMenu = xQueueCreate(1, 4);	
	struct Key_data	key_data;
		vTaskResume(xShowTimeTaskHandle);
	while(1)
	{	

		/* 读按键中断队列 */
		if(root_flag == 0)
		{
			pdPASS == xQueueReceive(g_xQueueMenu, &key_data, portMAX_DELAY);
		}
		/* processing data */
		if(key_data.updata == 1 && TimeTaskRunning == 1)
		{
			vTaskSuspend(xShowTimeTaskHandle);
						vTaskSuspend(xShowWoodenFishTaskHandle);
			vTaskResume(xShowMenuTaskHandle);
			//xSemaphoreGive(g_xSemMenu);
			key_data.updata = 0;
			TimeTaskRunning = 0;
		}
//		if(key_data.updata == 1 && MenuTaskRunning == 1 && TimeTaskRunning == 0)
//		{
//				vTaskSuspend(xShowTimeTaskHandle);
//			vTaskSuspend(xShowMenuTaskHandle);
//			vTaskResume(xShowWoodenFishTaskHandle);
//			//xSemaphoreGive(g_xSemMenu);
//			key_data.updata = 0;
//			MenuTaskRunning = 0;
//		}
		if(key_data.exdata == 1 && MenuTaskRunning == 1 &&TimeTaskRunning == 0)
		{
				vTaskSuspend(xShowWoodenFishTaskHandle);
			vTaskSuspend(xShowMenuTaskHandle);
			vTaskResume(xShowTimeTaskHandle);
			key_data.exdata = 0;
			MenuTaskRunning = 0;
		}
//		if(key_data.exdata == 1 && WfisTaskRuning == 1 && MenuTaskRunning == 0)
//		{
//			vTaskSuspend(xShowWoodenFishTaskHandle);
//				//vTaskSuspend(xShowTimeTaskHandle);
//			vTaskResume(xShowMenuTaskHandle);
//			key_data.exdata = 0;
//			WfisTaskRuning = 0;
//		}		
	}
}

