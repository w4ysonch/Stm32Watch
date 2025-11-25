/*  Copyright (s) 2019 深圳百问网科技有限公司
 *  All rights reserved
 * 
 * 文件名称：beep.c
 * 摘要：
 *  
 * 修改历史     版本号        Author       修改内容
 *--------------------------------------------------
 * 2023.9.08      v01         百问科技      创建文件
 *--------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

#include "driver_passive_buzzer.h"

static TimerHandle_t g_TimerSound;
extern int power_button;

/**********************************************************************
 * 函数名称： GameSoundTimer_Func
 * 功能描述： 游戏声音的定时器函数,用来停止声音
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/09/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void SystemSoundTimer_Func( TimerHandle_t xTimer )
{
	PassiveBuzzer_Control(0);
}

/**********************************************************************
 * 函数名称： buzzer_init
 * 功能描述： 初始化蜂鸣器并创建定时器(用于停止声音)
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/09/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
void buzzer_init(void)
{
	/* 初始化蜂鸣器 */
	PassiveBuzzer_Init();	
	
	/* 创建定时器 */

	g_TimerSound = xTimerCreate( "SystemSound", 
							200,
							pdFALSE,
							NULL,
							SystemSoundTimer_Func);
}


/**********************************************************************
 * 函数名称： buzzer_buzz
 * 功能描述： 启动蜂鸣器发出一段声音
 * 输入参数： freq - 声音频率
 *            time_ms - 持续时间
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/09/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
void buzzer_buzz(int freq, int time_ms)
{
	if(power_button == 0)
	{
		PassiveBuzzer_Set_Freq_Duty(freq, 50);		
		/* 启动定时器 */
		xTimerChangePeriod(g_TimerSound, time_ms, 0);		
	}

}
