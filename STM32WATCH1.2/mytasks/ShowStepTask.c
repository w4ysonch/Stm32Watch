/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
// #include <stdio.h> // 不需要标准IO，避免拉入大库
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beep.h"
#include "oled.h"
#include "Data.h"
#include "ShowCalendar.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "driver_timer.h"
#include <string.h>
/* USER CODE END Includes */

#define FILTER_CNT 4
#define SAMPLE_SIZE 50
#define DYNAMIC_PRECISION 30
#define ACTIVE_PRECISION 60
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define ABS(a)   (((a) > 0) ? (a) : (-(a)))

typedef struct {
		short x, y, z;
} axis3_t;

typedef struct {
		axis3_t newmax, newmin, oldmax, oldmin;
		axis3_t new_sample, old_sample; // 合并slid_reg
		axis3_t filter[FILTER_CNT];
		unsigned char filter_count;
		unsigned int sample_size;
} step_ctx_t;

static long int step_cnt = 0;



extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;


void ShowStepTask(void *params)
{
	static step_ctx_t ctx = {0};
	struct Key_data key_data;
	MPU_Init();
	while(mpu_dmp_init()) vTaskDelay(200);
	OLED_Init();
	OLED_Clear();
	OLED_Display_On();
	if(NULL != g_xQueueMenu) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	while (1) {
		OLED_Clear();
		// 滤波采样
		short x_sum = 0, y_sum = 0, z_sum = 0;
		for (int i = 0; i < FILTER_CNT; i++) {
			int retry = 0;
			while (MPU_Get_Accelerometer(&ctx.filter[i].x, &ctx.filter[i].y, &ctx.filter[i].z)) {
				vTaskDelay(1);
				if (++retry > 10) break;
			}
			x_sum += ctx.filter[i].x;
			y_sum += ctx.filter[i].y;
			z_sum += ctx.filter[i].z;
		}
		axis3_t sample = {
			.x = x_sum / FILTER_CNT,
			.y = y_sum / FILTER_CNT,
			.z = z_sum / FILTER_CNT
		};

		// 峰值检测与滑动窗口
		ctx.sample_size++;
		if (ctx.sample_size > SAMPLE_SIZE) {
			ctx.sample_size = 1;
			ctx.oldmax = ctx.newmax;
			ctx.oldmin = ctx.newmin;
		}
		ctx.newmax.x = MAX(ctx.newmax.x, sample.x);
		ctx.newmax.y = MAX(ctx.newmax.y, sample.y);
		ctx.newmax.z = MAX(ctx.newmax.z, sample.z);
		ctx.newmin.x = MIN(ctx.newmin.x, sample.x);
		ctx.newmin.y = MIN(ctx.newmin.y, sample.y);
		ctx.newmin.z = MIN(ctx.newmin.z, sample.z);

		// 滑动检测
		#define SLID_UPDATE(axis) do { \
			if (ABS(sample.axis - ctx.new_sample.axis) > DYNAMIC_PRECISION) { \
				ctx.old_sample.axis = ctx.new_sample.axis; \
				ctx.new_sample.axis = sample.axis; \
			} else { \
				ctx.old_sample.axis = ctx.new_sample.axis; \
			} \
		} while(0)
		SLID_UPDATE(x); SLID_UPDATE(y); SLID_UPDATE(z);

		// 步数判定
		short xchg = ABS(ctx.newmax.x - ctx.newmin.x);
		short ychg = ABS(ctx.newmax.y - ctx.newmin.y);
		short zchg = ABS(ctx.newmax.z - ctx.newmin.z);
		char axis = 0;
		if (xchg > ychg && xchg > zchg && xchg >= ACTIVE_PRECISION) axis = 1;
		else if (ychg > xchg && ychg > zchg && ychg >= ACTIVE_PRECISION) axis = 2;
		else if (zchg > xchg && zchg > ychg && zchg >= ACTIVE_PRECISION) axis = 3;

		if (axis == 1) {
			short th = (ctx.oldmax.x + ctx.oldmin.x) / 2;
			if (ctx.old_sample.x > th && ctx.new_sample.x < th) step_cnt++;
		} else if (axis == 2) {
			short th = (ctx.oldmax.y + ctx.oldmin.y) / 2;
			if (ctx.old_sample.y > th && ctx.new_sample.y < th) step_cnt++;
		} else if (axis == 3) {
			short th = (ctx.oldmax.z + ctx.oldmin.z) / 2;
			if (ctx.old_sample.z > th && ctx.new_sample.z < th) step_cnt++;
		}

		OLED_ShowString(15, 0, "step", 16, 0);
		OLED_ShowNum(10, 2, step_cnt, 5, 16, 0);
		vTaskDelay(200);

		/* 统一切换机制：收到退出信号时resume菜单并挂起自己 */
		if(xQueueReceive(g_xQueueMenu, &key_data, 1000) == pdPASS && key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);
			vTaskResume(xShowMenuTaskHandle);
			key_data.exdata = 0;
			vTaskSuspend(NULL);
		}
	}
}
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/* USER CODE BEGIN 4 */

