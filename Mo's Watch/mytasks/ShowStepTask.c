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
#include "driver_dht11.h"
#include "oled.h"
#include "Data.h"
#include "ShowCalendar.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "driver_timer.h"
#include <string.h>
/* USER CODE END Includes */
#define FILTER_CNT			4

typedef struct {
  short x;
  short y;
  short z;
}axis_info_t;

typedef struct filter_avg{
	axis_info_t info[FILTER_CNT];
	unsigned char count;
}filter_avg_t;

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define SAMPLE_SIZE   50

typedef struct {
	axis_info_t newmax;
	axis_info_t newmin;
	axis_info_t oldmax;
	axis_info_t oldmin;
}peak_value_t;

#define ABS(a) (0 - (a)) > 0 ? (-(a)) : (a)
#define DYNAMIC_PRECISION     			30     	 /*????*/

/*?????????,????????*/
typedef struct slid_reg{
	axis_info_t new_sample;
	axis_info_t old_sample;
}slid_reg_t;

#define MOST_ACTIVE_NULL					0      		
#define MOST_ACTIVE_X					1		 
#define MOST_ACTIVE_Y					2    
#define MOST_ACTIVE_Z					3      

#define ACTIVE_PRECISION      			60     

static void filter_calculate(filter_avg_t *filter, axis_info_t *sample);
static void peak_update(peak_value_t *peak, axis_info_t *cur_sample);
static char slid_update(slid_reg_t *slid, axis_info_t *cur_sample);
static void detect_step(peak_value_t *peak, slid_reg_t *slid, axis_info_t *cur_sample);

static long int step_cnt = 0;

filter_avg_t filter;
axis_info_t sample;
peak_value_t peak;
slid_reg_t slid;



extern TaskHandle_t xShowMenuTaskHandle;
extern QueueHandle_t g_xQueueMenu;

void ShowStepTask(void *params)
{
	OLED_Init();
  OLED_Clear(); 
  OLED_Display_On(); 
	MPU_Init();
  while(mpu_dmp_init())
  {
	  mdelay(200);
	  printf("mpu6050 error\r\n");
  };
	g_xQueueMenu = xQueueCreate(1, 4);
	if(NULL != g_xQueueMenu)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);


	struct Key_data	key_data;


	
  while (1)
  {
		OLED_Clear();
    OLED_ShowString(15, 0, "step", 16, 0);  
    OLED_ShowNum(10, 2, step_cnt, 5, 16, 0); 


	  filter_calculate(&filter, &sample);
	  peak_update(&peak, &sample);
	  slid_update(&slid, &sample);
	  detect_step(&peak, &slid, &sample);

	  mdelay(200);
		
		xQueueReceive(g_xQueueMenu, &key_data, 0);
		
		if(key_data.exdata == 1)
		{
			buzzer_buzz(2500, 100);
			vTaskResume(xShowMenuTaskHandle);
			vTaskSuspend(NULL);
			key_data.exdata = 0;
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

//??xyz?????????,??????,???????sample
static void filter_calculate(filter_avg_t *filter, axis_info_t *sample)
{
	unsigned int i = 0;
	unsigned char j = 0;
	short x_sum = 0, y_sum = 0, z_sum = 0;
	filter->count = FILTER_CNT;
	for (i = 0; i < FILTER_CNT; i++) {
		while(MPU_Get_Accelerometer(&filter->info[i].x,&filter->info[i].y,&filter->info[i].z))
		{
			udelay(100);
			j++;
			if(j > 20)
				break;
		}
		x_sum += filter->info[i].x;
		y_sum += filter->info[i].y;
		z_sum += filter->info[i].z;
	}
	sample->x = x_sum / FILTER_CNT;
	sample->y = y_sum / FILTER_CNT;
	sample->z = z_sum / FILTER_CNT;
}

//????????????,????max?????????,min??????,??????????
static void peak_update(peak_value_t *peak, axis_info_t *cur_sample)
{
  	static unsigned int sample_size = 0;
    sample_size ++;
	if (sample_size > SAMPLE_SIZE) {
		/*????50?,????*/
		sample_size = 1;
		peak->oldmax = peak->newmax;
		peak->oldmin = peak->newmin;
	}
	peak->newmax.x = MAX(peak->newmax.x, cur_sample->x);
	peak->newmax.y = MAX(peak->newmax.y, cur_sample->y);
	peak->newmax.z = MAX(peak->newmax.z, cur_sample->z);

	peak->newmin.x = MIN(peak->newmin.x, cur_sample->x);
	peak->newmin.y = MIN(peak->newmin.y, cur_sample->y);
	peak->newmin.z = MIN(peak->newmin.z, cur_sample->z);
}

static char slid_update(slid_reg_t *slid, axis_info_t *cur_sample)
{
  	char res = 0;
  	if (ABS((cur_sample->x - slid->new_sample.x)) > DYNAMIC_PRECISION) {
		slid->old_sample.x = slid->new_sample.x;
		slid->new_sample.x = cur_sample->x;
		res = 1;
	} else {
		slid->old_sample.x = slid->new_sample.x;
	}
	if (ABS((cur_sample->y - slid->new_sample.y)) > DYNAMIC_PRECISION) {
		slid->old_sample.y = slid->new_sample.y;
		slid->new_sample.y = cur_sample->y;
		res = 1;
	} else {
		slid->old_sample.y = slid->new_sample.y;
	}

	if (ABS((cur_sample->z - slid->new_sample.z)) > DYNAMIC_PRECISION) {
		slid->old_sample.z = slid->new_sample.z;
		slid->new_sample.z = cur_sample->z;
		res = 1;
	} else {
		slid->old_sample.z = slid->new_sample.z;
	}
	return res;
}

/*????????*/
static char is_most_active(peak_value_t *peak)
{
	char res = MOST_ACTIVE_NULL;
	short x_change = ABS((peak->newmax.x - peak->newmin.x));
	short y_change = ABS((peak->newmax.y - peak->newmin.y));
	short z_change = ABS((peak->newmax.z - peak->newmin.z));

	if (x_change > y_change && x_change > z_change && x_change >= ACTIVE_PRECISION) {
		res = MOST_ACTIVE_X;
	} else if (y_change > x_change && y_change > z_change && y_change >= ACTIVE_PRECISION) {
		res = MOST_ACTIVE_Y;
	} else if (z_change > x_change && z_change > y_change && z_change >= ACTIVE_PRECISION) {
		res = MOST_ACTIVE_Z;
	}
	return res;
}

/*??????*/
static void detect_step(peak_value_t *peak, slid_reg_t *slid, axis_info_t *cur_sample)
{
	char res = is_most_active(peak);
	switch (res) {
		case MOST_ACTIVE_NULL: {
			//fix
			break;
		}
		case MOST_ACTIVE_X: {
			short threshold_x = (peak->oldmax.x + peak->oldmin.x) / 2;
			if (slid->old_sample.x > threshold_x && slid->new_sample.x < threshold_x) {
				step_cnt ++;
			}
			break;
		}
		case MOST_ACTIVE_Y: {
			short threshold_y = (peak->oldmax.y + peak->oldmin.y) / 2;
			if (slid->old_sample.y > threshold_y && slid->new_sample.y < threshold_y) {
				step_cnt ++;
			}
			break;
		}
		case MOST_ACTIVE_Z: {
			short threshold_z = (peak->oldmax.z + peak->oldmin.z) / 2;
			if (slid->old_sample.z > threshold_z && slid->new_sample.z < threshold_z) {
				step_cnt ++;
			}
			break;
		}
		default:
			break;
	}
}
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
