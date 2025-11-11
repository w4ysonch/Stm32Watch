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
#include "ShowBMP280.h"
#include "bmp280.h"
#include "i2c.h"
#include "oled.h"
#include <math.h>
/* USER CODE END Includes */

extern TaskHandle_t xShowMenuTaskHandle;

double pow(double X,double Y);
// 从BMP280读取的空气压力和温度计算海拔高度
// 使用大气压强公式：h = ((P0/P)^(1/5.257) - 1) * T / 0.0065
// 其中P0=101325 Pa（标准大气压），5.257≈1/0.190（空气绝热指数倒数），0.0065为温度梯度（K/m）
double data_conversion(double air_pressure,double temperature)
{
	double conversion_alt;
	// 计算海拔高度：先计算压力比的幂，再调整温度和梯度
	conversion_alt=(pow(101325/air_pressure,1/5.257)-1)*temperature/0.0065f;
	return conversion_alt;
}


void ShowBMP280Task(void)
{
	double Alt;                             // 存储计算出的海拔高度（单位：米）
	bmp280_params_t params;                 // BMP280传感器配置参数结构体（包含模式、滤波、过采样等设置）
	BMP280_HandleTypedef bmp280_dev;        // BMP280设备句柄结构体（包含I2C地址、校准数据等）
	float pressure, temperature, humidity;  // 存储从BMP280读取的浮点数据：压力（Pa）、温度（°C）、湿度（%）	
	
	OLED_Init();
    OLED_Clear();
	
	bmp280_init_default_params(&params);            // 初始化BMP280默认配置参数（正常模式、无滤波、4倍过采样、250ms待机时间）
	bmp280_dev.addr = BMP280_I2C_ADDRESS_0;         // 设置BMP280的I2C地址（0x76）
	bmp280_dev.i2c = &hi2c2;                        // 绑定I2C2句柄用于通信
	bmp280_init(&bmp280_dev, &params);              // 初始化BMP280模块，读取校准数据并配置传感器
	bool bme280p = bmp280_dev.id == BME280_CHIP_ID; // 检查芯片ID，确定是否为BME280（支持湿度测量）

	while(1)
	{
		char buffer[50]; // 用于格式化显示字符串的缓冲区
		bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity); // 从BMP280读取浮点数据：温度、压力、湿度
		Alt = data_conversion(pressure, temperature); // 使用读取的压力和温度计算海拔高度
		
		sprintf(buffer, "pre: %.2f", pressure);
		OLED_ShowString(0, 0, buffer, 16, 0);
		
		sprintf(buffer, "temp: %.2f", temperature);
		OLED_ShowString(0, 2, buffer, 16, 0);
		
		sprintf(buffer, "alt: %.2f", Alt);
		OLED_ShowString(0, 4, buffer, 16, 0);
		
		if (bme280p)
		{
		  sprintf(buffer, "hum: %.2f", humidity);
		  OLED_ShowString(0, 6, buffer, 16, 0);
		}
		else
		{
		  OLED_ShowString(0, 6, "hum: N/A", 16, 0);
		}
	}
}

