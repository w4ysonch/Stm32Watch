/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "driver_timer.h"
#include "StepCountTask.h"
#include <string.h>

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

long int g_step_cnt = 0;

void StepCountTask(void *params)
{
	static step_ctx_t ctx = {0};
	
	MPU_Init();
	while(mpu_dmp_init()) vTaskDelay(200);
	
	while (1) {
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
			if (ctx.old_sample.x > th && ctx.new_sample.x < th) g_step_cnt++;
		} else if (axis == 2) {
			short th = (ctx.oldmax.y + ctx.oldmin.y) / 2;
			if (ctx.old_sample.y > th && ctx.new_sample.y < th) g_step_cnt++;
		} else if (axis == 3) {
			short th = (ctx.oldmax.z + ctx.oldmin.z) / 2;
			if (ctx.old_sample.z > th && ctx.new_sample.z < th) g_step_cnt++;
		}
		
		vTaskDelay(200);
	}
}
