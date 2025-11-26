#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Data.h"
#include "oled.h"

/* some functions and data */
/* you need use the const keyword to save this data to flash otherwise the ram can't save all data */

/* run! */
void ui_run(int* a, int* a_trg, int b)
{
	if(*a < *a_trg){
		*a+=b;
		if(*a > *a_trg)
		   *a = *a_trg;
	}
	if(*a > *a_trg){
		*a-=b;
		if(*a < *a_trg)
		   *a = *a_trg;
	}
	if(*a == *a_trg){
		*a = *a_trg;
	}
}

/* ÒÆ */
void ui_left(int32_t* a, int b)
{
	uint8_t i=0;
	if((i<=40))
	{
		*a-=b;
		i+=b; 
	}
}

void ui_right(int32_t* a, int b)
{
	uint8_t i=0;
	if((i<=40))
	{
		*a+=b;
		i+=b;
	}
}
/* move_up */
void ui_up(int32_t* a, int b)
{
	uint8_t i_up = 0;
	if((i_up<=20))
	{
		*a-=b;
		i_up+=b;
	}
}
/* move_down */
void ui_down(int32_t* a, int b)
{
	uint8_t i_down = 0;
	if(i_down<=20)
	{
		*a+=b;
		i_down+=b;
	}
}

/*************************/

ui torch = {"torch", 2, {0}, 9, 27, 30, 30}; 

ui hum = {"hum", 3, {0}, 49, 17, 30, 30};
ui clock = {"clock", 4, {0}, 89, 27, 30, 30};
ui setting = {"setting", 5, {0}, 129, 37, 30, 30};
ui cleder = {"cleder", 1, {0}, -30, 37, 30, 30};

const uint8_t light[] = {0};

/*************************/
const uint8_t Num_6x8[12][8] = {0};

const uint8_t wooden_flsh[2][384] = {0};

const uint8_t hammer[40] = {0};

const uint8_t gongde[64] = {0};

const uint8_t ShowPower[30] = {0};
const uint8_t ShowGame[30] = {0};
const uint8_t LeftMove[30] = {0};
const uint8_t RightMove[30] = {0};
const uint8_t BigNum[10][120] = {0};

