#include "stm32f1xx_hal.h"                // Device header
#include "Delay.h"
#include "Key.h"
#include "OLED.h"
extern u8 keynum,mode;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1|GPIO_Pin_3 | GPIO_Pin_5;         
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

uint8_t Key_GetNum(void)
{
  uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0);
		Delay_ms(20);
		KeyNum = 1;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0);
		Delay_ms(20);
		KeyNum = 2;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0);
		Delay_ms(20);
		KeyNum = 3;
	}
	return KeyNum;
}


extern unsigned long walk,Distance;
extern float steplength;

void Key_control(void)
{
		keynum=Key_GetNum();
	  if(keynum==1)
		{
				if(mode==0)
				{  
					 OLED_Clear();
					 mode=1;         //切换设置步距界面,mode=1
				}
				else if(mode==1||mode==2||mode==3)
				{  
					 OLED_Clear();
					 mode=0;        //初始界面
				 }
		 }
	  else if(keynum==2)      
		{
			  
				if(mode==1)       //设置界面时,按一下步距+10
				{
					 steplength+=0.1;
					 if(steplength>=1.3)steplength=0.2;
				 }
				else if(mode==0||mode==3){OLED_Clear();mode=2;} //加速度界面
				else if(mode==2){OLED_Clear();mode=0;}   //切换初始界面
		 }
	   else if(keynum==3)
		{
				if(mode==1)      //设置界面时,按一下-10
				{
					 steplength-=0.1;
					 if(steplength<=0.1)steplength=1.2;
				 }
				else if(mode==0||mode==2){OLED_Clear();mode=3;} //欧拉角界面
				else if(mode==3){OLED_Clear();mode=0;}   //切换初始界面
		 }
}
