/*
 * mpu6050iic.c
 *
 *  Created on: Feb 24, 2024
 *      Author: wolf.long
 */
#include "mpu6050iic.h"

//初始化IIC
void MPU_IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
}
//IO方向设置z
void MPU_SDA_OUT()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void MPU_SDA_IN()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MPU_IIC_SDA(1);
	MPU_IIC_SCL(1);
	udelay(2);
	MPU_IIC_SDA(0);//START:when CLK is high,DATA change form high to low
	udelay(2);
	MPU_IIC_SCL(0);//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	MPU_IIC_SCL(0);
	MPU_IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
	udelay(2);
	MPU_IIC_SCL(1);
	MPU_IIC_SDA(1);//发送I2C总线结束信号
	udelay(2);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入
	MPU_IIC_SDA(1);udelay(2);
	MPU_IIC_SCL(1);udelay(2);
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL(0);//时钟输出0
	return 0;
}
//产生ACK应答
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(0);
	udelay(2);
	MPU_IIC_SCL(1);
	udelay(2);
	MPU_IIC_SCL(0);
}
//不产生ACK应答
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(1);
	udelay(2);
	MPU_IIC_SCL(1);
	udelay(2);
	MPU_IIC_SCL(0);
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void MPU_IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	MPU_SDA_OUT();
	MPU_IIC_SCL(0);//拉低时钟开始数据传输
  for(t=0;t<8;t++)
  {
		MPU_IIC_SDA((txd&0x80)>>7);
    txd<<=1;
    MPU_IIC_SCL(1);
    udelay(2);
    MPU_IIC_SCL(0);
		udelay(2);
   }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t MPU_IIC_Read_Byte(uint8_t ack)
{
	uint8_t i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    	MPU_IIC_SCL(0);
    	udelay(2);
    	MPU_IIC_SCL(1);
      receive<<=1;
      if(MPU_READ_SDA)receive++;
      udelay(2);
   }
   if (!ack)
       MPU_IIC_NAck();//发送nACK
   else
       MPU_IIC_Ack(); //发送ACK
   return receive;
}
