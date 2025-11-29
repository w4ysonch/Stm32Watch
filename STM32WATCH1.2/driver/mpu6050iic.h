/*
 * mpu6050iic.h
 *
 *  Created on: Feb 24, 2024
 *      Author: wolf.long
 */
#include "main.h"
#include "gpio.h"
#ifndef MPU6050_MPU6050IIC_H_
#define MPU6050_MPU6050IIC_H_

//IO操作函数
#define MPU_IIC_SCL(x)    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, x) 		//SCL
#define MPU_IIC_SDA(x)    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, x) 		//SDA
#define MPU_READ_SDA   	  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) 		//输入SDA


//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(uint8_t ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

#endif /* MPU6050_MPU6050IIC_H_ */
