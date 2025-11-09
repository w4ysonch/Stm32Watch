#include "mpu6050.h"
#include "oled.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"   
#include "mpuiic.h"
#include "mpu6050.h"//MPU6050驱动库
#include "inv_mpu.h"//MPU6050计步驱动库
#include "inv_mpu_dmp_motion_driver.h"//MPU6050计步，计时驱动库
#include <math.h>
#include "Key.h"

int SVM;
u8 mpu_1_flag,mpu_2_flag;
u8 i=10;
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
unsigned long walk;
float steplength=0.3,Distance; //步距/米
u8 svm_set=1;        //路程   
short GX,GY,GZ;
u8 num;

//正点原子MPU6050驱动

//初始化MPU6050
void MPU_Init(void)
{ 
	MPU_IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x00);//解除休眠状态
	MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x07);//陀螺仪采样率，典型值：0x07(125Hz)
	MPU_Write_Byte(MPU_CFG_REG,0x06);//低通滤波频率，典型值：0x06(5Hz)
	MPU_Write_Byte(MPU_GYRO_CFG_REG,0x18);//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	MPU_Write_Byte(MPU_ACCEL_CFG_REG,0x01);//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
}
//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答 
	MPU_IIC_Send_Byte(data);//发送数据
	if(MPU_IIC_Wait_Ack())	//等待ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_IIC_Wait_Ack();		//等待应答 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
    MPU_IIC_Stop();			//产生一个停止条件 
	return res;		
}




//步数获取函数
void dmp_getwalk(void){
			dmp_get_pedometer_step_count(&walk);
	    OLED_ShowNum(1,7,walk,4);   //显示步数数值
	    Distance=steplength*walk;
      OLED_ShowFloat(2,7,Distance,1);	 //显示路程,单位cm
}

extern u8 mode;
//摔倒判断函数
void dmp_svm(void){
	dmp_getwalk();
	if(svm_set){	
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){
//			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			SVM = sqrt(pow(aacx,2)+  pow(aacy,2) + pow(aacz,2));	
			//printf("pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n",fabs(pitch),fabs(roll),fabs(yaw),SVM);
			//分析x、y、z角度的异常判断
			if( fabs(pitch)>60 || fabs(roll)>60 )//倾斜大于60°即认为摔倒
				      //fabs函数主要用于对浮点数或者整数类型取绝对值
				mpu_1_flag = 1;	
			else 
				mpu_1_flag = 0;

			//分析加速度SVM的异常判断
			if( SVM>23000 || SVM<12000 )i = 0;   //瞬时速度总值
			i++;
			
			if( i<=10 )mpu_2_flag = 1;
			else{
				i = 10;
				mpu_2_flag = 0;
			}
	
			//综合欧拉角、SVM异常判断异常	
			if( mpu_2_flag || mpu_1_flag ){
			//跌倒函数
				OLED_ShowChinese(4,5,4);
			}else {
			//非跌倒函数			
				OLED_ShowChinese(4,5,5);
			}
//			printf("pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n",fabs(pitch),fabs(roll),fabs(yaw),SVM);
//			printf("x=%d,y=%d,z=%d,2flag=%d,1flag=%d,flag=%d \r\n",aacx,aacy,aacz,mpu_2_flag,mpu_1_flag,mpu_flag);
		}
	}
	else OLED_ShowString(4,1,"  ");
}

