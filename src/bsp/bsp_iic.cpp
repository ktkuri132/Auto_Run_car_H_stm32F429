//
// Created by 34575 on 25-6-4.
//

#include <bsp/config.h>
#include <stdio.h>
using namespace System;
using namespace Dirver;

//初始化IIC
void Soft_IIC::Soft_IIC_Init()
{
	this->Soft_IIC_SCL(1);
	this->Soft_IIC_SDA(1);
}

//产生IIC起始信号
void Soft_IIC::Soft_IIC_Start()
{
	this->Soft_SDA_OUT();     //sda线输出
	this->Soft_IIC_SDA(1);
	this->Soft_IIC_SCL(1);
	// delay_us(4);
	this->Soft_IIC_SDA(0);//START:when CLK is high,DATA change form high to low
	// this->delay_us(1);
	this->Soft_IIC_SCL(0);//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void Soft_IIC::Soft_IIC_Stop()
{
	this->Soft_SDA_OUT();//sda线输出
	this->Soft_IIC_SCL(0);
	this->Soft_IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	// delay_us(4);
	 this->Soft_IIC_SCL(1);
	 this->Soft_IIC_SDA(1);//发送I2C总线结束信号
	// delay_us(4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t Soft_IIC::Soft_IIC_Wait_Ack()
{
	uint8_t ucErrTime=0;
	this->Soft_SDA_IN();      //SDA设置为输入
	this->Soft_IIC_SDA(1);
	this->delay_us(1);
	this->Soft_IIC_SCL(1);
	this->delay_us(1);
	while(this->Soft_READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Soft_IIC_Stop();
			return 1;
		}
	}
	this->Soft_IIC_SCL(0);//时钟输出0
	return 0;
}
//产生ACK应答
void Soft_IIC::Soft_IIC_Ack()
{
	this->Soft_IIC_SCL(0);
	this->Soft_SDA_OUT();
	this->Soft_IIC_SDA(0);
	// this->delay_us(1);
	this->Soft_IIC_SCL(1);
	// delay_us(2);
	this->Soft_IIC_SCL(0);
	// this->Soft_IIC_SDA(1);	// 释放SDA线
}
//不产生ACK应答
void Soft_IIC::Soft_IIC_NAck()
{
	this->Soft_IIC_SCL(0);
	this->Soft_SDA_OUT();
	this->Soft_IIC_SDA(1);
	// this->delay_us(1);
	this->Soft_IIC_SCL(1);
	// delay_us(2);
	this->Soft_IIC_SCL(0);
	// this->Soft_IIC_SDA(1);	// 释放SDA线
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void Soft_IIC::Soft_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	this->Soft_SDA_OUT();
    this->Soft_IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        this->Soft_IIC_SDA((txd&0x80)>>7);
        txd<<=1;
		// this->delay_us(1);
		this->Soft_IIC_SCL(1);
		// this->delay_us(1);
		this->Soft_IIC_SCL(0);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t Soft_IIC::Soft_IIC_Receive_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	this->Soft_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        this->Soft_IIC_SCL(0);
        // this->delay_us(1);
		this->Soft_IIC_SCL(1);
        receive<<=1;
        if(this->Soft_READ_SDA())receive++;
		// delay_us(1);
    }
    if (!ack)
        Soft_IIC_NAck();//发送nACK
    else
        Soft_IIC_Ack(); //发送ACK
    return receive;
}

uint8_t Soft_IIC::Soft_IIC_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    Soft_IIC_Start();
	Soft_IIC_Send_Byte(addr|0);//发送器件地址+写命令
	if(Soft_IIC_Wait_Ack())	//等待应答
	{
		Soft_IIC_Stop();
		return 1;
	}
    Soft_IIC_Send_Byte(reg);	//写寄存器地址
    Soft_IIC_Wait_Ack();		//等待应答
	Soft_IIC_Send_Byte(data);//发送数据
	if(Soft_IIC_Wait_Ack())	//等待ACK
	{
		Soft_IIC_Stop();
		return 1;
	}
    Soft_IIC_Stop();
	return 0;
}

uint8_t Soft_IIC::Soft_IIC_Read_Byte(uint8_t addr,uint8_t reg)
{
	uint8_t res;
    Soft_IIC_Start();
	Soft_IIC_Send_Byte(addr|0);//发送器件地址+写命令
	Soft_IIC_Wait_Ack();		//等待应答
    Soft_IIC_Send_Byte(reg);	//写寄存器地址
    Soft_IIC_Wait_Ack();		//等待应答
    Soft_IIC_Start();
	Soft_IIC_Send_Byte(addr|1);//发送器件地址+读命令
    Soft_IIC_Wait_Ack();		//等待应答
	res=Soft_IIC_Receive_Byte(0);//读取数据,发送nACK
    Soft_IIC_Stop();			//产生一个停止条件
	return res;
}

uint8_t Soft_IIC::Soft_IIC_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i;
    Soft_IIC_Start();
	Soft_IIC_Send_Byte(addr|0);//发送器件地址+写命令
	if(Soft_IIC_Wait_Ack())	//等待应答
	{
		Soft_IIC_Stop();
		return 1;
	}
    Soft_IIC_Send_Byte(reg);	//写寄存器地址
    Soft_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		Soft_IIC_Send_Byte(buf[i]);	//发送数据
		if(Soft_IIC_Wait_Ack())		//等待ACK
		{
			Soft_IIC_Stop();
			return 1;
		}
	}
    Soft_IIC_Stop();
	return 0;
}

uint8_t Soft_IIC::Soft_IIC_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
 	Soft_IIC_Start();
	Soft_IIC_Send_Byte(addr|0);//发送器件地址+写命令
	if(Soft_IIC_Wait_Ack())	//等待应答
	{;
		printf("IIC Device :%d Not Found\n", addr);
		Soft_IIC_Stop();
		return 1;
	}
    Soft_IIC_Send_Byte(reg);	//写寄存器地址
    Soft_IIC_Wait_Ack();		//等待应答
    Soft_IIC_Start();
	Soft_IIC_Send_Byte(addr|1);//发送器件地址+读命令
    Soft_IIC_Wait_Ack();		//等待应答
	while(len)
	{
		if(len==1)*buf=Soft_IIC_Receive_Byte(0);//读数据,发送nACK
		else *buf=Soft_IIC_Receive_Byte(1);		//读数据,发送ACK
		len--;
		buf++;
	}
    Soft_IIC_Stop();	//产生一个停止条件
	return 0;
}



