//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F429IGT6,����ԭ��Apollo STM32F4/F7������,��Ƶ180MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V        //��Դ
//      GND          ��          GND            //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//     DB0~DB15      ��        PE0~PE15         //Һ����16λ���������ź�
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 				  STM32��Ƭ�� 
//     LCD_WR        ��          PC7           //Һ����д���ݿ����ź�
//     LCD_RD        ��          PC6           //Һ���������ݿ����ź�
//     LCD_RS        ��          PC8           //Һ��������/��������ź�
//     LCD_RST       ��          PC10          //Һ������λ�����ź�
//     LCD_CS        ��          PC9           //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//      T_IRQ        ��          PH10        //�����������ж��ź�
//      T_DO         ��          PH11          //������SPI���߶��ź�
//      T_DIN        ��          PH12         //������SPI����д�ź�
//      T_CS         ��          PH13         //������Ƭѡ�����ź�
//      T_CLK        ��          PH9         //������SPI����ʱ���ź�
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/		
#ifndef __LCD_H
#define __LCD_H		
#include "sys.h"	 
#include "stdlib.h"
#include "stm32f4xx.h"

//#include "main.h"
//LCD��Ҫ������
typedef struct  
{										    
	uint16_t width;			//LCD ���
	uint16_t height;			//LCD �߶�
	uint16_t id;				//LCD ID
	uint8_t  dir;			//���������������ƣ�0��������1��������	
	uint16_t	 wramcmd;		//��ʼдgramָ��
	uint16_t  rramcmd;   //��ʼ��gramָ��
	uint16_t  setxcmd;		//����x����ָ��
	uint16_t  setycmd;		//����y����ָ��	 
}_lcd_dev; 	

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
/////////////////////////////////////�û�������///////////////////////////////////	 
#define USE_HORIZONTAL  	 0 //����Һ����˳ʱ����ת���� 	0-0����ת��1-90����ת��2-180����ת��3-270����ת
#define USE_VERTICAL         1
#define USE_180DEGREES       2
#define USE_270DEGREES       3

#define LCD_USE8BIT_MODEL   1	//�������������Ƿ�ʹ��8λģʽ 0,ʹ��16λģʽ.1,ʹ��8λģʽ

//////////////////////////////////////////////////////////////////////////////////	  
//����LCD�ĳߴ�
#define LCD_W 320
#define LCD_H 480

//TFTLCD������Ҫ���õĺ���		   
extern uint16_t  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern uint16_t  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

////////////////////////////////////////////////////////////////////
//-----------------LCD�˿ڶ���---------------- 
//#define CS    7          //Ƭѡ����             PC9
//#define RS    11          //�Ĵ���/����ѡ������  PC8
//#define RST   6             //��λ����             PC10
//#define WR    5          //д��������           PC7
//#define RD    4          //����������           PC6
 		 
//#define LCD_CS  //PCout(CS) 
//#define LCD_RS  PCout(RS) 
//#define LCD_RST PCout(RST)
//#define LCD_WR  PCout(WR)
//#define LCD_RD  PCout(RD)

/*
 * @brief GPIO interface parameters
 */
#define GPIO_PIN_LCD_D0                GPIO_PIN_14
#define GPIO_PORT_LCD_D0               GPIOD
#define GPIO_CLK_EN_LCD_D0()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_D1                GPIO_PIN_15
#define GPIO_PORT_LCD_D1               GPIOD
#define GPIO_CLK_EN_LCD_D1()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_D2                GPIO_PIN_0
#define GPIO_PORT_LCD_D2               GPIOD
#define GPIO_CLK_EN_LCD_D2()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_D3                GPIO_PIN_1
#define GPIO_PORT_LCD_D3               GPIOD
#define GPIO_CLK_EN_LCD_D3()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_D4                GPIO_PIN_7
#define GPIO_PORT_LCD_D4               GPIOE
#define GPIO_CLK_EN_LCD_D4()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PIN_LCD_D5                GPIO_PIN_8
#define GPIO_PORT_LCD_D5               GPIOE
#define GPIO_CLK_EN_LCD_D5()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PIN_LCD_D6                GPIO_PIN_9
#define GPIO_PORT_LCD_D6               GPIOE
#define GPIO_CLK_EN_LCD_D6()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PIN_LCD_D7                GPIO_PIN_10
#define GPIO_PORT_LCD_D7               GPIOE
#define GPIO_CLK_EN_LCD_D7()           __HAL_RCC_GPIOE_CLK_ENABLE()

#define GPIO_PIN_LCD_RST               GPIO_PIN_6
#define GPIO_PORT_LCD_RST              GPIOD
#define GPIO_CLK_EN_LCD_RST()          __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_CS                GPIO_PIN_7
#define GPIO_PORT_LCD_CS               GPIOD
#define GPIO_CLK_EN_LCD_CS()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_RS                GPIO_PIN_11
#define GPIO_PORT_LCD_RS               GPIOD
#define GPIO_CLK_EN_LCD_RS()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_WR                GPIO_PIN_5
#define GPIO_PORT_LCD_WR               GPIOD
#define GPIO_CLK_EN_LCD_WR()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PIN_LCD_RD                GPIO_PIN_4
#define GPIO_PORT_LCD_RD               GPIOD
#define GPIO_CLK_EN_LCD_RD()           __HAL_RCC_GPIOD_CLK_ENABLE()

#define	LCD_RST_SET                    HAL_GPIO_WritePin(GPIO_PORT_LCD_RST, GPIO_PIN_LCD_RST, GPIO_PIN_SET)
#define	LCD_RST_CLR                    HAL_GPIO_WritePin(GPIO_PORT_LCD_RST, GPIO_PIN_LCD_RST, GPIO_PIN_RESET)
#define	LCD_CS_SET                     HAL_GPIO_WritePin(GPIO_PORT_LCD_CS, GPIO_PIN_LCD_CS, GPIO_PIN_SET)
#define	LCD_CS_CLR                     HAL_GPIO_WritePin(GPIO_PORT_LCD_CS, GPIO_PIN_LCD_CS, GPIO_PIN_RESET)
#define	LCD_RS_SET                     HAL_GPIO_WritePin(GPIO_PORT_LCD_RS, GPIO_PIN_LCD_RS, GPIO_PIN_SET)
#define	LCD_RS_CLR                     HAL_GPIO_WritePin(GPIO_PORT_LCD_RS, GPIO_PIN_LCD_RS, GPIO_PIN_RESET)
#define	LCD_WR_SET                     HAL_GPIO_WritePin(GPIO_PORT_LCD_WR, GPIO_PIN_LCD_WR, GPIO_PIN_SET)
#define	LCD_WR_CLR	                   HAL_GPIO_WritePin(GPIO_PORT_LCD_WR, GPIO_PIN_LCD_WR, GPIO_PIN_RESET)
#define	LCD_RD_SET                     HAL_GPIO_WritePin(GPIO_PORT_LCD_RD, GPIO_PIN_LCD_RD, GPIO_PIN_SET)
#define	LCD_RD_CLR                     HAL_GPIO_WritePin(GPIO_PORT_LCD_RD, GPIO_PIN_LCD_RD, GPIO_PIN_RESET)
				

//#define DATAOUT(x) GPIOE->ODR = (GPIOE->ODR&0XF87F)| ((x&0XF0)<<7);GPIOD->ODR = (GPIOD->ODR&0X3FFC)|(x&0X1D);
//#define DATAIN     GPIOE->IDR&0XF87F | GPIOD->IDR&0X2FFC ;  

//������ɫ
#define WHITE       0xFFFF
#define BLACK       0x0000	  
#define BLUE        0x001F  
#define BRED        0XF81F
#define GRED 	    0XFFE0
#define GBLUE	    0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 //��ɫ
#define BRRED 			0XFC07 //�غ�ɫ
#define GRAY  			0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	0X841F //ǳ��ɫ
#define LIGHTGRAY     0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 		0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE      	0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE          0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
	    															  
void LCD_Init(void);
void LCD_write(uint16_t VAL);
uint16_t LCD_read(void);
void LCD_Clear(uint16_t Color);	 
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_DrawPoint(uint16_t x,uint16_t y);//����
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y); //����	   
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);
uint16_t LCD_RD_DATA(void);//��ȡLCD����								    
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WR_REG(uint16_t data);
void LCD_WR_DATA(uint16_t data);
void LCD_ReadReg(uint16_t LCD_Reg,uint8_t *Rval,int n);
void LCD_WriteRAM_Prepare(void);
void LCD_ReadRAM_Prepare(void);   
void Lcd_WriteData_16Bit(uint16_t Data);
uint16_t Lcd_ReadData_16Bit(void);

void LCD_direction(uint8_t direction );
uint16_t LCD_Read_ID(void);

void DATAOUT(uint16_t x);
uint16_t DATAIN(void);
				  		 
#endif  
	 
	 



