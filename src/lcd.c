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
#include "lcd.h"
#include "stdlib.h"
//#include "delay.h"	 
_lcd_dev lcddev;
uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
uint16_t DeviceCode;

/*
 * @name _ delay_us.
 * @brief  halts program from microseconds accurate delay.
 * @param  usec: delay time [us].
 * @retval none.
 */
extern void
delay_us (uint32_t usec);

void
DATAOUT (uint16_t x)
{
  uint16_t temp = 0;

  temp = (x>>8) & (uint16_t)0x0001;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D0, GPIO_PIN_LCD_D0, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0002) >> 1;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D1, GPIO_PIN_LCD_D1, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0004) >> 2;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D2, GPIO_PIN_LCD_D2, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0008) >> 3;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D3, GPIO_PIN_LCD_D3, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0010) >> 4;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D4, GPIO_PIN_LCD_D4, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0020) >> 5;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D5, GPIO_PIN_LCD_D5, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0040) >> 6;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D6, GPIO_PIN_LCD_D6, (GPIO_PinState)temp);
  temp = ((x>>8) & (uint16_t)0x0080) >> 7;
  HAL_GPIO_WritePin(GPIO_PORT_LCD_D7, GPIO_PIN_LCD_D7, (GPIO_PinState)temp);

  return;
}

uint16_t
DATAIN (void)
{
  uint16_t temp = 0;

  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D7, GPIO_PIN_LCD_D7) << 7;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D6, GPIO_PIN_LCD_D6) << 6;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D5, GPIO_PIN_LCD_D5) << 5;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D4, GPIO_PIN_LCD_D4) << 4;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D3, GPIO_PIN_LCD_D3) << 3;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D2, GPIO_PIN_LCD_D2) << 2;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D1, GPIO_PIN_LCD_D1) << 1;
  temp |= HAL_GPIO_ReadPin (GPIO_PORT_LCD_D0, GPIO_PIN_LCD_D0);

  return (temp << 8);
}

void LCD_write(uint16_t VAL)
{
	LCD_CS_CLR;  
	DATAOUT(VAL);
	LCD_WR_CLR;
	LCD_WR_SET; 
	LCD_CS_SET;
}

uint16_t LCD_read(void)
{
	uint16_t data;
	LCD_CS_CLR;
	LCD_RD_CLR;
	delay_us(1);//��ʱ1us
	data = DATAIN();
	LCD_RD_SET;
	LCD_CS_SET;
	return data;
}

/*****************************************************************************
 * @name       :void LCD_WR_REG(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(uint16_t data)
{ 
   LCD_RS_CLR;     
	 #if LCD_USE8BIT_MODEL
	 LCD_write(data<<8);
	 #else
	 LCD_write(data);
 #endif
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(uint16_t data)
{
	 LCD_RS_SET;
	 #if LCD_USE8BIT_MODEL
	 LCD_write(data<<8);
	 #else
	 LCD_write(data);
	 #endif
}

/*****************************************************************************
 * @name       :uint16_t LCD_RD_DATA(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/
uint16_t LCD_RD_DATA(void)
{
	LCD_RS_SET; 
	#if LCD_USE8BIT_MODEL
	return (LCD_read()>>8);
	#else
	return LCD_read();
	#endif
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

/*****************************************************************************
 * @name       :uint16_t LCD_ReadReg(uint16_t LCD_Reg)
 * @date       :2018-11-13 
 * @function   :read value from specially registers
 * @parameters :LCD_Reg:Register address
 * @retvalue   :read value
******************************************************************************/
void LCD_ReadReg(uint16_t LCD_Reg,uint8_t *Rval,int n)
{
//	LCD_WR_REG(LCD_Reg);
//	GPIOD->MODER&=0x3FFC;
//	GPIOE->MODER&=0xF87F;
//	GPIOE->BSRR=0X00FFFF0; //PE0-15 ����
//	GPIOD->BSRR=0X00FFFF0; //PE0-15 ����
//	while(n--)
//	{
//		*(Rval++) = LCD_RD_DATA();
//	}
//
//	GPIOD->MODER|=0xC003;
//	GPIOE->MODER=0x0780;

  return;
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void LCD_ReadRAM_Prepare(void)
 * @date       :2018-11-13 
 * @function   :Read GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(uint16_t Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(uint16_t Data)
{	
   LCD_RS_SET; 
	 #if LCD_USE8BIT_MODEL
		LCD_CS_CLR;
		DATAOUT(Data);
		LCD_WR_CLR; 
		LCD_WR_SET;
		DATAOUT(Data<<8);
		LCD_WR_CLR; 
		LCD_WR_SET;
		LCD_CS_SET;
 //  LCD_write(Data&0xFF00);
//	 LCD_write(Data<<8);
	 #else
	 LCD_write(Data);
	 #endif
}

/*****************************************************************************
 * @name       :uint16_t Lcd_ReadData_16Bit(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/	
uint16_t Lcd_ReadData_16Bit(void)
{
	uint16_t r;
	#if LCD_USE8BIT_MODEL	
	uint16_t g;	
	#endif
	LCD_RS_SET;
	LCD_CS_CLR;
	
	//dummy data
	LCD_RD_CLR;
	delay_us(1);//��ʱ1us
	r = DATAIN();
	LCD_RD_SET;
	
	LCD_RD_CLR;
	delay_us(1);//��ʱ1us
	r = DATAIN();
	LCD_RD_SET;
		
	#if LCD_USE8BIT_MODEL	
	//blue data
	LCD_RD_CLR;
	delay_us(1);//��ʱ1us
	g = DATAIN();
	LCD_RD_SET;
	r &= 0xFF00;
	r |= ((g>>8)&0x00FF);
	#endif
	LCD_CS_SET;
	return r;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(uint16_t x,uint16_t y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//���ù��λ�� 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}



/*****************************************************************************
 * @name       :void LCD_Clear(uint16_t Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(uint16_t Color)
{
  unsigned int i;//,m;
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	for(i=0;i<lcddev.height*lcddev.width;i++)
	{
 //   for(m=0;m<lcddev.width;m++)
  //  {
			Lcd_WriteData_16Bit(Color);
	//	}
	}
} 

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_Initure;
    
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_Initure.Mode= GPIO_MODE_OUTPUT_PP;
  GPIO_Initure.Pull= GPIO_PULLUP;
  GPIO_Initure.Speed= GPIO_SPEED_HIGH;

  GPIO_Initure.Pin = GPIO_PIN_LCD_RST | GPIO_PIN_LCD_CS | GPIO_PIN_LCD_RS | GPIO_PIN_LCD_WR | GPIO_PIN_LCD_RD
                     | GPIO_PIN_LCD_D0 | GPIO_PIN_LCD_D1 | GPIO_PIN_LCD_D2 | GPIO_PIN_LCD_D3;
  HAL_GPIO_Init(GPIOD, &GPIO_Initure);
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_LCD_RST | GPIO_PIN_LCD_CS | GPIO_PIN_LCD_RS | GPIO_PIN_LCD_WR | GPIO_PIN_LCD_RD
                            | GPIO_PIN_LCD_D0 | GPIO_PIN_LCD_D1 | GPIO_PIN_LCD_D2 | GPIO_PIN_LCD_D3, GPIO_PIN_SET);

  GPIO_Initure.Pin = GPIO_PIN_LCD_D4 | GPIO_PIN_LCD_D5 | GPIO_PIN_LCD_D6 | GPIO_PIN_LCD_D7;
  HAL_GPIO_Init (GPIOE, &GPIO_Initure);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_LCD_D4 | GPIO_PIN_LCD_D5 | GPIO_PIN_LCD_D6 | GPIO_PIN_LCD_D7, GPIO_PIN_SET);

  return;
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	HAL_Delay(10);	
	LCD_RST_SET;
	HAL_Delay(200);
}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
  LCD_GPIOInit();//LCD GPIO��ʼ��
 	LCD_RESET(); //LCD ��λ
//************* ILI9486��ʼ��**********//	
	LCD_WR_REG(0XF1);
	LCD_WR_DATA(0x36);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x3C);
	LCD_WR_DATA(0X0F);
	LCD_WR_DATA(0x8F);
	LCD_WR_REG(0XF2);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0xA3);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0XB2);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0xFF);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0XF8);
	LCD_WR_DATA(0x21);
	LCD_WR_DATA(0x04);
	LCD_WR_REG(0XF9);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x08);
	LCD_WR_REG(0x36);
	LCD_WR_DATA(0x08);
	LCD_WR_REG(0xB4);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA(0x41);
	LCD_WR_REG(0xC5);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x91);
	LCD_WR_DATA(0x80);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x1F);
	LCD_WR_DATA(0x1C);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x48);
	LCD_WR_DATA(0x98);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x32);
	LCD_WR_DATA(0x2E);
	LCD_WR_DATA(0x0B);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x47);
	LCD_WR_DATA(0x75);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x24);
	LCD_WR_DATA(0x20);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x55);
	LCD_WR_REG(0x11);
	LCD_WR_REG(0x36);
	LCD_WR_DATA(0x28);
	HAL_Delay (120);
	LCD_WR_REG(0x29);

  LCD_direction(USE_HORIZONTAL);//����LCD��ʾ����
//	LCD_LED=1;//��������	 
	LCD_Clear(GBLUE);//��ȫ����ɫ
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//��ʼд��GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(uint8_t direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(uint8_t direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<6)|(1<<3));//0 degree MY=0,MX=0,MV=0,ML=0,BGR=1,MH=0
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<4)|(1<<5));//90 degree MY=0,MX=1,MV=1,ML=1,BGR=1,MH=0
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7));//180 degree MY=1,MX=1,MV=0,ML=0,BGR=1,MH=0
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6)|(1<<7));//270 degree MY=1,MX=0,MV=1,ML=0,BGR=1,MH=0
		break;	
		default:break;
	}		
}	 

/*****************************************************************************
 * @name       :uint16_t LCD_Read_ID(void)
 * @date       :2018-11-13 
 * @function   :Read ID
 * @parameters :None
 * @retvalue   :ID value
******************************************************************************/ 
uint16_t LCD_Read_ID(void)
{
	uint8_t val[4] = {0};
	LCD_ReadReg(0xD3,val,4);
	return (val[2]<<8)|val[3];
}
/*****************************************************************************
 * @name       :uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
 * @date       :2018-11-13 
 * @function   :Read a pixel color value at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :the read color value
******************************************************************************/	
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	uint16_t color;
	if(x>=lcddev.width||y>=lcddev.height)
	{
		return 0;	//�����˷�Χ,ֱ�ӷ���	
	}
	LCD_SetCursor(x,y);//���ù��λ�� 
	LCD_ReadRAM_Prepare();
//	GPIOE->MODER=0;               //����
//	GPIOE->BSRR=0X0780; //PE0-15 ����
//	GPIOD->MODER=0;               //����
//	GPIOD->BSRR=0XC003; //PE0-15 ����
		GPIOD->MODER&=0x3FFC;
	GPIOE->MODER&=0xF87F;
	GPIOE->BSRR=0X00FFFF0; //PE0-15 ����
	GPIOD->BSRR=0X00FFFF0; //PE0-15 ����
	color = Lcd_ReadData_16Bit();
	GPIOD->MODER|=0xC003; 
	GPIOE->MODER=0x0780;
//	GPIOE->MODER=0X55555555;      //���
//	GPIOE->BSRR=0X0000FFFF; //PE0-15 ����
//	
//	GPIOE->MODER=0;               //����
//	GPIOE->BSRR=0X0780; //PE0-15 ����
//	GPIOD->MODER=0;               //����
//	GPIOD->BSRR=0XC003; //PE0-15 ����
	return color;
}


