/**************************************************************************************************
  Filename:       hal_lcd.c
  Revised:        $Date: 2012-08-03 14:28:46 -0700 (Fri, 03 Aug 2012) $
  Revision:       $Revision: 31092 $

  Description:    This file contains the interface to the HAL LCD Service.


  Copyright 2007 - 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
/*******************************************************************************
 * 文件名称：hal_led.c
 * 功    能：OLED驱动
 *           使用硬件SPI总线驱动128*64点阵OLED液晶
 * 硬件连接：液晶模块与CC2530的硬件连接关系如下：
 *                液晶模块                       CC2530
 *                  CS                            P1.2
 *                  SDA                           P1.6
 *                  SCK                           P1.5
 *                  RESET                         P0.0
 *                  D/C#                          P2.2
 *                字库芯片
 *                  CS#                           P2.1
 *                  SCLK                          P1.5
 *                  SI                            P1.6
 *                  SO                            P1.7
 *
 * 作    者：w
 * 公    司：无锡泛太科技有限公司
 ******************************************************************************/

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "hal_lcd.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_assert.h"

#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  #include "DebugTrace.h"
#endif

/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/


/* LCD管脚宏定义 */
/********************************************************************/
#define HAL_LCD_RESET_PORT          0
#define HAL_LCD_RESET_PIN           0

#define HAL_LCD_CS_PORT             1
#define HAL_LCD_CS_PIN              2

#define HAL_LCD_CLK_PORT            1
#define HAL_LCD_CLK_PIN             5

#define HAL_LCD_MOSI_PORT           1
#define HAL_LCD_MOSI_PIN            6

#define HAL_LCD_MISO_PORT           1
#define HAL_LCD_MISO_PIN            7

#define HAL_GT20L_CS_PORT             2
#define HAL_GT20L_CS_PIN              1

#define HAL_LCD_DC_PORT             2
#define HAL_LCD_DC_PIN              2

/* SPI 配置 */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_POL_HI       0x80
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_CLOCK_PHA_1        0x40
#define HAL_SPI_TRANSFER_MSB_LAST  0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20

/*命令 数据*/
#define COMMAND             0
#define LCDDATA                1

/* LCD显示定义 */
#define LCD_MAX_LINE_COUNT          6     //最大行数
#define LCD_MAX_LINE_LENGTH         21    //每行最大显示字符数
#define LCD_MAX_BUF                 25

/* 字体设置 */
#define LCD_X_WITCH                 7
#define LCD_Y_WITCH                 127

#define FUNCTION_SET(options,OLED_DC)       HalLcd_HW_Control(options,OLED_DC)//LCD控制数据写入

/* Set Display Start Line *//* 设置显示开始行 */
#define LINE1                           0x00
#define LINE2                           0x01
#define LINE3                           0x02
#define LINE4                           0x03

/* IO设置 */
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )

/* 设置IO输出 */
#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin## = val; \
                                                      P##port##DIR |= BV(pin); )
/* 设置IO为功能端口 */
#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )

/* SPI接口控制 */
#define LCD_SPI_BEGIN()     HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  0);
#define LCD_SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  1); /* chip select */         \
}
/*SPI接口命令  数据控制*/
#define LCD_SPI_COMMAND()     HAL_IO_SET(HAL_LCD_DC_PORT,  HAL_LCD_DC_PIN,  0);
#define LCD_SPI_DATA()     HAL_IO_SET(HAL_LCD_DC_PORT,  HAL_LCD_DC_PIN,  1);
/*GT20L SPI 接口  选通*/
#define GT20L_SPI_BEGIN()     HAL_IO_SET(HAL_GT20L_CS_PORT, HAL_GT20L_CS_PIN,  0);
#define GT20L_SPI_END()     HAL_IO_SET(HAL_GT20L_CS_PORT, HAL_GT20L_CS_PIN,  1);



/* 清空接收和发送字节状态，往缓冲区里写入发送数据，等待传输完成 */
#define LCD_SPI_TX(x)         { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define LCD_SPI_WAIT_RXRDY()  { while(!(U1CSR & BV(1))); }

/* LCD复位 */
#define LCD_ACTIVATE_RESET()  HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 0);
#define LCD_RELEASE_RESET()   HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
/********************************************************************/


/* 本地函数声明 */
/********************************************************************/
#if (HAL_LCD == TRUE)
uint16 ASCII_CODE_ADDR[2];
uint8 font_buffer[32];
void HalLcd_HW_Init(void);
void HalLcd_HW_WaitUs(uint16 i);
void HalLcd_HW_SetBackLight(unsigned char Deg) ;
void HalLcd_HW_Control(uint8 cmd,uint8 OLED_DC);
void HalLcd_HW_WriteChar(uint8 line, uint8 col,uint16 text,uint8 MODE);
void HalLcd_HW_WriteLine(uint8 line, uint16 * pText,uint8 totalLength);
void halASCII_Searh_ADDR(uint16 ASCIICodeADDR,uint8 MODE);
void halGT20L_HRD_Font(uint16 * Dst, uint8 no_bytes,uint8 *buffer);
#endif
/********************************************************************/

/**************************************************************************************************
 * @fn      HalLcdInit
 *
 * @brief   Initilize LCD Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcdInit(void)
{
#if (HAL_LCD == TRUE)
// Lcd_Line1 = NULL;
  HalLcd_HW_Init();
#endif
}

/*************************************************************************************************
 *                    LCD EMULATION FUNCTIONS
 *
 * Some evaluation boards are equipped with Liquid Crystal Displays
 * (LCD) which may be used to display diagnostic information. These
 * functions provide LCD emulation, sending the diagnostic strings
 * to Z-Tool via the RS232 serial port. These functions are enabled
 * when the "LCD_SUPPORTED" compiler flag is placed in the makefile.
 *
 * Most applications update both lines (1 and 2) of the LCD whenever
 * text is posted to the device. This emulator assumes that line 1 is
 * updated first (saved locally) and the formatting and send operation
 * is triggered by receipt of line 2. Nothing will be transmitted if
 * only line 1 is updated.
 *
 *************************************************************************************************/


/**************************************************************************************************
 * @fn      HalLcdWriteString
 *
 * @brief   Write a string to the LCD
 *
 * @param   str    - pointer to the string that will be displayed
 *          option - display options
 *
 * @return  None
 **************************************************************************************************/
/*********************************************************************
 * 函数名称：HalLcdWriteString
 * 功    能：写一串字符到LCD
 * 入口参数：str    被写入的字符串
 *           option 写入选项，这里指写入的行
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcdWriteString(const char *str, uint8 option)
{
  uint16 TemP[30];
  uint8 totalLength = (uint8)osal_strlen((char *)str);
  uint8 i=0;
  for(i=0;i<totalLength ;i++)
  {
    if(i>30)
      return;
    TemP[i] = str[i];
  }
  HalLcd_HW_WriteLine(option,TemP,totalLength);
}

#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                    HARDWARE LCD
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      halLcd_ConfigIO
 *
 * @brief   Configure IO lines needed for LCD control.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
/*********************************************************************
 * 函数名称：halLcd_ConfigIO
 * 功    能：LCD显示IO配置
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void halLcd_ConfigIO(void)
{
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT, HAL_LCD_CS_PIN, 1);
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_DC_PORT,HAL_LCD_DC_PIN, 1);
  HAL_CONFIG_IO_OUTPUT(HAL_GT20L_CS_PORT,HAL_GT20L_CS_PIN, 1);
}



/**************************************************************************************************
 * @fn      halLcd_ConfigSPI
 *
 * @brief   Configure SPI lines needed for talking to LCD.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
/*********************************************************************
 * 函数名称：halLcd_ConfigSPI
 * 功    能：LCD显示SPI总线配置
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void halLcd_ConfigSPI(void)
{
  uint8 baud_exponent;
  uint8 baud_mantissa;

  PERCFG |= 0x02;   // 配置SPI为UART1,alt2

  /* 配置CLK MOSI 等IO口 */
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);


  /* 设置 SPI 速度为1MHz(系统时钟为32MHz) */
  baud_exponent = 14;
  baud_mantissa =  0;

  /* 配置SPI总线各项具体参数 */
  U1UCR  = 0x80;  //Flush 和IDLE状态
  U1CSR &= ~0xA0; // SPI 主模式
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_1 | HAL_SPI_CLOCK_POL_HI | baud_exponent;
  U1BAUD = baud_mantissa;
}


/**************************************************************************************************
 * @fn      HalLcd_HW_Init
 *
 * @brief   Initilize HW LCD Driver.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Init(void)
{
  halLcd_ConfigIO(); // 初始化LCD IO口
  halLcd_ConfigSPI();// 初始化 SPI 口

  /* 复位LCD */
  LCD_ACTIVATE_RESET();
  HalLcd_HW_WaitUs(8000);
  LCD_RELEASE_RESET();
  HalLcd_HW_WaitUs(30000);
  HalLcd_HW_WaitUs(30000);

  //Charge Pump Setting
  FUNCTION_SET(0x8d,COMMAND);
  FUNCTION_SET(0x14,COMMAND);
  FUNCTION_SET(0xaf,COMMAND);
  //Set Display Clock
  FUNCTION_SET(0xD5,COMMAND);
  FUNCTION_SET(0xF0,COMMAND);
  //Set Pre-charge Period
  FUNCTION_SET(0xD9,COMMAND);
  FUNCTION_SET(0x11,COMMAND);
  //Set VCOMH Deselect Level
  FUNCTION_SET(0xDb,COMMAND);
  FUNCTION_SET(0x0,COMMAND);
  //Set Norma Display
  FUNCTION_SET(0xa6,COMMAND);
  //Entire Display ON
  //FUNCTION_SET(0xa5,COMMAND);
  //Set Contrast Control
  FUNCTION_SET(0x81,COMMAND);
  FUNCTION_SET(0xff,COMMAND);  //1-256
  //Set Segment Re-map
  FUNCTION_SET(0xa1,COMMAND);
  //Set COM Output Scan Direction
  FUNCTION_SET(0xc8,COMMAND);
  //Set Memory Addressing Mode
  FUNCTION_SET(0x20,COMMAND);
  FUNCTION_SET(0x00,COMMAND);
  
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Control
 *
 * @brief   Write 1 command to the LCD
 *
 * @param   uint8 cmd - command to be written to the LCD
 *
 * @return  None
 **************************************************************************************************/
/*********************************************************************
 * 函数名称：HalLcd_HW_Control
 * 功    能：LCD硬件写入控制
 * 入口参数：cmd   写入的命令或者数据
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Control(uint8 cmd ,uint8 OLED_DC)
{
  LCD_SPI_BEGIN();
  if(OLED_DC==COMMAND)
  {
    LCD_SPI_COMMAND();
  }
  else
  {
    LCD_SPI_DATA();
  }

  LCD_SPI_TX(cmd);
 // LCD_SPI_WAIT_RXRDY();

  LCD_SPI_DATA();
  LCD_SPI_END();
}


/*********************************************************************
 * 函数名称：HalGT20L_TX_CMD
 * 功    能：GT20L硬件写入控制
 * 入口参数：cmd   写入数据
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalGT20L_TX_CMD(uint8 cmd )
{
  LCD_SPI_TX(cmd);
}

/*********************************************************************
 * 函数名称：HalLcd_SET_XY
 * 功    能：设置 x y 地址
 * 入口参数：x   x地址
 *           y   y地址
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_SET_XY(unsigned char x,unsigned char y)
{
  FUNCTION_SET(0x22,COMMAND);	//传送指令0x22
  FUNCTION_SET(y,COMMAND);	//要显示字符的左上角的Y轴位置
  FUNCTION_SET(LCD_X_WITCH ,COMMAND);	//要显示字符的左上角的Y轴位置
  FUNCTION_SET(0x21,COMMAND);	//传送指令0x21
  FUNCTION_SET(x,COMMAND);	//要显示字符的左上角的X轴位置
  FUNCTION_SET(LCD_Y_WITCH ,COMMAND);	//要显示字符的左上角的Y轴位置
}


/*********************************************************************
 * 函数名称：HalLcd_HW_Clear
 * 功    能：LCD硬件清屏
 * 入口参数：uint8 X_U,uint8 X_D,uint8 Y_L,uint8 Y_R
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Clear(uint8 X_L,uint8 X_R,uint8 Y_U,uint8 Y_D)
{
  uint16 i,j;
  //清屏操作
  FUNCTION_SET(0x22,COMMAND);
  FUNCTION_SET(Y_U,COMMAND);
  FUNCTION_SET(Y_D,COMMAND);
  FUNCTION_SET(0x21,COMMAND);
  FUNCTION_SET(X_L,COMMAND);
  FUNCTION_SET(X_R,COMMAND);

  for(i=0;i<(Y_D - Y_U+1);i++)
  {
    for(j=0;j<(X_R - X_L+1);j++)
      FUNCTION_SET(0x00,LCDDATA);
  }
}


/********************************************************************
 * 函数名称：HalLcd_HW_SetBackLight
 * 功    能：设置背光亮度等级
 * 入口参数：Deg  亮度等级0~127
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_SetBackLight(unsigned char Deg)
{
  LCD_SPI_BEGIN();
  FUNCTION_SET(0x81,COMMAND);
  FUNCTION_SET(Deg,COMMAND);
  LCD_SPI_END();	
}



/*********************************************************************
 * 函数名称：HalLcd_HW_WriteChar
 * 功    能：显示ASCII码字符
 * 入口参数：line  要显示的字符的行位置
 *           col   要显示的字符列位置
 *           text  要显示的ASCII码字符值
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_WriteChar(uint8 line, uint8 col,uint16 text,uint8 MODE)
{

  uint8 i;
  if(MODE==0)
  {

//      HalLcd_HW_Clear(col,col+16,line,line+2);
      halASCII_Searh_ADDR(text,0);
      halGT20L_HRD_Font(ASCII_CODE_ADDR,32,font_buffer);
      HalLcd_SET_XY(col,line);
      for(i=0;i<16;i++)
      {
        FUNCTION_SET(font_buffer[i],LCDDATA);
      }
      HalLcd_SET_XY(col,line+1);
      for(i=16;i<32;i++)
      {
        FUNCTION_SET(font_buffer[i],LCDDATA);
      }

  }
  else if(MODE==1)
  {
//      HalLcd_HW_Clear(col,col+8,line,line+1);
      halASCII_Searh_ADDR(text,1);
      halGT20L_HRD_Font(ASCII_CODE_ADDR,8,font_buffer);
      HalLcd_SET_XY(col,line);
      for(i=0;i<8;i++)
      {
        FUNCTION_SET(font_buffer[i],LCDDATA);
      }
  }
  else if(MODE==2)
    {
//      HalLcd_HW_Clear(col,col+8,line,line+1);
      halASCII_Searh_ADDR(text ,2);
      halGT20L_HRD_Font(ASCII_CODE_ADDR,8,font_buffer);
      HalLcd_SET_XY(col,line);
      for(i=0;i<8;i++)
      {
        FUNCTION_SET(font_buffer[i],LCDDATA);
      }
    }
  else if(MODE==3)
    {
      HalLcd_HW_Clear(col,col+8,line,line+2);
 //     halASCII_Searh_ADDR(text,0x03);
      halGT20L_HRD_Font(ASCII_CODE_ADDR,16,font_buffer);
      HalLcd_SET_XY(col,line);
      for(i=0;i<8;i++)
      {
        FUNCTION_SET(font_buffer[i],LCDDATA);
      }
      HalLcd_SET_XY(col,line+1);
      for(i=8;i<16;i++)
      {
        FUNCTION_SET(font_buffer[i],LCDDATA);
      }
    }

}


/*********************************************************************
 * 函数名称：HalLcd_HW_WriteLine
 * 功    能：LCD显示一行字符串
 * 入口参数：line  要显示的字符的行位置
 *           pText 要显示的字符串
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_WriteLine(uint8 line, uint16 * pText,uint8 totalLength)
{
  uint8 num,countDOT=0,MODE=0x05;
  for(num=0;num<totalLength;num++)
  {
    if((pText[num])>0xA1)
    {
      MODE=0x00;
    }
  }
  if(MODE==0x00)
  {
    countDOT=0;
    for(num=0;num<totalLength;)
    {
      if((LCD_Y_WITCH-countDOT) < 16)
        return;
      if((pText[num])>0xA1)
      {
        HalLcd_HW_WriteChar(countDOT,line ,((pText[num] << 8)+pText[num+1]),0x00);
        countDOT=countDOT+16;
        num=num+2;
      }
       else
       {
        HalLcd_HW_WriteChar(countDOT,line , pText[num],0x03);
        countDOT=countDOT+8;
        num++;
       }
     }
  }
  else
  {
    countDOT=0;
    for(num=0;num<totalLength;num++)
    {
      if((LCD_Y_WITCH-countDOT) < 8)
        return;
      HalLcd_HW_WriteChar(line, countDOT, pText[num],0x01);
      countDOT=countDOT+7;

    }
  }
}


/*********************************************************************
 * 函数名称：HalLcd_HW_WaitUs
 * 功    能：LCD延时函数
 * 入口参数：microSecs  要延时的微妙数
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOP为1微妙 */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}



void halASCII_Searh_ADDR(uint16 ASCIICodeADDR,uint8 MODE)
{
  uint16 ASCII_Code_Addr = 0;
  uint8 ASCIICode[2];
  ASCIICode[0]=(uint8)(ASCIICodeADDR & 0xFF);
  ASCIICode[1]=(uint8)((ASCIICodeADDR>>8) & 0xFF);
  switch(MODE)
  {
  case 0 :   //8*16点阵国标扩展字符
        if((ASCIICode[1]==0XA9)&&(ASCIICode[0]>=0xA1))
        {
               ASCII_Code_Addr = ((((uint16)ASCIICode[0]) - (uint16)0xA1 + 0X011A) * 32);
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x00;
        }
        else if((ASCIICode[1]>=0XA1)&&(ASCIICode[1]<=0XA3)&&(ASCIICode[0]>=0xA1))
        {
               ASCII_Code_Addr =(((uint16)(ASCIICode[1])-0xa1)*94 + ((uint16)(ASCIICode[1])-0xa1) )*32;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x00;
        }
        else if(  (ASCIICodeADDR >= 0xB0A1)&&(ASCIICodeADDR <= 0XBE2E)   )
        {
               ASCII_Code_Addr =(((uint16)(ASCIICode[1])-0xa1)*94 + ((uint16)(ASCIICode[1])-0xa1)+846 )*32;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x00;
        }
        else if((ASCIICodeADDR > 0xBE2E)&&(ASCIICodeADDR <= 0XD41A) )
        {
               ASCII_Code_Addr =(((uint16)(ASCIICode[1])-0xa1)*94 + ((uint16)(ASCIICode[1])-0xa1) +846)*32;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x01;
        }
        else if((ASCIICodeADDR > 0xD41A)&&(ASCIICodeADDR <= 0XEA06) )
        {
               ASCII_Code_Addr =(((uint16)(ASCIICode[1])-0xa1)*94 + ((uint16)(ASCIICode[1])-0xa1) +846)*32;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x02;
        }
        else if((ASCIICodeADDR > 0XEA06)&&(ASCIICodeADDR <= 0XF7Fe) )
        {
               ASCII_Code_Addr =(((uint16)(ASCIICode[1])-0xa1)*94 + ((uint16)(ASCIICode[1])-0xa1) +846)*32;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x03;
        }

        break;
  case 1:   //5*7 点 ASCII字符
        if( (ASCIICode[0]>=0x20) && (ASCIICode[0]<=0x7E) )
                //ASCII_Code_Addr = (ASCIICode[0] - 0x20) * 8 + 0x3BfC0;
               ASCII_Code_Addr = ((((uint16)ASCIICode[0]) - (uint16)0x20) * 8) +0xbfC0;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x03;
        break;

  case 2:   //7*8 点 ASCII字符
        if( (ASCIICode[0]>=0x20) && (ASCIICode[0]<=0x7E) )
                //ASCII_Code_Addr = (ASCIICode[0] - 0x20) * 8 + 0x66C0;
               ASCII_Code_Addr = ((((uint16)ASCIICode[0]) - (uint16)0x20) * 8) +0x66C0;
              ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x00;
        break;

  case 3:   //8*16 点 ASCII字符
        if( (ASCIICode[0]>=0x20) && (ASCIICode[0]<=0x7E) )
        {
               ASCII_Code_Addr = ((((uint16)ASCIICode[0]) - (uint16)0x20) * 16) +0xcf80;
               ASCII_CODE_ADDR[0]=ASCII_Code_Addr ;
               ASCII_CODE_ADDR[1]=0x03;
        }
        break;

  case 4:   //8*16 点 ASCIIc 粗体字符
        if( (ASCIICode[0]>=0x20) && (ASCIICode[0]<=0x7E) )
                ASCII_Code_Addr = (ASCIICode[0] - 0x20) * 8 + 0x3bf80;
        break;
  }

}

/*********************************************************************
 * 函数名称：halGT20L_HRD_Font
 * 功    能：GT23L读取字符码值函数
 * 入口参数： Dst 字库地址, no_bytes 一个字符需要的码值 字节数 ,  *buffer 码值
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halGT20L_HRD_Font(uint16 * Dst, uint8 no_bytes,uint8 *buffer)
{
    unsigned char i = 0;
    GT20L_SPI_BEGIN();                                // enable device
    HalGT20L_TX_CMD(0x0B);                       // read command
    HalGT20L_TX_CMD(Dst[1]);  // send 3 address bytes
    HalGT20L_TX_CMD(((Dst[0]) >> 8));
    HalGT20L_TX_CMD(Dst[0] & 0xFF);
    HalGT20L_TX_CMD(0xFF);                       //dummy byte
    for (i = 0; i < no_bytes; i++)              // read until no_bytes is reached
    {
      HalGT20L_TX_CMD(0xFF);
      buffer[i] = U1DBUF;       // receive byte and store at address 80H - FFH
    }
    GT20L_SPI_END();                                // disable device
}

void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
#if (HAL_LCD == TRUE)
  uint8 tmpLen;
  uint8 buf[LCD_MAX_BUF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  HalLcdWriteString( (char*)buf, line );		
#endif
}

#endif


/**************************************************************************************************
**************************************************************************************************/



