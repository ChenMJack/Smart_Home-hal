

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"  // 然后包含这个

#include "driver_uart.h"
#include "driver_lcd.h"
#include "driver_timer.h"
#include "usart.h"
#include "driver_oled.h"


#include <stdio.h>

#define DEBUG_UART_TIMEOUT 500

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef * g_HDebugUART = &huart1;



/**********************************************************************
 * 函数名称： UART_Init
 * 功能描述： UART初始化函数
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 成功, 其他值 - 失败
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/05	     V1.0	  韦东山	      创建
 ***********************************************************************/
void UART_Init(void)
{
}



int fputc(int c, FILE *f)
{
    (void)f;
    if (HAL_UART_Transmit(g_HDebugUART, (uint8_t *)&c, 1, HAL_MAX_DELAY) == HAL_OK)
    {
        return c;
    }
    else
    {
        return EOF;
    }
}

int fgetc(FILE *f)
{
    uint8_t ch = 0;
    (void)f;

	 /* Clear the Overrun flag just before receiving the first character */
    __HAL_UART_CLEAR_OREFLAG(g_HDebugUART);
	
    if (HAL_UART_Receive(g_HDebugUART, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
    {
        HAL_UART_Transmit(g_HDebugUART, &ch, 1, HAL_MAX_DELAY); // 回显接收到的字符
        return ch;
    }
    else
    {
        return EOF;
    }
}





/*
************************************************************
*	函数名称：	Usart_SendString
*
*	函数功能：	串口数据发送
*
*	入口参数：	USARTx：串口组
*				str：要发送的数据
*				len：数据长度
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/

void Usart_SendString(UART_HandleTypeDef *huart, unsigned char *str, unsigned short len)
{
    HAL_UART_Transmit(huart, str, len, HAL_MAX_DELAY); // 发送所有字节，阻塞直到发送完成
}
/**********************************************************************
 * 函数名称： UART_Test
 * 功能描述： UART测试程序
 * 输入参数： 无
 * 输出参数： 无
 *            无
 * 返 回 值： 0 - 成功, 其他值 - 失败
 * 修改日期        版本号     修改人        修改内容
 * -----------------------------------------------
 * 2023/08/05        V1.0     韦东山       创建
 ***********************************************************************/
//void UART_Test(void)
//{    
//    char c;
//    OLED_Clear();
//    UART_Init();
//	LCD_ClearLine(0,0);
//    LCD_PrintString(0, 0, "Use Serial Tool on PC to test UART");

//    printf("100ask UART test:\n\r");
//    while (1)
//    {
//        c = fgetc(NULL);
//        printf("\n\rget %c\n\r", c);
//    }
//}


