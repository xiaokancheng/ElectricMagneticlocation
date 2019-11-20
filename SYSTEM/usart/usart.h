#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	

//V1.0修改说明 
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef UART1_Handler; //UART句柄

#define RXBUFFERSIZE   1 //缓存大小
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer

    /*虚拟示波器*/
//********************
#define uart_putbuff Usart_SendArray
extern int16_t Val_Osc[8];
void Virtual_Osc(uint8_t n,int16_t Val_Osc1, int16_t Val_Osc2, int16_t Val_Osc3, int16_t Val_Osc4, int16_t Val_Osc5,int16_t Val_Osc6,int16_t Val_Osc7,int16_t Val_Osc8);
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch );
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num);
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize);
//********************

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);


#endif
