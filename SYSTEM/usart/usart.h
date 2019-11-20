#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	

//V1.0�޸�˵�� 
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern UART_HandleTypeDef UART1_Handler; //UART���

#define RXBUFFERSIZE   1 //�����С
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer

    /*����ʾ����*/
//********************
#define uart_putbuff Usart_SendArray
extern int16_t Val_Osc[8];
void Virtual_Osc(uint8_t n,int16_t Val_Osc1, int16_t Val_Osc2, int16_t Val_Osc3, int16_t Val_Osc4, int16_t Val_Osc5,int16_t Val_Osc6,int16_t Val_Osc7,int16_t Val_Osc8);
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch );
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num);
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize);
//********************

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);


#endif
