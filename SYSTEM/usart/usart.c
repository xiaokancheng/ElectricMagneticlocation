#include "usart.h"
#include "delay.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��os,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os ʹ��	  
#endif

//V1.0�޸�˵�� 
////////////////////////////////////////////////////////////////////////////////// 	  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
UART_HandleTypeDef UART1_Handler; //UART���

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound)
{	
	//UART ��ʼ������
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //������
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
	
	//HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������(ʹ�ûص����������ж���Ҫ���øú�����
  
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//����ΪUSART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
		__HAL_UART_DISABLE_IT(huart,UART_IT_TC);
#if EN_USART1_RX
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//���������ж�
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}

}


 


//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 
	u8 Res;
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
#endif
	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
        HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART1_Handler);	
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
#endif
} 
#endif	


/**
  * @brief  ����ʾ�����Ķ��弰��������
  *         
  * @param  
  * @retval 
  */

#define  __GNUC__

#ifdef   __GNUC__ 
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */ 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch) 
#else 
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) 
#endif 
/* __GNUC__ */ /** * @brief Retargets the C library printf function to the USART. * @param None * @retval None */ 

PUTCHAR_PROTOTYPE
{ 
    HAL_UART_Transmit(&UART1_Handler, (uint8_t *)&ch, 1, 0); 
    return ch;
}//���ڵ��Ա���

        /*����ʾ����*/
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
#define  CMD_WARE   0x03
    
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����

    uart_putbuff(&UART1_Handler, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
    uart_putbuff(&UART1_Handler, wareaddr, waresize);    //��������
    uart_putbuff(&UART1_Handler, cmdr, sizeof(cmdr));    //���ͺ�����
}

/******** ����һ���ַ� *********/
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch )
{
  HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 10); 
}
	
/******** ����8λ������ *********/
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num)
{
	uint8_t i;
	for(i=0;i<num;i++)
	{
		Usart_SendByte(huart,array[i]);
	}
		
}


int16_t Val_Osc[8];
void Virtual_Osc(uint8_t n,int16_t Val_Osc1, int16_t Val_Osc2, int16_t Val_Osc3, int16_t Val_Osc4, int16_t Val_Osc5,int16_t Val_Osc6,int16_t Val_Osc7,int16_t Val_Osc8)
{
    int16_t  *ptr = Val_Osc;          //��ʼ��ָ��
    uint8_t  i;
      
    Val_Osc[0] = (int16_t)(Val_Osc1);
    Val_Osc[1] = (int16_t)(Val_Osc2);
    Val_Osc[2] = (int16_t)(Val_Osc3);
    Val_Osc[3] = (int16_t)(Val_Osc4);
    Val_Osc[4] = (int16_t)(Val_Osc5);
    Val_Osc[5] = (int16_t)(Val_Osc6);
    Val_Osc[6] = (int16_t)(Val_Osc7);
    Val_Osc[7] = (int16_t)(Val_Osc8);
    
    //��Щ��ֵ���Ըĳ���Ҫ��������
    /*������ʾ��������������*/
    for(i=0; i<n; i++)
    {    
        vcan_sendware( (uint8_t *)ptr, 4*sizeof(Val_Osc[i]) );
    }
}
