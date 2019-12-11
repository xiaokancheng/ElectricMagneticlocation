#include "AD9851.h"
#include "delay.h"
#include "usart.h"

//���н��� D0-D7 �ֱ����� PD0-PD7
//���н��� D7 �� PD7
//w_clk��fq_up,rest �ֱ����� PC8��PC7��PC6

void Ad9850_Init(u8 mode, u16 frequency)
{
    ad9851_Init();                  //��ʼ��AD9851ģ�飬0x00����ǰ8λ�����Ƶ��Ϊ��32λ
    if(mode == parrel)
    {
    	ad9851_reset_parrel();
    	ad9851_wr_parrel(0x00,frequency);   //����
    }
    if(mode == serial)
    {
    ad9851_reset_serial();          
    ad9851_wr_serial(0x00,frequency);   //����
    }
}


u8 data1;u8 data2;u8 data3;u8 data4;u8 data5;           //��������

void ad9851_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    __HAL_RCC_GPIOD_CLK_ENABLE();			//ʹ��GPIODʱ��
    __HAL_RCC_GPIOC_CLK_ENABLE();			//ʹ��GPIOCʱ��

    GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;    //��ͨ�������ģʽ
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;    //����
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);    //��ʼ��GPIO
	
    GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //��ͨ�������ģʽ
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET);	//��0 
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,GPIO_PIN_RESET);	//��0 	
}

void ad9851_reset_parrel(void)
{
    ad9851_w_clk=0;
    ad9851_fq_up=0;
    //rest�ź�
    ad9851_rest=0;
    ad9851_rest=1;
    ad9851_rest=0;
}

void ad9851_reset_serial(void)
{
    ad9851_w_clk=0;
    ad9851_fq_up=0;
    //rest�ź�
    ad9851_rest=0;
    ad9851_rest=1;
    ad9851_rest=0;
    //w_clk�ź�
    ad9851_w_clk=0;
    ad9851_w_clk=1;
    ad9851_w_clk=0;
    //fq_up�ź�
    ad9851_fq_up=0;
    ad9851_fq_up=1;
    ad9851_fq_up=0;
}


                        /*���нӿ�*/
/* ǰ��λΪǰ5λ������λ����11.25�㣬��6λ��Դ1����0���У�����λ�����߼�0���ڰ�λ6��ʱ�ӱ�����1����0�ر�*/

void ad9851_wr_parrel(unsigned char w0,double frequence)
{
    volatile unsigned char w;
    long int y;
    double x;

    x=4294967295/125;
    frequence=frequence/1000000;
    frequence=frequence*x;                      //��32λƵ��FFFF FFFF�ֳ�125M   34.359738D
    y=frequence;                                //�������Ƶ��

    /*дw0����*/
    w=w0;
    HAL_GPIO_Write(GPIOD,w&0xff);            //w0
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data1=w;
    /*дw1����*/
    w=(y>>24);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w1                
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data2=w;
    /*дw2����*/
    w=(y>>16);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w2
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data3=w;
    /*дw3����*/
    w=(y>>8);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w3
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data4=w;
    /*дw4����*/
    w=(y>>0);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w4
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data5=w;
    
    /*����ʹ��*/
    ad9851_fq_up=1;
    ad9851_fq_up=0;
}

                        /*���нӿ�*/
void ad9851_wr_serial(unsigned char w0,double frequence)
{
    unsigned char i;
    volatile unsigned char w;
    long int y;
    double x;

    x=4294967295/125;
    frequence=frequence/1000000;
    frequence=frequence*x;                      //��32λƵ��FFFF FFFF�ֳ�125M   34.359738D
    y=frequence;                                //�������Ƶ��

        //дw4����
    w=(y>>=0);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;           //w4
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //дw3����
    w=(y>>8);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w3
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //дw2����
    w=(y>>16);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w2
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //дw1����
    w=(y>>24);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w1
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //дw0����
    w=w0;   
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w0
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }

        //����ʼ��
    ad9851_fq_up=1;
    ad9851_fq_up=0;
}
