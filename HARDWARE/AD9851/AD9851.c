#include "AD9851.h"
#include "delay.h"
#include "usart.h"

//并行接线 D0-D7 分别连接 PD0-PD7
//串行接线 D7 接 PD7
//w_clk，fq_up,rest 分别连接 PC8，PC7，PC6

void Ad9850_Init(u8 mode, u16 frequency)
{
    ad9851_Init();                  //初始化AD9851模块，0x00代表前8位，输出频率为后32位
    if(mode == parrel)
    {
    	ad9851_reset_parrel();
    	ad9851_wr_parrel(0x00,frequency);   //并行
    }
    if(mode == serial)
    {
    ad9851_reset_serial();          
    ad9851_wr_serial(0x00,frequency);   //串行
    }
}


u8 data1;u8 data2;u8 data3;u8 data4;u8 data5;           //做测试用

void ad9851_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    __HAL_RCC_GPIOD_CLK_ENABLE();			//使能GPIOD时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();			//使能GPIOC时钟

    GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;    //普通推挽输出模式
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;    //下拉
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);    //初始化GPIO
	
    GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //普通推挽输出模式
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET);	//置0 
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,GPIO_PIN_RESET);	//置0 	
}

void ad9851_reset_parrel(void)
{
    ad9851_w_clk=0;
    ad9851_fq_up=0;
    //rest信号
    ad9851_rest=0;
    ad9851_rest=1;
    ad9851_rest=0;
}

void ad9851_reset_serial(void)
{
    ad9851_w_clk=0;
    ad9851_fq_up=0;
    //rest信号
    ad9851_rest=0;
    ad9851_rest=1;
    ad9851_rest=0;
    //w_clk信号
    ad9851_w_clk=0;
    ad9851_w_clk=1;
    ad9851_w_clk=0;
    //fq_up信号
    ad9851_fq_up=0;
    ad9851_fq_up=1;
    ad9851_fq_up=0;
}


                        /*并行接口*/
/* 前八位为前5位控制相位精度11.25°，第6位电源1休眠0运行，第七位总是逻辑0，第八位6倍时钟倍乘器1启动0关闭*/

void ad9851_wr_parrel(unsigned char w0,double frequence)
{
    volatile unsigned char w;
    long int y;
    double x;

    x=4294967295/125;
    frequence=frequence/1000000;
    frequence=frequence*x;                      //将32位频率FFFF FFFF分成125M   34.359738D
    y=frequence;                                //给定输出频率

    /*写w0数据*/
    w=w0;
    HAL_GPIO_Write(GPIOD,w&0xff);            //w0
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data1=w;
    /*写w1数据*/
    w=(y>>24);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w1                
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data2=w;
    /*写w2数据*/
    w=(y>>16);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w2
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data3=w;
    /*写w3数据*/
    w=(y>>8);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w3
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data4=w;
    /*写w4数据*/
    w=(y>>0);
    HAL_GPIO_Write(GPIOD,w&0xff);            //w4
    ad9851_w_clk=1;
    ad9851_w_clk=0;     data5=w;
    
    /*移入使能*/
    ad9851_fq_up=1;
    ad9851_fq_up=0;
}

                        /*串行接口*/
void ad9851_wr_serial(unsigned char w0,double frequence)
{
    unsigned char i;
    volatile unsigned char w;
    long int y;
    double x;

    x=4294967295/125;
    frequence=frequence/1000000;
    frequence=frequence*x;                      //将32位频率FFFF FFFF分成125M   34.359738D
    y=frequence;                                //给定输出频率

        //写w4数据
    w=(y>>=0);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;           //w4
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //写w3数据
    w=(y>>8);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w3
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //写w2数据
    w=(y>>16);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w2
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //写w1数据
    w=(y>>24);
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w1
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }
        //写w0数据
    w=w0;   
    for(i=0;i<8;i++)
    {
        ad9851_bit_data=(w>>i)&0x01;            //w0
        ad9851_w_clk=1;
        ad9851_w_clk=0;
    }

        //移入始能
    ad9851_fq_up=1;
    ad9851_fq_up=0;
}
