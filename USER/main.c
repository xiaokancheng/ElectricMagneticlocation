#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "sdram.h"
#include "AD9851.h"
#include "ADS1256.h"
#include "timer.h"

void Modular_Init(void);
void Screen_Init(void);
void Read_ID(void);
void sin(void);

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
//    int32_t iTemp;
//    uint8_t i;
//    int32_t adc[8];
//    int32_t volt[8];
    
    
    Modular_Init();                 //模块初始化
//    TIM3_Init(50-1,900-1);       //定时器3初始化，定时器时钟为90M，分频系数为9000-1，
                                    //所以定时器3的频率为90M/9000=10K，自动重装载为5000-1，那么定时器周期就是500ms
    Screen_Init();
    
    delay_ms(100);
    
    Ad9850_Init(serial, 10000);    //AD9850初始化，串并行,频率
    bsp_InitADS1256();	    /* 初始化配置ADS1256.  PGA=1, DRATE=30KSPS, BUFEN=1, 输入正负5V */
    /*AD1256初始化，1为差分，0为单端,后面为PGA增益及数据输出速率*/
    bsp_Cfg_ADS1256(0, ADS1256_GAIN_1, ADS1256_30000SPS);
    Read_ID();              /* 打印芯片ID (通过读ID可以判断硬件接口是否正常) , 正常时状态寄存器的高4bit = 3 */	

    
	while(1)
	{
////        m=KEY_Scan(0);
////        if(m==KEY0_PRES)
////        {
//////            ad9851_wr_parrel(0x04,10000);
////            ad9851_wr_serial(0x04,1);
////        }
////        else if(m==KEY1_PRES)
////        {
//////            ad9851_wr_parrel(0x00,10000);
////            ad9851_wr_serial(0x00,1);
////        }


        LED_Test();		/* 空闲时执行的函数 */

//    for (i = 0; i < ch_num; i++)
//    {
//        /* 从全局缓冲区读取采样结果。 采样结果是在中断服务程序中读取的。*/
//        adc[i] = ADS1256_GetAdc(i);

//        /* 4194303 = 2.5V , 这是理论值，实际可以根据2.5V基准的实际值进行公式矫正 */
//        volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* 计算实际电压值（近似估算的），如需准确，请进行校准 */
//    }
//    
//    for (i = 0; i < ch_num; i++)
//    {
//        iTemp = volt[i];    	/* 余数，uV  */
////        if (iTemp < 0)
////        {
////            //iTemp = -iTemp;
////            //printf("%d 路=%6d,(-%d.%03d %03d V) \r\n", i, adc[i], iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);
////            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
////            //LCD_ShowxNum(40,150,iTemp,8,16,0);
////            if(i == 1)
////                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
////        }
////        else
////        {
////            //printf("%d 路=%6d,( %d.%03d %03d V) \r\n", i, adc[i], iTemp/1000000, (iTemp%1000000)/1000, iTemp%1000);
////            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
////            //LCD_ShowxNum(40,150,iTemp,8,16,0);
////            if(i == 1)
////                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
////        }
//        if(i == 1)
//                Virtual_Osc(4,(int16_t) (iTemp), 0, 0, 0, 0, 0, 0, 0);
//    }        



//            printf("%d 路扫描\r\n", ch_num);   
//            sin();

//            delay_ms(1);	              /* 每隔50ms 输出一次数据 */
        
	}
//    return 0;
}


//********************************************
//main函数END
//以下是模块函数


void Modular_Init(void)
{
    HAL_Init();                     //初始化HAL库   
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
    uart_init(115200);              //初始化USART
    LED_Init();                     //初始化LED 
    KEY_Init();                     //初始化按键
    SDRAM_Init();                   //初始化SDRAM
    LCD_Init();                     //初始化LCD
}
void Screen_Init(void)
{
  	POINT_COLOR=RED;                //电磁场精确定位系统 Accurate Positioning System of Electromagnetic Field
	LCD_ShowString(30,50,200,16,16,"Positioning System of EMF");
	LCD_ShowString(30,70,200,16,16,"NCU College of Physics");
	LCD_ShowString(30,90,200,16,16,"Physics Department");
	LCD_ShowString(30,110,200,16,16,"2019/10/1");
}

void Read_ID(void)
{
	/* 打印芯片ID (通过读ID可以判断硬件接口是否正常) , 正常时状态寄存器的高4bit = 3 */	    
    {
		uint8_t id;

		id = ADS1256_ReadChipID();

		printf("\r\n");
		printf("读取芯片ID\r\n");
		if (id != 3)
		{
			printf("Error, ASD1256 Chip ID = 0x%X\r\n", id);
		}
		else
		{
			printf("Ok, ASD1256 Chip ID = 0x%X\r\n", id);
		}
	}
}


















