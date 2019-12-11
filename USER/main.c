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
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{
//    int32_t iTemp;
//    uint8_t i;
//    int32_t adc[8];
//    int32_t volt[8];
    
    
    Modular_Init();                 //ģ���ʼ��
//    TIM3_Init(50-1,900-1);       //��ʱ��3��ʼ������ʱ��ʱ��Ϊ90M����Ƶϵ��Ϊ9000-1��
                                    //���Զ�ʱ��3��Ƶ��Ϊ90M/9000=10K���Զ���װ��Ϊ5000-1����ô��ʱ�����ھ���500ms
    Screen_Init();
    
    delay_ms(100);
    
    Ad9850_Init(serial, 10000);    //AD9850��ʼ����������,Ƶ��
    bsp_InitADS1256();	    /* ��ʼ������ADS1256.  PGA=1, DRATE=30KSPS, BUFEN=1, ��������5V */
    /*AD1256��ʼ����1Ϊ��֣�0Ϊ����,����ΪPGA���漰�����������*/
    bsp_Cfg_ADS1256(0, ADS1256_GAIN_1, ADS1256_30000SPS);
    Read_ID();              /* ��ӡоƬID (ͨ����ID�����ж�Ӳ���ӿ��Ƿ�����) , ����ʱ״̬�Ĵ����ĸ�4bit = 3 */	

    
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


        LED_Test();		/* ����ʱִ�еĺ��� */

//    for (i = 0; i < ch_num; i++)
//    {
//        /* ��ȫ�ֻ�������ȡ��������� ������������жϷ�������ж�ȡ�ġ�*/
//        adc[i] = ADS1256_GetAdc(i);

//        /* 4194303 = 2.5V , ��������ֵ��ʵ�ʿ��Ը���2.5V��׼��ʵ��ֵ���й�ʽ���� */
//        volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* ����ʵ�ʵ�ѹֵ�����ƹ���ģ�������׼ȷ�������У׼ */
//    }
//    
//    for (i = 0; i < ch_num; i++)
//    {
//        iTemp = volt[i];    	/* ������uV  */
////        if (iTemp < 0)
////        {
////            //iTemp = -iTemp;
////            //printf("%d ·=%6d,(-%d.%03d %03d V) \r\n", i, adc[i], iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);
////            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
////            //LCD_ShowxNum(40,150,iTemp,8,16,0);
////            if(i == 1)
////                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
////        }
////        else
////        {
////            //printf("%d ·=%6d,( %d.%03d %03d V) \r\n", i, adc[i], iTemp/1000000, (iTemp%1000000)/1000, iTemp%1000);
////            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
////            //LCD_ShowxNum(40,150,iTemp,8,16,0);
////            if(i == 1)
////                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
////        }
//        if(i == 1)
//                Virtual_Osc(4,(int16_t) (iTemp), 0, 0, 0, 0, 0, 0, 0);
//    }        



//            printf("%d ·ɨ��\r\n", ch_num);   
//            sin();

//            delay_ms(1);	              /* ÿ��50ms ���һ������ */
        
	}
//    return 0;
}


//********************************************
//main����END
//������ģ�麯��


void Modular_Init(void)
{
    HAL_Init();                     //��ʼ��HAL��   
    Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
    delay_init(180);                //��ʼ����ʱ����
    uart_init(115200);              //��ʼ��USART
    LED_Init();                     //��ʼ��LED 
    KEY_Init();                     //��ʼ������
    SDRAM_Init();                   //��ʼ��SDRAM
    LCD_Init();                     //��ʼ��LCD
}
void Screen_Init(void)
{
  	POINT_COLOR=RED;                //��ų���ȷ��λϵͳ Accurate Positioning System of Electromagnetic Field
	LCD_ShowString(30,50,200,16,16,"Positioning System of EMF");
	LCD_ShowString(30,70,200,16,16,"NCU College of Physics");
	LCD_ShowString(30,90,200,16,16,"Physics Department");
	LCD_ShowString(30,110,200,16,16,"2019/10/1");
}

void Read_ID(void)
{
	/* ��ӡоƬID (ͨ����ID�����ж�Ӳ���ӿ��Ƿ�����) , ����ʱ״̬�Ĵ����ĸ�4bit = 3 */	    
    {
		uint8_t id;

		id = ADS1256_ReadChipID();

		printf("\r\n");
		printf("��ȡоƬID\r\n");
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


















