#ifndef __ADS1256_H_
#define	__ADS1256_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "sys.h"
#include "delay.h"
#include "stm32f4xx_hal_exti.h"

#define SOFT_SPI		/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */
//#define HARD_SPI		/* ������б�ʾʹ��CPU��Ӳ��SPI�ӿ� */


#if !defined(SOFT_SPI) && !defined(HARD_SPI)
#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif


#ifdef HARD_SPI		/* Ӳ��SPI */
	;
#endif


//��ʼdefine
#ifdef SOFT_SPI		/* ���SPI */


	/*�˿ڶ���*/ 
#define PORT_SCK	GPIOB
#define PIN_SCK		GPIO_PIN_3

#define PORT_DIN	GPIOB
#define PIN_DIN		GPIO_PIN_4

#define PORT_CS		GPIOB
#define PIN_CS		GPIO_PIN_5

#define PORT_DRDY	GPIOB
#define PIN_DRDY	GPIO_PIN_6

#define PORT_DOUT	GPIOB
#define PIN_DOUT	GPIO_PIN_7

#define PORT_PWDN	GPIOB
#define PIN_PWDN	GPIO_PIN_8
	
#define PORT_RESET	GPIOB
#define PIN_RESET	GPIO_PIN_9


	/* ���������0����1�ĺ� */
#define CS_0()		HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_RESET)
#define CS_1()		HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_SET)

#define SCK_0()		HAL_GPIO_WritePin(PORT_SCK, PIN_SCK, GPIO_PIN_RESET)
#define SCK_1()		HAL_GPIO_WritePin(PORT_SCK, PIN_SCK, GPIO_PIN_SET)

#define DI_0()		HAL_GPIO_WritePin(PORT_DIN, PIN_DIN, GPIO_PIN_RESET)
#define DI_1()		HAL_GPIO_WritePin(PORT_DIN, PIN_DIN, GPIO_PIN_SET)
	
#define PWDN_0()		HAL_GPIO_WritePin(PORT_PWDN, PIN_PWDN, GPIO_PIN_RESET)
#define PWDN_1()		HAL_GPIO_WritePin(PORT_PWDN, PIN_PWDN, GPIO_PIN_SET)
	
#define RESET_0()		HAL_GPIO_WritePin(PORT_RESET, PIN_RESET, GPIO_PIN_RESET)
#define RESET_1()		HAL_GPIO_WritePin(PORT_RESET, PIN_RESET, GPIO_PIN_SET)

#define DO_IS_HIGH()	(PBin(7) == 1)
#define DRDY_IS_LOW()	(PBin(6) == 0)


#endif
//����define



/* ����ѡ�� */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* ����1��ȱʡ�� */
	ADS1256_GAIN_2			= (1),	/* ����2 */
	ADS1256_GAIN_4			= (2),	/* ����4 */
	ADS1256_GAIN_8			= (3),	/* ����8 */
	ADS1256_GAIN_16			= (4),	/* ����16 */
	ADS1256_GAIN_32			= (5),	/* ����32 */
	ADS1256_GAIN_64			= (6),	/* ����64 */	
}ADS1256_GAIN_E;

/* ��������ѡ�� */
/* ����ת����ѡ��
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,
	
	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* ���� */
	ADS1256_DRATE_E DataRate;	/* ����������� */
	int32_t AdcNow[8];			/* 8·ADC�ɼ������ʵʱ���з����� */
	uint8_t Channel;			/* ��ǰͨ�� */	
	uint8_t ScanMode;			/* ɨ��ģʽ��0��ʾ����8·�� 1��ʾ���4· */
    uint8_t ReadOver;
}ADS1256_VAR_T;

void bsp_InitADS1256(void);
extern uint8_t ch_num;
void bsp_Cfg_ADS1256(u8 mode, ADS1256_GAIN_E ADS1256_GAIN, ADS1256_DRATE_E ADS1256_SPS);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

uint8_t ADS1256_ReadChipID(void);
void ADS1256_StartScan(uint8_t _ucScanMode);
void ADS1256_StopScan(void);
void ADS1256_ISR(void);
int32_t ADS1256_GetAdc(uint8_t _ch);

extern ADS1256_VAR_T g_tADS1256;

#endif


/* �Ĵ������壺 Table 23. Register Map --- ADS1256�����ֲ��30ҳ */
enum
{
	/* �Ĵ�����ַ�� �����Ǹ�λ��ȱʡֵ */
	REG_STATUS = 0,	    // x1H
	REG_MUX    = 1,     // 01H
	REG_ADCON  = 2,     // 20H
	REG_DRATE  = 3,     // F0H
	REG_IO     = 4,     // E0H
	REG_OFC0   = 5,     // xxH
	REG_OFC1   = 6,     // xxH
	REG_OFC2   = 7,     // xxH
	REG_FSC0   = 8,     // xxH
	REG_FSC1   = 9,     // xxH
	REG_FSC2   = 10,    // xxH
};

/* ����壺 TTable 24. Command Definitions --- ADS1256�����ֲ��34ҳ */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/* ��λʱȱʡֵ */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};



/*��������*/
static void ADS1256_Send8Bit(uint8_t _data);
static uint8_t ADS1256_Recive8Bit(void);
static void ADS1256_WaitDRDY(void);
static void ADS1256_ResetHard(void);
static void ADS1256_DelaySCLK(void);
static void ADS1256_DelayDATA(void);

static void ADS1256_WriteCmd(uint8_t _cmd);
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
static uint8_t ADS1256_ReadReg(uint8_t _RegID);
static int32_t ADS1256_ReadData(void);
static void ADS1256_SetChannal(uint8_t _ch);
//static void ADS1256_SetDiffChannal(uint8_t _ch);









//	/* ����PGA���棬���ݸ������� */
//	#if 1
//		printf("\r\nPGA���� = 1, ����������� = 500sps, ����8·ɨ��\r\n\r\n");
//		
//		ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_500SPS);	/* ����ADC������ ����1:1, ����������� 15Hz */

//		/* 		
//		   �жϷ��������Զ���ȡADC���������ȫ�ֱ�����������ͨ�� ADS1256_GetAdc() ��������ȡ��Щ����
//		*/
//		ADS1256_StartScan(0);	/* �����ж�ɨ��ģʽ. 0��ʾ����8·��1��ʾ���4· */
//		ch_num = 8;		/* ͨ���� = 8 ����4 */
//	#else
//		printf("\r\nPGA���� = 1, ����������� = 500sps, ���4·ɨ��\r\n\r\n");
//		
//		ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_500SPS);	/* ����ADC������ ����1:1, ����������� 15Hz */

//		/* 		
//		   �жϷ��������Զ���ȡADC���������ȫ�ֱ�����������ͨ�� ADS1256_GetAdc() ��������ȡ��Щ����
//		*/
//		ADS1256_StartScan(1);	/* �����ж�ɨ��ģʽ. 0��ʾ����8·��1��ʾ���4· */
//		ch_num = 4;		/* ͨ���� = 8 ����4 */	
//	#endif












/*
    ADS1256ģ��    STM32-V5������(STM32F407IG)
      +5V   <------  5.0V      5V����
      GND   -------  GND       ��
      DRDY  ------>  PH9       ׼������
      CS    <------  PH10      SPI_CS
      DIN   <------  PH11      SPI_MOSI
      DOUT  ------>  PA5       SPI_MISO
      SCLK  <------  PA4       SPIʱ��
      GND   -------  GND       ��
      PDWN  <------  PA0       �������
      RST   <------  PC0       ��λ�ź�
      NC   �ս�
      NC   �ս�
*/

/*
	ADS1256��������:
	1��ģ�ⲿ�ֹ���5V;
	2��SPI���ֽӿڵ�ƽ��3.3V
	3��PGA���÷�Χ�� 1��2��4��8��16��32��64��
	4���ο���ѹ2.5V (�Ƽ�ȱʡ�ģ����õģ�
	5�������ѹ��Χ��PGA = 1 ʱ, ����������5V
	6. �Զ�У׼ ����������PGA,BUFʹ�ܡ����ݲ�����ʱ����������У׼)
	7. ����Ļ��������������ú͹رգ�һ��ѡ���ã�

	�ⲿ����Ƶ�� = 7.68MHz,
		ʱ��Ƶ�� tCLK = 1/7.68M = 0.13uS
		����������� tDATA =  1 / 30K = 0.033mS  (��30Ksps����)

	��SPI��ʱ���ٶ�Ҫ��: (ads1256.pdf page 6)
		��� 4��tCLK = 0.52uS
		���� 10��tDATA = 0.3mS (�� 30Ksps ����)

		SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns

	RREG, WREG, RDATA ����֮����Ҫ�ӳ� 4 * tCLK = 0.52uS;
	RDATAC, RESET, SYNC ����֮����Ҫ�ӳ� 24 * tCLK = 3.12uS;

	ʵ�ʲ��ԣ���3.3V�ϵ��, ��ʱ�����κ����ã�ADS125��DRDY ���߼���ʼ��������źţ�2.6us��,33.4�ͣ�Ƶ��30KHz��
*/

/*
	���Լ�¼
	(1) ���üĴ���ʱ��SCK���쵼��оƬ����ÿ�ζ��յ����ݡ�ԭ��: ���͵����ڵ��ֽ�֮����Ҫ�ӳ�һС��ʱ��.
	(2) ������λCPUʱ��ż������оƬ����������쳣��
*/
