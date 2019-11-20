#ifndef __ADS1256_H_
#define	__ADS1256_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "sys.h"
#include "delay.h"
#include "stm32f4xx_hal_exti.h"

#define SOFT_SPI		/* 定义此行表示使用GPIO模拟SPI接口 */
//#define HARD_SPI		/* 定义此行表示使用CPU的硬件SPI接口 */


#if !defined(SOFT_SPI) && !defined(HARD_SPI)
#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif


#ifdef HARD_SPI		/* 硬件SPI */
	;
#endif


//开始define
#ifdef SOFT_SPI		/* 软件SPI */


	/*端口定义*/ 
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


	/* 定义口线置0和置1的宏 */
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
//结束define



/* 增益选项 */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* 增益1（缺省） */
	ADS1256_GAIN_2			= (1),	/* 增益2 */
	ADS1256_GAIN_4			= (2),	/* 增益4 */
	ADS1256_GAIN_8			= (3),	/* 增益8 */
	ADS1256_GAIN_16			= (4),	/* 增益16 */
	ADS1256_GAIN_32			= (5),	/* 增益32 */
	ADS1256_GAIN_64			= (6),	/* 增益64 */	
}ADS1256_GAIN_E;

/* 采样速率选项 */
/* 数据转换率选择
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
	ADS1256_GAIN_E Gain;		/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int32_t AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;			/* 当前通道 */	
	uint8_t ScanMode;			/* 扫描模式，0表示单端8路， 1表示差分4路 */
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


/* 寄存器定义： Table 23. Register Map --- ADS1256数据手册第30页 */
enum
{
	/* 寄存器地址， 后面是复位后缺省值 */
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

/* 命令定义： TTable 24. Command Definitions --- ADS1256数据手册第34页 */
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
	0xF0,		/* 复位时缺省值 */
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



/*函数定义*/
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









//	/* 设置PGA增益，数据更新速率 */
//	#if 1
//		printf("\r\nPGA增益 = 1, 数据输出速率 = 500sps, 单端8路扫描\r\n\r\n");
//		
//		ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_500SPS);	/* 配置ADC参数： 增益1:1, 数据输出速率 15Hz */

//		/* 		
//		   中断服务程序会自动读取ADC结果保存在全局变量，主程序通过 ADS1256_GetAdc() 函数来读取这些数据
//		*/
//		ADS1256_StartScan(0);	/* 启动中断扫描模式. 0表示单端8路，1表示差分4路 */
//		ch_num = 8;		/* 通道数 = 8 或者4 */
//	#else
//		printf("\r\nPGA增益 = 1, 数据输出速率 = 500sps, 差分4路扫描\r\n\r\n");
//		
//		ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_500SPS);	/* 配置ADC参数： 增益1:1, 数据输出速率 15Hz */

//		/* 		
//		   中断服务程序会自动读取ADC结果保存在全局变量，主程序通过 ADS1256_GetAdc() 函数来读取这些数据
//		*/
//		ADS1256_StartScan(1);	/* 启动中断扫描模式. 0表示单端8路，1表示差分4路 */
//		ch_num = 4;		/* 通道数 = 8 或者4 */	
//	#endif












/*
    ADS1256模块    STM32-V5开发板(STM32F407IG)
      +5V   <------  5.0V      5V供电
      GND   -------  GND       地
      DRDY  ------>  PH9       准备就绪
      CS    <------  PH10      SPI_CS
      DIN   <------  PH11      SPI_MOSI
      DOUT  ------>  PA5       SPI_MISO
      SCLK  <------  PA4       SPI时钟
      GND   -------  GND       地
      PDWN  <------  PA0       掉电控制
      RST   <------  PC0       复位信号
      NC   空脚
      NC   空脚
*/

/*
	ADS1256基本特性:
	1、模拟部分供电5V;
	2、SPI数字接口电平：3.3V
	3、PGA设置范围： 1、2、4、8、16、32、64、
	4、参考电压2.5V (推荐缺省的，外置的）
	5、输入电压范围：PGA = 1 时, 可输入正负5V
	6. 自动校准 （当设置了PGA,BUF使能、数据采样率时，会启动自校准)
	7. 输入的缓冲器可设置启用和关闭（一般选启用）

	外部晶振频率 = 7.68MHz,
		时钟频率 tCLK = 1/7.68M = 0.13uS
		输出数据周期 tDATA =  1 / 30K = 0.033mS  (按30Ksps计算)

	对SPI的时钟速度要求: (ads1256.pdf page 6)
		最快 4个tCLK = 0.52uS
		最慢 10个tDATA = 0.3mS (按 30Ksps 计算)

		SCL高电平和低电平持续时间最小 200ns

	RREG, WREG, RDATA 命令之后，需要延迟 4 * tCLK = 0.52uS;
	RDATAC, RESET, SYNC 命令之后，需要延迟 24 * tCLK = 3.12uS;

	实际测试，在3.3V上电后, 及时不做任何配置，ADS125的DRDY 口线即开始输出脉冲信号（2.6us高,33.4低，频率30KHz）
*/

/*
	调试记录
	(1) 设置寄存器时，SCK过快导致芯片不能每次都收到数据。原因: 发送的相邻的字节之间需要延迟一小段时间.
	(2) 连续复位CPU时，偶尔出现芯片输出采样率异常。
*/
