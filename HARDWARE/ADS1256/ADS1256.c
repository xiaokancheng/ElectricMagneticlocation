#include <stdio.h>
#include "stm32f4xx.h"
#include "ADS1256.h"
#include "usart.h"
#include "stm32f4xx_hal_cortex.h"


ADS1256_VAR_T g_tADS1256;
EXTI_ConfigTypeDef   EXTI_InitStructure;
EXTI_HandleTypeDef   EXTI_InitHandle;
GPIO_InitTypeDef     GPIO_Initure;
uint8_t ch_num;

////初始化ADS1256 GPIO
void bsp_InitADS1256(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef  SOFT_SPI
    
	RESET_1();
	PWDN_1();
	CS_1();
	SCK_0();		/* SPI总线空闲时，钟线是低电平 */
	DI_1();
    
    
    __HAL_RCC_GPIOB_CLK_ENABLE();			                //使能GPIOB时钟

    GPIO_InitStructure.Pin = PIN_SCK | PIN_DIN | PIN_CS | PIN_RESET | PIN_PWDN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;          //普通推挽输出模式
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP;                  //上拉
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化GPIO

    GPIO_InitStructure.Pin = PIN_DOUT | PIN_DRDY;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;          //普通推挽输出模式
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP;                  //上拉
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化GPIO

#endif
}

void bsp_Cfg_ADS1256(u8 mode, ADS1256_GAIN_E ADS1256_GAIN, ADS1256_DRATE_E ADS1256_SPS)
{

    if(mode == 0)
    {
		printf("\r\nPGA增益 = %d, 数据输出速率 = %d, 单端8路扫描\r\n\r\n",ADS1256_GAIN, ADS1256_SPS);
		
		ADS1256_CfgADC(ADS1256_GAIN, ADS1256_SPS);	/* 配置ADC参数： 增益1:1, 数据输出速率 15Hz */

		/* 		
		   中断服务程序会自动读取ADC结果保存在全局变量，主程序通过 ADS1256_GetAdc() 函数来读取这些数据
		*/
		ADS1256_StartScan(mode);	/* 启动中断扫描模式. 0表示单端8路，1表示差分4路 */
		ch_num = 8;		/* 通道数 = 8 或者4 */
    }
    else
    {
        printf("\r\nPGA增益 = %d, 数据输出速率 = %d, 差分4路扫描\r\n\r\n",ADS1256_GAIN, ADS1256_SPS);
		
		ADS1256_CfgADC(ADS1256_GAIN, ADS1256_SPS);	/* 配置ADC参数： 增益1:1, 数据输出速率 15Hz */

		/* 		
		   中断服务程序会自动读取ADC结果保存在全局变量，主程序通过 ADS1256_GetAdc() 函数来读取这些数据
		*/
		ADS1256_StartScan(mode);	/* 启动中断扫描模式. 0表示单端8路，1表示差分4路 */
		ch_num = 4;		/* 通道数 = 8 或者4 */	
    }
}


/*
*********************************************************************************************************
*	函 数 名: ADS1256_CfgADC
*	功能说明: 配置ADC参数，增益和数据输出速率
*	形    参: _gain : 增益
*			  _drate : 数据输出速率
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_StopScan();			/* 暂停CPU中断 */

	ADS1256_ResetHard();		/* 硬件复位 */

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* 暂存ADS1256 寄存器配置参数，之后连续写4个寄存器 */

		/* 状态寄存器定义
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1使能自校准功能。当 PGA，BUFEEN, DRATE改变时会启动自校准
		*/
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	/* 高四位0表示AINP接 AIN0,  低四位8表示 AINN 固定接 AINCOM */

		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 μ A
				10 = Sensor Detect Current = 2 μ A
				11 = Sensor Detect Current = 10μ A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 3) | (GAIN_1 << 0));	/* 选择1;1增益, 输入正负5V */

		/* 因为切换通道和读数据耗时 123uS, 因此扫描中断模式工作时，最大速率 = DRATE_1000SPS */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* 选择数据输出速率 */

		CS_0();	/* SPI片选 = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* 写寄存器的命令, 并发送寄存器地址 */
		ADS1256_Send8Bit(0x03);			/* 寄存器个数 - 1, 此处3表示写4个寄存器 */

		ADS1256_Send8Bit(buf[0]);	/* 设置状态寄存器 */
		ADS1256_Send8Bit(buf[1]);	/* 设置输入通道参数 */
		ADS1256_Send8Bit(buf[2]);	/* 设置ADCON控制寄存器，增益 */
		ADS1256_Send8Bit(buf[3]);	/* 设置输出数据速率 */

		CS_1();	/* SPI片选 = 1 */
	}

	delay_us(50);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_DelaySCLK
*	功能说明: CLK之间的延迟，时序延迟. 用于STM32F429  180M主频
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelaySCLK(void)
{
	uint16_t i;

	/*
		取 5 时，实测高电平200ns, 低电平250ns <-- 不稳定
		取 10 以上，可以正常工作， 低电平400ns 高定400ns <--- 稳定
	*/
	for (i = 0; i < 15; i++);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_DelayDATA
*	功能说明: 读取DOUT之前的延迟
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		最小 50 个tCLK = 50 * 0.13uS = 6.5uS
	*/
	delay_us(10);	/* 最小延迟 6.5uS, 此处取10us */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ResetHard
*	功能说明: 硬件复位 ADS1256芯片.低电平复位。最快4个时钟，也就是 4x0.13uS = 0.52uS
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_ResetHard(void)
{
	/* ADS1256数据手册第7页 */
	RESET_0();			/* 复位 */
	delay_us(5);
	RESET_1();

	//PWDN_0();			/* 进入掉电 同步*/
	//delay_us(2);
	//PWDN_1();			/* 退出掉电 */

	delay_us(5);

	//ADS1256_WaitDRDY();	/* 等待 DRDY变为0, 此过程实测: 630us */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_Send8Bit
*	功能说明: 向SPI总线发送8个bit数据。 不带CS控制。
*	形    参: _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{
	uint8_t i;

	/* 连续发送多个字节时，需要延迟一下 */
	ADS1256_DelaySCLK();
	ADS1256_DelaySCLK();

	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for(i = 0; i < 8; i++)
	{
		if (_data & 0x80)
		{
			DI_1();
		}
		else
		{
			DI_0();
		}
		SCK_1();
		ADS1256_DelaySCLK();
		_data <<= 1;
		SCK_0();			/* <----  ADS1256 是在SCK下降沿采样DIN数据, 数据必须维持 50nS */
		ADS1256_DelaySCLK();
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_Recive8Bit
*	功能说明: 从SPI总线接收8个bit数据。 不带CS控制。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t i;
	uint8_t read = 0;

	ADS1256_DelaySCLK();
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for (i = 0; i < 8; i++)
	{
		SCK_1();
		ADS1256_DelaySCLK();
		read = read<<1;
		SCK_0();
		if (DO_IS_HIGH())
		{
			read++;
		}
		ADS1256_DelaySCLK();
	}
	return read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WriteReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);		/* 寄存器个数 - 1, 此处写1个寄存器 */

	ADS1256_Send8Bit(_RegValue);	/* 发送寄存器值 */
	CS_1();	/* SPI片选 = 1 */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值。
*	返 回 值: 读到的寄存器值。
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);	/* 寄存器个数 - 1, 此处读1个寄存器 */

	ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */

	read = ADS1256_Recive8Bit();	/* 读寄存器值 */
	CS_1();	/* SPI片选 = 1 */

	return read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WriteCmd
*	功能说明: 发送单字节命令
*	形    参:  _cmd : 命令
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI片选 = 1 */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadChipID
*	功能说明: 读芯片ID, 读状态寄存器中的高4bit
*	形    参: 无
*	返 回 值: 8bit状态寄存器值的高4位
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetChannal
*	功能说明: 配置通道号。多路复用。AIN- 固定接地（ACOM).
*	形    参: _ch : 通道号， 0-7
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN 固定接 AINCOM */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetDiffChannal
*	功能说明: 配置差分通道号。多路复用。
*	形    参: _ch : 通道号,0-3；共4对
*	返 回 值: 8bit状态寄存器值的高4位
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* 差分输入 AIN0， AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* 差分输入 AIN2， AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* 差分输入 AIN4， AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* 差分输入 AIN6， AIN7 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WaitDRDY
*	功能说明: 等待内部操作完成。 自校准时间较长，需要等待。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 40000000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 40000000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* 调试语句. 用语排错 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadData
*	功能说明: 读ADC数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;

	CS_0();	/* SPI片选 = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* 读数据的命令 */

	ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */

	/* 读采样结果，3个字节，高字节在前 */
	read = ADS1256_Recive8Bit() << 16;
	read += (ADS1256_Recive8Bit() << 8);
	read += ADS1256_Recive8Bit();

	CS_1();	/* SPI片选 = 1 */

	/* 负数进行扩展。24位有符号数扩展为32位有符号数 */
	if (read & 0x800000)
	{
		read += 0xFF000000;
	}

	return (int32_t)read;
}

#if 0
/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadAdc
*	功能说明: 读指定通道的ADC数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t ADS1256_ReadAdc(uint8_t _ch)
{
	/* ADS1256 数据手册第21页 */

#if 0	/* 对于30Ksps 采样速率 */
	int32_t read;

	while (DRDY_IS_LOW());	/* 等待 DRDY 高 */
	while (!DRDY_IS_LOW());	/* 等待 DRDY 低 */

	ADS1256_SetChannal(_ch);	/* 切换模拟通道 */
	delay_us(5);

	ADS1256_WriteCmd(CMD_SYNC);
	delay_us(5);

	ADS1256_WriteCmd(CMD_WAKEUP);  /* 正常情况下，这个时候 DRDY 已经为高 */
	delay_us(25);

	read =  (int32_t)ADS1256_ReadData();

	while (DRDY_IS_LOW());	/* 等待 DRDY 高 */
	while (!DRDY_IS_LOW());	/* 等待 DRDY 低 */

	read =  (int32_t)ADS1256_ReadData();

	return read;
#else
	//while (DRDY_IS_LOW());

	/* ADS1256 数据手册第21页 */
	ADS1256_WaitDRDY();		/* 等待 DRDY = 0 */

	ADS1256_SetChannal(_ch);	/* 切换模拟通道 */
	delay_us(5);

	ADS1256_WriteCmd(CMD_SYNC);
	delay_us(5);

	ADS1256_WriteCmd(CMD_WAKEUP);
	delay_us(25);

	//ADS1256_WaitDRDY();		/* 等待 DRDY = 0 */

	return (int32_t)ADS1256_ReadData();
#endif
}

#endif


/*
*********************************************************************************************************
*	下面的函数用于DRDY中断工作模式
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StartScan
*	功能说明: 将 DRDY引脚 （PH9 ）配置成外部中断触发方式， 中断服务程序中扫描8个通道的数据。
*	形    参: _ucDiffMode : 0 表示单端模式（扫描8路）； 1表示差分模式，扫描4路
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode)
{
	g_tADS1256.ScanMode = _ucScanMode;
	/* 开始扫描前, 清零结果缓冲区 */
	{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; i++)
		{
			g_tADS1256.AdcNow[i] = 0;
		}
	}

	/* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();             //开启GPIOB时钟
    
	/* Configure EXTI3 line */
    GPIO_Initure.Pin=PIN_DRDY;                //PB6
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //下降沿触发
    GPIO_Initure.Pull=GPIO_PULLUP;
    
    /* Connect EXTI3 Line to PB6 pin */
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
	/* Enable and set EXTI3 Interrupt  priority */   
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StopScan
*	功能说明: 停止 DRDY 中断
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_StopScan(void)
{
	/* Configure EXTI3 line */
    GPIO_Initure.Pin=PIN_DRDY;                //PB6
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //下降沿触发
    GPIO_Initure.Pull=GPIO_PULLUP;
    
    /* Connect EXTI3 Line to PB6 pin */
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    __HAL_GPIO_EXTI_CLEAR_IT(PIN_DRDY);
    
	/* 中断优先级配置 最低优先级 这里一定要分开的设置中断，不能够合并到一个里面设置 */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_GetAdc
*	功能说明: 从缓冲区读取ADC采样结果。采样结构是由中断服务程序填充的。
*	形    参: _ch 通道号 (0 - 7)
*	返 回 值: ADC采集结果（有符号数）
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	__set_PRIMASK(1);	/* 禁止中断 */

	iTemp = g_tADS1256.AdcNow[_ch];

	__set_PRIMASK(0);	/* 使能中断 */

	return iTemp;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ISR
*	功能说明: 定时采集中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	if (g_tADS1256.ScanMode == 0)	/* 0 表示单端8路扫描，1表示差分4路扫描 */
	{
		/* 读取采集结构，保存在全局变量 */
		ADS1256_SetChannal(g_tADS1256.Channel);	/* 切换模拟通道 */
		delay_us(5);

		ADS1256_WriteCmd(CMD_SYNC);
		delay_us(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		delay_us(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
		}

		if (++g_tADS1256.Channel >= 8)
		{
			g_tADS1256.Channel = 0;
		}
	}
	else	/* 差分4路扫描 */
	{
		/* 读取采集结构，保存在全局变量 */
		ADS1256_SetDiffChannal(g_tADS1256.Channel);	/* 切换模拟通道 */
		delay_us(5);

		ADS1256_WriteCmd(CMD_SYNC);
		delay_us(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		delay_us(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[3] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
		}

		if (++g_tADS1256.Channel >= 4)
		{
			g_tADS1256.Channel = 0;
            g_tADS1256.ReadOver = 1;
		}
	}
}


/*
*********************************************************************************************************
*	函 数 名: EXTI3_IRQHandler
*	功能说明: 外部中断服务程序.
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_DRDY);     //调用中断处理公用函数
}

//中断服务程序中需要做的事情
//在HAL库中所有的外部中断服务函数都会调用此函数
//GPIO_Pin:中断引脚号
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    int32_t iTemp;
	uint8_t i;
    int32_t adc[8];
	int32_t volt[8];
    
    ADS1256_ISR();
    
        for (i = 0; i < ch_num; i++)
    {
        /* 从全局缓冲区读取采样结果。 采样结果是在中断服务程序中读取的。*/
        adc[i] = ADS1256_GetAdc(i);

        /* 4194303 = 2.5V , 这是理论值，实际可以根据2.5V基准的实际值进行公式矫正 */
        volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* 计算实际电压值（近似估算的），如需准确，请进行校准 */
    }
    
    for (i = 0; i < ch_num; i++)
    {
        iTemp = volt[i];    	/* 余数，uV  */
//        if (iTemp < 0)
//        {
//            //iTemp = -iTemp;
//            //printf("%d 路=%6d,(-%d.%03d %03d V) \r\n", i, adc[i], iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);
//            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
//            //LCD_ShowxNum(40,150,iTemp,8,16,0);
//            if(i == 1)
//                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
//        }
//        else
//        {
//            //printf("%d 路=%6d,( %d.%03d %03d V) \r\n", i, adc[i], iTemp/1000000, (iTemp%1000000)/1000, iTemp%1000);
//            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
//            //LCD_ShowxNum(40,150,iTemp,8,16,0);
//            if(i == 1)
//                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
//        }
        if(i == 1)
                Virtual_Osc(4,(int16_t) (iTemp/10), 0, 0, 0, 0, 0, 0, 0);
        
        __HAL_GPIO_EXTI_CLEAR_IT(PIN_DRDY);
    }   
}
