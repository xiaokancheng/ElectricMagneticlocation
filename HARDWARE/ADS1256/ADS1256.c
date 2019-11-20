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

////��ʼ��ADS1256 GPIO
void bsp_InitADS1256(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef  SOFT_SPI
    
	RESET_1();
	PWDN_1();
	CS_1();
	SCK_0();		/* SPI���߿���ʱ�������ǵ͵�ƽ */
	DI_1();
    
    
    __HAL_RCC_GPIOB_CLK_ENABLE();			                //ʹ��GPIOBʱ��

    GPIO_InitStructure.Pin = PIN_SCK | PIN_DIN | PIN_CS | PIN_RESET | PIN_PWDN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;          //��ͨ�������ģʽ
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP;                  //����
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);              //��ʼ��GPIO

    GPIO_InitStructure.Pin = PIN_DOUT | PIN_DRDY;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;          //��ͨ�������ģʽ
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP;                  //����
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);              //��ʼ��GPIO

#endif
}

void bsp_Cfg_ADS1256(u8 mode, ADS1256_GAIN_E ADS1256_GAIN, ADS1256_DRATE_E ADS1256_SPS)
{

    if(mode == 0)
    {
		printf("\r\nPGA���� = %d, ����������� = %d, ����8·ɨ��\r\n\r\n",ADS1256_GAIN, ADS1256_SPS);
		
		ADS1256_CfgADC(ADS1256_GAIN, ADS1256_SPS);	/* ����ADC������ ����1:1, ����������� 15Hz */

		/* 		
		   �жϷ��������Զ���ȡADC���������ȫ�ֱ�����������ͨ�� ADS1256_GetAdc() ��������ȡ��Щ����
		*/
		ADS1256_StartScan(mode);	/* �����ж�ɨ��ģʽ. 0��ʾ����8·��1��ʾ���4· */
		ch_num = 8;		/* ͨ���� = 8 ����4 */
    }
    else
    {
        printf("\r\nPGA���� = %d, ����������� = %d, ���4·ɨ��\r\n\r\n",ADS1256_GAIN, ADS1256_SPS);
		
		ADS1256_CfgADC(ADS1256_GAIN, ADS1256_SPS);	/* ����ADC������ ����1:1, ����������� 15Hz */

		/* 		
		   �жϷ��������Զ���ȡADC���������ȫ�ֱ�����������ͨ�� ADS1256_GetAdc() ��������ȡ��Щ����
		*/
		ADS1256_StartScan(mode);	/* �����ж�ɨ��ģʽ. 0��ʾ����8·��1��ʾ���4· */
		ch_num = 4;		/* ͨ���� = 8 ����4 */	
    }
}


/*
*********************************************************************************************************
*	�� �� ��: ADS1256_CfgADC
*	����˵��: ����ADC����������������������
*	��    ��: _gain : ����
*			  _drate : �����������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_StopScan();			/* ��ͣCPU�ж� */

	ADS1256_ResetHard();		/* Ӳ����λ */

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* �ݴ�ADS1256 �Ĵ������ò�����֮������д4���Ĵ��� */

		/* ״̬�Ĵ�������
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

			ACAL=1ʹ����У׼���ܡ��� PGA��BUFEEN, DRATE�ı�ʱ��������У׼
		*/
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	/* ����λ0��ʾAINP�� AIN0,  ����λ8��ʾ AINN �̶��� AINCOM */

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
				01 = Sensor Detect Current = 0.5 �� A
				10 = Sensor Detect Current = 2 �� A
				11 = Sensor Detect Current = 10�� A
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
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 3) | (GAIN_1 << 0));	/* ѡ��1;1����, ��������5V */

		/* ��Ϊ�л�ͨ���Ͷ����ݺ�ʱ 123uS, ���ɨ���ж�ģʽ����ʱ��������� = DRATE_1000SPS */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* ѡ������������� */

		CS_0();	/* SPIƬѡ = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* д�Ĵ���������, �����ͼĴ�����ַ */
		ADS1256_Send8Bit(0x03);			/* �Ĵ������� - 1, �˴�3��ʾд4���Ĵ��� */

		ADS1256_Send8Bit(buf[0]);	/* ����״̬�Ĵ��� */
		ADS1256_Send8Bit(buf[1]);	/* ��������ͨ������ */
		ADS1256_Send8Bit(buf[2]);	/* ����ADCON���ƼĴ��������� */
		ADS1256_Send8Bit(buf[3]);	/* ��������������� */

		CS_1();	/* SPIƬѡ = 1 */
	}

	delay_us(50);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_DelaySCLK
*	����˵��: CLK֮����ӳ٣�ʱ���ӳ�. ����STM32F429  180M��Ƶ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_DelaySCLK(void)
{
	uint16_t i;

	/*
		ȡ 5 ʱ��ʵ��ߵ�ƽ200ns, �͵�ƽ250ns <-- ���ȶ�
		ȡ 10 ���ϣ��������������� �͵�ƽ400ns �߶�400ns <--- �ȶ�
	*/
	for (i = 0; i < 15; i++);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_DelayDATA
*	����˵��: ��ȡDOUT֮ǰ���ӳ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		��С 50 ��tCLK = 50 * 0.13uS = 6.5uS
	*/
	delay_us(10);	/* ��С�ӳ� 6.5uS, �˴�ȡ10us */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ResetHard
*	����˵��: Ӳ����λ ADS1256оƬ.�͵�ƽ��λ�����4��ʱ�ӣ�Ҳ���� 4x0.13uS = 0.52uS
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_ResetHard(void)
{
	/* ADS1256�����ֲ��7ҳ */
	RESET_0();			/* ��λ */
	delay_us(5);
	RESET_1();

	//PWDN_0();			/* ������� ͬ��*/
	//delay_us(2);
	//PWDN_1();			/* �˳����� */

	delay_us(5);

	//ADS1256_WaitDRDY();	/* �ȴ� DRDY��Ϊ0, �˹���ʵ��: 630us */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Send8Bit
*	����˵��: ��SPI���߷���8��bit���ݡ� ����CS���ơ�
*	��    ��: _data : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{
	uint8_t i;

	/* �������Ͷ���ֽ�ʱ����Ҫ�ӳ�һ�� */
	ADS1256_DelaySCLK();
	ADS1256_DelaySCLK();

	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
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
		SCK_0();			/* <----  ADS1256 ����SCK�½��ز���DIN����, ���ݱ���ά�� 50nS */
		ADS1256_DelaySCLK();
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Recive8Bit
*	����˵��: ��SPI���߽���8��bit���ݡ� ����CS���ơ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t i;
	uint8_t read = 0;

	ADS1256_DelaySCLK();
	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
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
*	�� �� ��: ADS1256_WriteReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);		/* �Ĵ������� - 1, �˴�д1���Ĵ��� */

	ADS1256_Send8Bit(_RegValue);	/* ���ͼĴ���ֵ */
	CS_1();	/* SPIƬѡ = 1 */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ��
*	�� �� ֵ: �����ļĴ���ֵ��
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);	/* �Ĵ������� - 1, �˴���1���Ĵ��� */

	ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */

	read = ADS1256_Recive8Bit();	/* ���Ĵ���ֵ */
	CS_1();	/* SPIƬѡ = 1 */

	return read;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WriteCmd
*	����˵��: ���͵��ֽ�����
*	��    ��:  _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPIƬѡ = 1 */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadChipID
*	����˵��: ��оƬID, ��״̬�Ĵ����еĸ�4bit
*	��    ��: ��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
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
*	�� �� ��: ADS1256_SetChannal
*	����˵��: ����ͨ���š���·���á�AIN- �̶��ӵأ�ACOM).
*	��    ��: _ch : ͨ���ţ� 0-7
*	�� �� ֵ: ��
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
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

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
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN �̶��� AINCOM */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetDiffChannal
*	����˵��: ���ò��ͨ���š���·���á�
*	��    ��: _ch : ͨ����,0-3����4��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
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
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

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
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* ������� AIN0�� AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* ������� AIN2�� AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* ������� AIN4�� AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* ������� AIN6�� AIN7 */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WaitDRDY
*	����˵��: �ȴ��ڲ�������ɡ� ��У׼ʱ��ϳ�����Ҫ�ȴ���
*	��    ��: ��
*	�� �� ֵ: ��
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
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* �������. �����Ŵ� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadData
*	����˵��: ��ADC����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;

	CS_0();	/* SPIƬѡ = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* �����ݵ����� */

	ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */

	/* �����������3���ֽڣ����ֽ���ǰ */
	read = ADS1256_Recive8Bit() << 16;
	read += (ADS1256_Recive8Bit() << 8);
	read += ADS1256_Recive8Bit();

	CS_1();	/* SPIƬѡ = 1 */

	/* ����������չ��24λ�з�������չΪ32λ�з����� */
	if (read & 0x800000)
	{
		read += 0xFF000000;
	}

	return (int32_t)read;
}

#if 0
/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadAdc
*	����˵��: ��ָ��ͨ����ADC����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t ADS1256_ReadAdc(uint8_t _ch)
{
	/* ADS1256 �����ֲ��21ҳ */

#if 0	/* ����30Ksps �������� */
	int32_t read;

	while (DRDY_IS_LOW());	/* �ȴ� DRDY �� */
	while (!DRDY_IS_LOW());	/* �ȴ� DRDY �� */

	ADS1256_SetChannal(_ch);	/* �л�ģ��ͨ�� */
	delay_us(5);

	ADS1256_WriteCmd(CMD_SYNC);
	delay_us(5);

	ADS1256_WriteCmd(CMD_WAKEUP);  /* ��������£����ʱ�� DRDY �Ѿ�Ϊ�� */
	delay_us(25);

	read =  (int32_t)ADS1256_ReadData();

	while (DRDY_IS_LOW());	/* �ȴ� DRDY �� */
	while (!DRDY_IS_LOW());	/* �ȴ� DRDY �� */

	read =  (int32_t)ADS1256_ReadData();

	return read;
#else
	//while (DRDY_IS_LOW());

	/* ADS1256 �����ֲ��21ҳ */
	ADS1256_WaitDRDY();		/* �ȴ� DRDY = 0 */

	ADS1256_SetChannal(_ch);	/* �л�ģ��ͨ�� */
	delay_us(5);

	ADS1256_WriteCmd(CMD_SYNC);
	delay_us(5);

	ADS1256_WriteCmd(CMD_WAKEUP);
	delay_us(25);

	//ADS1256_WaitDRDY();		/* �ȴ� DRDY = 0 */

	return (int32_t)ADS1256_ReadData();
#endif
}

#endif


/*
*********************************************************************************************************
*	����ĺ�������DRDY�жϹ���ģʽ
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_StartScan
*	����˵��: �� DRDY���� ��PH9 �����ó��ⲿ�жϴ�����ʽ�� �жϷ��������ɨ��8��ͨ�������ݡ�
*	��    ��: _ucDiffMode : 0 ��ʾ����ģʽ��ɨ��8·���� 1��ʾ���ģʽ��ɨ��4·
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode)
{
	g_tADS1256.ScanMode = _ucScanMode;
	/* ��ʼɨ��ǰ, ������������ */
	{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; i++)
		{
			g_tADS1256.AdcNow[i] = 0;
		}
	}

	/* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();             //����GPIOBʱ��
    
	/* Configure EXTI3 line */
    GPIO_Initure.Pin=PIN_DRDY;                //PB6
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //�½��ش���
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
*	�� �� ��: ADS1256_StopScan
*	����˵��: ֹͣ DRDY �ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_StopScan(void)
{
	/* Configure EXTI3 line */
    GPIO_Initure.Pin=PIN_DRDY;                //PB6
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //�½��ش���
    GPIO_Initure.Pull=GPIO_PULLUP;
    
    /* Connect EXTI3 Line to PB6 pin */
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    __HAL_GPIO_EXTI_CLEAR_IT(PIN_DRDY);
    
	/* �ж����ȼ����� ������ȼ� ����һ��Ҫ�ֿ��������жϣ����ܹ��ϲ���һ���������� */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_GetAdc
*	����˵��: �ӻ�������ȡADC��������������ṹ�����жϷ���������ġ�
*	��    ��: _ch ͨ���� (0 - 7)
*	�� �� ֵ: ADC�ɼ�������з�������
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	__set_PRIMASK(1);	/* ��ֹ�ж� */

	iTemp = g_tADS1256.AdcNow[_ch];

	__set_PRIMASK(0);	/* ʹ���ж� */

	return iTemp;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ISR
*	����˵��: ��ʱ�ɼ��жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	if (g_tADS1256.ScanMode == 0)	/* 0 ��ʾ����8·ɨ�裬1��ʾ���4·ɨ�� */
	{
		/* ��ȡ�ɼ��ṹ��������ȫ�ֱ��� */
		ADS1256_SetChannal(g_tADS1256.Channel);	/* �л�ģ��ͨ�� */
		delay_us(5);

		ADS1256_WriteCmd(CMD_SYNC);
		delay_us(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		delay_us(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}

		if (++g_tADS1256.Channel >= 8)
		{
			g_tADS1256.Channel = 0;
		}
	}
	else	/* ���4·ɨ�� */
	{
		/* ��ȡ�ɼ��ṹ��������ȫ�ֱ��� */
		ADS1256_SetDiffChannal(g_tADS1256.Channel);	/* �л�ģ��ͨ�� */
		delay_us(5);

		ADS1256_WriteCmd(CMD_SYNC);
		delay_us(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		delay_us(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[3] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
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
*	�� �� ��: EXTI3_IRQHandler
*	����˵��: �ⲿ�жϷ������.
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_DRDY);     //�����жϴ����ú���
}

//�жϷ����������Ҫ��������
//��HAL�������е��ⲿ�жϷ�����������ô˺���
//GPIO_Pin:�ж����ź�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    int32_t iTemp;
	uint8_t i;
    int32_t adc[8];
	int32_t volt[8];
    
    ADS1256_ISR();
    
        for (i = 0; i < ch_num; i++)
    {
        /* ��ȫ�ֻ�������ȡ��������� ������������жϷ�������ж�ȡ�ġ�*/
        adc[i] = ADS1256_GetAdc(i);

        /* 4194303 = 2.5V , ��������ֵ��ʵ�ʿ��Ը���2.5V��׼��ʵ��ֵ���й�ʽ���� */
        volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* ����ʵ�ʵ�ѹֵ�����ƹ���ģ�������׼ȷ�������У׼ */
    }
    
    for (i = 0; i < ch_num; i++)
    {
        iTemp = volt[i];    	/* ������uV  */
//        if (iTemp < 0)
//        {
//            //iTemp = -iTemp;
//            //printf("%d ·=%6d,(-%d.%03d %03d V) \r\n", i, adc[i], iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);
//            //LCD_ShowString(30,130,200,16,16,"Value of Current:");
//            //LCD_ShowxNum(40,150,iTemp,8,16,0);
//            if(i == 1)
//                Virtual_Osc(4,(int16_t) iTemp, 0, 0, 0, 0, 0, 0, 0);
//        }
//        else
//        {
//            //printf("%d ·=%6d,( %d.%03d %03d V) \r\n", i, adc[i], iTemp/1000000, (iTemp%1000000)/1000, iTemp%1000);
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
