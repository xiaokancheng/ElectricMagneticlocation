#ifndef __AD9851_H
#define __AD9851_H
#include "sys.h"

#define parrel 0
#define serial 1

#define ad9851_w_clk     PCout(8)	
#define ad9851_fq_up     PCout(7)	
#define ad9851_rest      PCout(6)
#define ad9851_bit_data  PDout(7)


void Ad9850_Init(u8 mode, u16 frequency);
void ad9851_Init(void);
void ad9851_reset_parrel(void);
void ad9851_reset_serial(void);
void ad9851_wr_parrel(unsigned char w0,double frequence);
void ad9851_wr_serial(unsigned char w0,double frequence);


extern u8 data1;
extern u8 data2;
extern u8 data3;
extern u8 data4;
extern u8 data5;
#endif


