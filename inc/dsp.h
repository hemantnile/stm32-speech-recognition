#ifndef __DSP_H
#define __DSP_H

#include <stdint.h>

#define SAMPLES											4400
#define LPF_TAPS										1024
#define DECIMATION_FACTOR						0x00000080U		//	0x00000040U:64;		0x00000080U:128

#define RECORDING		0

#define NUMBER_OF_FRAMES		26		// 	22
#define FRAME_SIZE					400
#define ACCURACY						4		//4,3,2		2=fastest(prone to error)
#define WORDS								6

void PDM_to_PCM(uint16_t *PDMBuf, float *PCMBuf);
void cost_calc(float* PCM_buff,uint32_t* count);
void PCM_post_processing(float* PCM_buff);
void init_mfcc(void);
void calc_mfcc(const float* buffer_float);
uint32_t result_mfcc(void);

/*
#ifdef __GNUC__
//#define d(res,a,b)	__ASM volatile ("VSUB.F32 %[result],%[op1], %[op2]\n\t"	"VABS.F32 %[result], %[result]\n\t":[result] "+t" (res):[op1] "t" (a),[op2] "t" (b)	);		//res=|a-b|
#define abzf(res,f)			res=((f)>=0?(f):(0-f));		//res=|f|
#define d(res,a,b) res=(((a)-(b))>=0?((a)-(b)):(0-((a)-(b))));
#else
#define abzf(res,f)	__ASM volatile {	VABS.F32 res, f;	}		//res=|f|
#define d(res,a,b)	__ASM volatile {	VSUB.F32 res, a, b;	VABS.F32 res, res;	}		//res=|a-b|
#endif
//#define d(res,a,b) res=(((a)-(b))>=0?((a)-(b)):(-1*((a)-(b))));
//#define abzf(res,f)	__ASM volatile("VABS.F32 %[result], %[value]" : [result] "=t" (res) : [value] "t" (f));	//res=|f|
*/
//#define abzf(res,f)			res=((f)>=0?(f):(0-f));		//res=|f|
//#define d(res,a,b) res=(((a)-(b))>=0?((a)-(b)):(0-((a)-(b))));

#define abzf(res,f)	__ASM volatile {	VABS.F32 res, f;	}		//res=|f|
#define d(res,a,b)	__ASM volatile {	VSUB.F32 res, a, b;	VABS.F32 res, res;	}		//res=|a-b|

#define min3(a,b,c)		((a)<(b)?((a)<(c)?(a):(c)):((b)<(c)?(b):(c)))
#define min2(a,b)		((a)<(b)?(a):(b))
#define max2(a,b)		((a)>(b)?(a):(b))

#endif
