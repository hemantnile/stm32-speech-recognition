/**
 ******************************************************************************
 * @file    inc/lookup.h 
 * @author  Hemant Nile
 * @version V1
 * @date    04-Feb-2017
 * @brief  const data tables
 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOOKUP_H
#define __LOOKUP_H

extern const float LPF_precalc[256][16];
extern const float hamming_window_400[400];
extern const float mel_filterbank_257x26_div400[6682];
extern const float dct_table_26x12[312];
extern const int frame_base[44];
extern const float mfcc_templates[24][312];

#endif /* __LOOKUP_H */
