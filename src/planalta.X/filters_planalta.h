#ifndef __FILTERS_PLANALTA_H__

#include <xc.h>
#include <dsp.h>

#ifndef PLANALTA_50KHZ_N_ADC_CHANNELS
#define PLANALTA_50KHZ_N_ADC_CHANNELS  1
#endif

#ifndef PLANALTA_25KHZ_N_ADC_CHANNELS
#define PLANALTA_25KHZ_N_ADC_CHANNELS  2
#endif

#ifndef PLANALTA_10KHZ_N_ADC_CHANNELS
#define PLANALTA_10KHZ_N_ADC_CHANNELS  4
#endif

#ifndef PLANALTA_5KHZ_N_ADC_CHANNELS
#define PLANALTA_5KHZ_N_ADC_CHANNELS  8
#endif

#define PLANALTA_DEC_FACT_F0 1
// F1 has a decimation factor of 10, but due to the mixing optimisation, the 
// effective rate becomes 5
#define PLANALTA_DEC_FACT_F1 5
#define PLANALTA_DEC_FACT_F2 10
#define PLANALTA_DEC_FACT_F3 10
#define PLANALTA_DEC_FACT_F4 10
#define PLANALTA_DEC_FACT_F5 20
#define PLANALTA_DEC_FACT_F6 10
#define PLANALTA_DEC_FACT_F7 4
#define PLANALTA_DEC_FACT_F8 2
#define PLANALTA_DEC_FACT_F9 5

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

extern FIRStruct filters_0[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filter_9;
extern FIRStruct filters_1_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_1_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_2_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_2_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_3_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_3_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_4_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_4_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_5_q[PLANALTA_50KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_5_i[PLANALTA_50KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_6_q[PLANALTA_25KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_6_i[PLANALTA_25KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_7_q[PLANALTA_10KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_7_i[PLANALTA_10KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_8_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
extern FIRStruct filters_8_i[PLANALTA_5KHZ_N_ADC_CHANNELS];

void init_filters(void);

#ifdef	__cplusplus
}
#endif
#endif
