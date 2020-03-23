#ifndef __FILTERS_SYLVATICA_H__

#include <xc.h>
#include <dsp.h>

#ifndef SYLVATICA_N_CHANNELS
#define SYLVATICA_N_CHANNELS 8
#endif

#define SYLVATICA_DEC_FACT_F0 10
#define SYLVATICA_DEC_FACT_F1 10
#define SYLVATICA_DEC_FACT_F2 10
#define SYLVATICA_DEC_FACT_F3 10

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

extern FIRStruct filters_0[SYLVATICA_N_CHANNELS];
extern FIRStruct filters_1[SYLVATICA_N_CHANNELS];
extern FIRStruct filters_2[SYLVATICA_N_CHANNELS];
extern FIRStruct filters_3[SYLVATICA_N_CHANNELS];

void init_filters(void);

#ifdef	__cplusplus
}
#endif
#endif
