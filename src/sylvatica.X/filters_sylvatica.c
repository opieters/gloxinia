#include <dsp.h>
#include "filters_sylvatica.h"
#include "fir_coeffs.h"
#include "sylvatica.h"

FIRStruct filters_0[SYLVATICA_N_CHANNELS];
FIRStruct filters_1[SYLVATICA_N_CHANNELS];
FIRStruct filters_2[SYLVATICA_N_CHANNELS];
FIRStruct filters_3[SYLVATICA_N_CHANNELS];


fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_0[SYLVATICA_N_CHANNELS][100];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_1[SYLVATICA_N_CHANNELS][100];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_2[SYLVATICA_N_CHANNELS][100];
fractional __attribute__((space(xmemory), aligned(512), eds)) delay_buffers_3[SYLVATICA_N_CHANNELS][192];


void init_filters(void){
    uint16_t i;

    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        FIRStructInit(&filters_0[i],
            N_FIR_COEFFS0,
            fir_coeffs_0,
            COEFFS_IN_DATA,
            delay_buffers_0[i]
        );

        FIRDelayInit(&filters_0[i]);
    }

    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        FIRStructInit(&filters_1[i],
            N_FIR_COEFFS1,
            fir_coeffs_1,
            COEFFS_IN_DATA,
            delay_buffers_1[i]
        );

        FIRDelayInit(&filters_1[i]);
    }

    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        FIRStructInit(&filters_2[i],
            N_FIR_COEFFS2,
            fir_coeffs_2,
            COEFFS_IN_DATA,
            delay_buffers_2[i]
        );

        FIRDelayInit(&filters_2[i]);
    }

    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        FIRStructInit(&filters_3[i],
            N_FIR_COEFFS3,
            fir_coeffs_3,
            COEFFS_IN_DATA,
            delay_buffers_3[i]
        );

        FIRDelayInit(&filters_3[i]);
    }

}
