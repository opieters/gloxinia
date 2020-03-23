#include <dsp.h>
#include "filters_planalta.h"
#include "fir_coeffs.h"
#include "planalta.h"

FIRStruct filters_0[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_1_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_1_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_1_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_1_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_2_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_2_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_2_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_2_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_3_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_3_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_3_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_3_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_4_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_4_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_4_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_4_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_5_i[PLANALTA_50KHZ_N_ADC_CHANNELS];
FIRStruct filters_5_q[PLANALTA_50KHZ_N_ADC_CHANNELS];
FIRStruct filters_5_i[PLANALTA_50KHZ_N_ADC_CHANNELS];
FIRStruct filters_5_q[PLANALTA_50KHZ_N_ADC_CHANNELS];
FIRStruct filters_6_i[PLANALTA_25KHZ_N_ADC_CHANNELS];
FIRStruct filters_6_q[PLANALTA_25KHZ_N_ADC_CHANNELS];
FIRStruct filters_6_i[PLANALTA_25KHZ_N_ADC_CHANNELS];
FIRStruct filters_6_q[PLANALTA_25KHZ_N_ADC_CHANNELS];
FIRStruct filters_7_i[PLANALTA_10KHZ_N_ADC_CHANNELS];
FIRStruct filters_7_q[PLANALTA_10KHZ_N_ADC_CHANNELS];
FIRStruct filters_7_i[PLANALTA_10KHZ_N_ADC_CHANNELS];
FIRStruct filters_7_q[PLANALTA_10KHZ_N_ADC_CHANNELS];
FIRStruct filters_8_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_8_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_8_i[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filters_8_q[PLANALTA_5KHZ_N_ADC_CHANNELS];
FIRStruct filter_9;


fractional __attribute__((space(xmemory), aligned(64), eds)) delay_buffers_0[PLANALTA_5KHZ_N_ADC_CHANNELS][24];
fractional __attribute__((space(xmemory), aligned(64), eds)) delay_buffers_1_q[PLANALTA_5KHZ_N_ADC_CHANNELS][46];
fractional __attribute__((space(xmemory), aligned(64), eds)) delay_buffers_1_i[PLANALTA_5KHZ_N_ADC_CHANNELS][46];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_2_q[PLANALTA_5KHZ_N_ADC_CHANNELS][96];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_2_i[PLANALTA_5KHZ_N_ADC_CHANNELS][96];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_3_q[PLANALTA_5KHZ_N_ADC_CHANNELS][96];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_3_i[PLANALTA_5KHZ_N_ADC_CHANNELS][96];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_4_q[PLANALTA_5KHZ_N_ADC_CHANNELS][96];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_4_i[PLANALTA_5KHZ_N_ADC_CHANNELS][96];
fractional __attribute__((space(xmemory), aligned(1024), eds)) delay_buffers_5_q[PLANALTA_50KHZ_N_ADC_CHANNELS][380];
fractional __attribute__((space(xmemory), aligned(1024), eds)) delay_buffers_5_i[PLANALTA_50KHZ_N_ADC_CHANNELS][380];
fractional __attribute__((space(xmemory), aligned(512), eds)) delay_buffers_6_q[PLANALTA_25KHZ_N_ADC_CHANNELS][200];
fractional __attribute__((space(xmemory), aligned(512), eds)) delay_buffers_6_i[PLANALTA_25KHZ_N_ADC_CHANNELS][200];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_7_q[PLANALTA_10KHZ_N_ADC_CHANNELS][80];
fractional __attribute__((space(xmemory), aligned(256), eds)) delay_buffers_7_i[PLANALTA_10KHZ_N_ADC_CHANNELS][80];
fractional __attribute__((space(xmemory), aligned(128), eds)) delay_buffers_8_q[PLANALTA_5KHZ_N_ADC_CHANNELS][40];
fractional __attribute__((space(xmemory), aligned(128), eds)) delay_buffers_8_i[PLANALTA_5KHZ_N_ADC_CHANNELS][40];

fractional __attribute__((space(xmemory), aligned(128), eds)) delay_buffer_9[40];


void init_filters(void){
    uint16_t i;

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_0[i],
            N_FIR_COEFFS0,
            fir_coeffs_0,
            COEFFS_IN_DATA,
            delay_buffers_0[i]
        );

        FIRDelayInit(&filters_0[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_1_q[i],
            N_FIR_COEFFS1_Q,
            fir_coeffs_1_q,
            COEFFS_IN_DATA,
            delay_buffers_1_q[i]
        );

        FIRDelayInit(&filters_1_q[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_1_i[i],
            N_FIR_COEFFS1_I,
            fir_coeffs_1_i,
            COEFFS_IN_DATA,
            delay_buffers_1_i[i]
        );

        FIRDelayInit(&filters_1_i[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_2_q[i],
            N_FIR_COEFFS2,
            fir_coeffs_2,
            COEFFS_IN_DATA,
            delay_buffers_2_q[i]
        );

        FIRDelayInit(&filters_2_q[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_2_i[i],
            N_FIR_COEFFS2,
            fir_coeffs_2,
            COEFFS_IN_DATA,
            delay_buffers_2_i[i]
        );

        FIRDelayInit(&filters_2_i[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_3_q[i],
            N_FIR_COEFFS3,
            fir_coeffs_3,
            COEFFS_IN_DATA,
            delay_buffers_3_q[i]
        );

        FIRDelayInit(&filters_3_q[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_3_i[i],
            N_FIR_COEFFS3,
            fir_coeffs_3,
            COEFFS_IN_DATA,
            delay_buffers_3_i[i]
        );

        FIRDelayInit(&filters_3_i[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_4_q[i],
            N_FIR_COEFFS4,
            fir_coeffs_4,
            COEFFS_IN_DATA,
            delay_buffers_4_q[i]
        );

        FIRDelayInit(&filters_4_q[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_4_i[i],
            N_FIR_COEFFS4,
            fir_coeffs_4,
            COEFFS_IN_DATA,
            delay_buffers_4_i[i]
        );

        FIRDelayInit(&filters_4_i[i]);
    }

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_5_q[i],
            N_FIR_COEFFS5,
            fir_coeffs_5,
            COEFFS_IN_DATA,
            delay_buffers_5_q[i]
        );

        FIRDelayInit(&filters_5_q[i]);
    }

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_5_i[i],
            N_FIR_COEFFS5,
            fir_coeffs_5,
            COEFFS_IN_DATA,
            delay_buffers_5_i[i]
        );

        FIRDelayInit(&filters_5_i[i]);
    }

    for(i = 0; i < PLANALTA_25KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_6_q[i],
            N_FIR_COEFFS6,
            fir_coeffs_6,
            COEFFS_IN_DATA,
            delay_buffers_6_q[i]
        );

        FIRDelayInit(&filters_6_q[i]);
    }

    for(i = 0; i < PLANALTA_25KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_6_i[i],
            N_FIR_COEFFS6,
            fir_coeffs_6,
            COEFFS_IN_DATA,
            delay_buffers_6_i[i]
        );

        FIRDelayInit(&filters_6_i[i]);
    }

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_7_q[i],
            N_FIR_COEFFS7,
            fir_coeffs_7,
            COEFFS_IN_DATA,
            delay_buffers_7_q[i]
        );

    FIRDelayInit(&filters_7_q[i]);
    }

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_7_i[i],
            N_FIR_COEFFS7,
            fir_coeffs_7,
            COEFFS_IN_DATA,
            delay_buffers_7_i[i]
        );

        FIRDelayInit(&filters_7_i[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_8_q[i],
            N_FIR_COEFFS8,
            fir_coeffs_8,
            COEFFS_IN_DATA,
            delay_buffers_8_q[i]
        );

        FIRDelayInit(&filters_8_q[i]);
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        FIRStructInit(&filters_8_i[i],
            N_FIR_COEFFS8,
            fir_coeffs_8,
            COEFFS_IN_DATA,
            delay_buffers_8_i[i]
        );

        FIRDelayInit(&filters_8_i[i]);
    }

}
