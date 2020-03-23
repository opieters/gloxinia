#include "planalta_filtering.h"
#include "filters_planalta.h"
#include <adc.h>
#include <fir_common.h>
#include <dsp.h>
#include "planalta.h"
#include <stdbool.h>

volatile uint8_t copy_buffer_selector = 0;
uint8_t adc_buffer_selector = 0;

volatile uint8_t start_filter_block0 = 0;
uint8_t start_filter2 = 0, start_filter3 = 0;
uint8_t start_filter4 = 0, start_filter5 = 0, start_filter6 = 0;

#define PLANALTA_F2_INPUT_SIZE MAX( \
            MAX(PLANALTA_5KHZ_F2_INPUT_SIZE, PLANALTA_10KHZ_F2_INPUT_SIZE),\
            MAX(PLANALTA_25KHZ_F2_INPUT_SIZE, PLANALTA_50KHZ_F2_INPUT_SIZE))
#define PLANALTA_F3_INPUT_SIZE MAX(\
            MAX(PLANALTA_5KHZ_F3_INPUT_SIZE, PLANALTA_10KHZ_F3_INPUT_SIZE),\
            MAX(PLANALTA_25KHZ_F3_INPUT_SIZE, PLANALTA_50KHZ_F3_INPUT_SIZE))
#define PLANALTA_F4_INPUT_SIZE MAX(\
            MAX(PLANALTA_5KHZ_F4_INPUT_SIZE, PLANALTA_10KHZ_F4_INPUT_SIZE),\
            MAX(PLANALTA_25KHZ_F4_INPUT_SIZE, PLANALTA_50KHZ_F4_INPUT_SIZE))
#define PLANALTA_FX_INPUT_SIZE MAX(\
            MAX(PLANALTA_5KHZ_FX_INPUT_SIZE, PLANALTA_10KHZ_FX_INPUT_SIZE),\
            MAX(PLANALTA_25KHZ_FX_INPUT_SIZE, PLANALTA_50KHZ_FX_INPUT_SIZE))

fractional* fo2_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo2_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo3_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo3_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo4_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo4_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo5_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
fractional* fo5_buffer_q_read[PLANALTA_N_ADC_CHANNELS];

fractional* fo1_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo1_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo2_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo2_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo3_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo3_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo4_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
fractional* fo4_buffer_q_write[PLANALTA_N_ADC_CHANNELS];


uint8_t select_f1_to_f2, select_f2_to_f3, select_f3_to_f4;

fractional f1_to_f2_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_F2_INPUT_SIZE];
fractional f1_to_f2_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_F2_INPUT_SIZE];
fractional f2_to_f3_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_F3_INPUT_SIZE];
fractional f2_to_f3_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_F3_INPUT_SIZE];
fractional f1_to_f2_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_F2_INPUT_SIZE];
fractional f1_to_f2_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_F2_INPUT_SIZE];
fractional f2_to_f3_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_F3_INPUT_SIZE];
fractional f2_to_f3_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_F3_INPUT_SIZE];
fractional f3_to_f4_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_F4_INPUT_SIZE];
fractional f3_to_f4_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_F4_INPUT_SIZE];
fractional f4_to_fx_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_FX_INPUT_SIZE];
fractional f4_to_fx_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_FX_INPUT_SIZE];
fractional f3_to_f4_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_F4_INPUT_SIZE];
fractional f3_to_f4_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_F4_INPUT_SIZE];
fractional f4_to_fx_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_FX_INPUT_SIZE];
fractional f4_to_fx_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_FX_INPUT_SIZE];
fractional output_buffer_a_i[PLANALTA_N_ADC_CHANNELS];
fractional output_buffer_a_q[PLANALTA_N_ADC_CHANNELS];
fractional output_buffer_b_i[PLANALTA_N_ADC_CHANNELS];
fractional output_buffer_b_q[PLANALTA_N_ADC_CHANNELS];

bool output_buffer_full = false;

//uint16_t n_writes_output_buffer_a[PLANALTA_N_ADC_CHANNELS];
//uint16_t n_writes_output_buffer_b[PLANALTA_N_ADC_CHANNELS];

     
void init_filtering(void){
    uint16_t i;
    output_buffer_full = false;
    
    for(i = 0; i < PLANALTA_N_ADC_CHANNELS; i++){
        fo2_buffer_i_read[i] = f1_to_f2_buffer_i_b[i];
        fo2_buffer_q_read[i] = f1_to_f2_buffer_q_b[i];
        fo3_buffer_i_read[i] = f2_to_f3_buffer_i_b[i];
        fo3_buffer_q_read[i] = f2_to_f3_buffer_q_b[i];
        fo4_buffer_i_read[i] = f3_to_f4_buffer_i_b[i];
        fo4_buffer_q_read[i] = f3_to_f4_buffer_q_b[i];
        
        fo1_buffer_i_write[i] = f1_to_f2_buffer_i_a[i];
        fo1_buffer_q_write[i] = f1_to_f2_buffer_q_a[i];
        fo2_buffer_i_write[i] = f2_to_f3_buffer_i_a[i];
        fo2_buffer_q_write[i] = f2_to_f3_buffer_q_a[i];
        fo3_buffer_i_write[i] = f3_to_f4_buffer_i_a[i];
        fo3_buffer_q_write[i] = f3_to_f4_buffer_q_a[i];
        //fo4_buffer_i_write[i] = f4_to_f5_buffer_i_a[i];
        //fo4_buffer_q_write[i] = f4_to_f5_buffer_q_a[i];
    }
    select_f1_to_f2 = 0;
    select_f2_to_f3 = 0;
    select_f3_to_f4 = 0;
}

void planalta_clear_filter_buffers(void){
    uint16_t i, j;
    
    init_filtering();
    
    // todo: clear delay buffers
    
    for(i = 0; i < PLANALTA_N_ADC_CHANNELS; i++){
        for(j = 0; j < PLANALTA_F2_INPUT_SIZE; j++){
            f1_to_f2_buffer_i_a[i][j] = 0;
            f1_to_f2_buffer_q_a[i][j] = 0;
            
            f1_to_f2_buffer_i_b[i][j] = 0;
            f1_to_f2_buffer_q_b[i][j] = 0;
        }
    }
    for(i = 0; i < PLANALTA_N_ADC_CHANNELS; i++){
        for(j = 0; j < PLANALTA_F3_INPUT_SIZE; j++){
            f2_to_f3_buffer_i_a[i][j] = 0;
            f2_to_f3_buffer_q_a[i][j] = 0;
            
            f2_to_f3_buffer_i_b[i][j] = 0;
            f2_to_f3_buffer_q_b[i][j] = 0;
        }
    }
    for(i = 0; i < PLANALTA_N_ADC_CHANNELS; i++){
        for(j = 0; j < PLANALTA_F4_INPUT_SIZE; j++){
            f3_to_f4_buffer_i_a[i][j] = 0;
            f3_to_f4_buffer_q_a[i][j] = 0;
            
            f3_to_f4_buffer_i_b[i][j] = 0;
            f3_to_f4_buffer_q_b[i][j] = 0;
        }
    }
    for(i = 0; i < PLANALTA_N_ADC_CHANNELS; i++){
        for(j = 0; j < PLANALTA_FX_INPUT_SIZE; j++){
            f4_to_fx_buffer_i_a[i][j] = 0;
            f4_to_fx_buffer_i_a[i][j] = 0;
            
            f4_to_fx_buffer_i_b[i][j] = 0;
            f4_to_fx_buffer_i_b[i][j] = 0;
        }
    }
    for(i = 0; i < PLANALTA_N_ADC_CHANNELS; i++){
        output_buffer_a_i[i] = 0;
        output_buffer_b_i[i] = 0;
        output_buffer_a_q[i] = 0;
        output_buffer_b_q[i] = 0;
    }
}
