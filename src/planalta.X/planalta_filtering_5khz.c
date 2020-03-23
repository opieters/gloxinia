#include "planalta_filtering.h"
#include <fir_common.h>
#include <dsp.h>
#include "planalta.h"
#include <utilities.h>

extern volatile uint8_t copy_buffer_selector;
extern uint8_t adc_buffer_selector;

extern volatile uint8_t start_filter_block0;
extern uint8_t start_filter2, start_filter3, start_filter4, start_filter5;

extern fractional* fo1_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo1_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_q_write[PLANALTA_N_ADC_CHANNELS];

extern fractional* fo2_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo3_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo3_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo4_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo4_buffer_q_write[PLANALTA_N_ADC_CHANNELS];

extern fractional* fo2_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo3_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo3_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo4_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo4_buffer_q_read[PLANALTA_N_ADC_CHANNELS];

extern uint8_t select_f1_to_f2, select_f2_to_f3, select_f3_to_f4;

extern fractional f1_to_f2_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f1_to_f2_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f2_to_f3_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f2_to_f3_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f1_to_f2_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f1_to_f2_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f2_to_f3_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f2_to_f3_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f3_to_f4_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F4_INPUT_SIZE];
extern fractional f3_to_f4_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F4_INPUT_SIZE];
extern fractional f4_to_fx_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_FX_INPUT_SIZE];
extern fractional f4_to_fx_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_FX_INPUT_SIZE];
extern fractional f3_to_f4_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F4_INPUT_SIZE];
extern fractional f3_to_f4_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F4_INPUT_SIZE];
extern fractional f4_to_fx_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_FX_INPUT_SIZE];
extern fractional f4_to_fx_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_FX_INPUT_SIZE];
extern fractional output_buffer_a_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_a_q[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_q[PLANALTA_N_ADC_CHANNELS];

extern planalta_config_t gconfig;

extern uint8_t output_buffer_full;

void adc_rx_callback_5khz(void){
    uint16_t i = 0;
    static uint16_t copy_counter = 0;
    fractional conversion_buffer[PLANALTA_5KHZ_F0_OUTPUT_SIZE];
    fractional sample_buffer[PLANALTA_5KHZ_F0_OUTPUT_SIZE];

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        // convert samples to fractional format and split the different channels
        //  from this point onwards, all channels are processed one by one
        if (adc_buffer_selector == 0) {
            copy_uint_to_fract(PLANALTA_5KHZ_F0_OUTPUT_SIZE,
                    (fractional*) &adc_rx_buffer_a[i],
                    conversion_buffer,
                    PLANALTA_5KHZ_N_ADC_CHANNELS);
        } else {
            copy_uint_to_fract(PLANALTA_5KHZ_F0_OUTPUT_SIZE,
                    (fractional*) &adc_rx_buffer_b[i],
                    conversion_buffer,
                    PLANALTA_5KHZ_N_ADC_CHANNELS);
        }

        // FO0
        fir_compressed(PLANALTA_5KHZ_F0_OUTPUT_SIZE,
            sample_buffer,
            conversion_buffer,
            &filters_0[i],
            PLANALTA_DEC_FACT_F0);

        // mixing operation: 1/2 of output samples are 0 and thus not stored in
        // the buffer to save space
        lia_mixer_no_dc(PLANALTA_5KHZ_F0_OUTPUT_SIZE,
            sample_buffer,
            conversion_buffer,
            &conversion_buffer[PLANALTA_5KHZ_F1_INPUT_SIZE]);

        // FO1
        // due to not storing the zeros, this filter has a modified layout:
        // decimation factor is half of the expected value since the output
        // already got "decimated" by the mixing operation. Additionally, not
        // all coefficients are stored since some will be multiplied with a 0. 
        // Finally, the "actual" decimation factor is half of the expected one,
        // thus requiring the decimation factor to always be greater than or 
        // equal to 2.
        fir_compressed(PLANALTA_5KHZ_F1_OUTPUT_SIZE,
                fo1_buffer_i_write[i],
                conversion_buffer,
                &filters_1_i[i],
                PLANALTA_DEC_FACT_F1);
        fir_compressed(PLANALTA_5KHZ_F1_OUTPUT_SIZE,
                fo1_buffer_q_write[i],
                &conversion_buffer[PLANALTA_5KHZ_F1_INPUT_SIZE],
                &filters_1_q[i],
                PLANALTA_DEC_FACT_F1);
        
        fo1_buffer_i_write[i] += PLANALTA_5KHZ_F1_OUTPUT_SIZE;
        fo1_buffer_q_write[i] += PLANALTA_5KHZ_F1_OUTPUT_SIZE;
    }

    copy_counter += PLANALTA_5KHZ_F1_OUTPUT_SIZE;
    
    if(copy_counter == PLANALTA_5KHZ_F2_INPUT_SIZE){
        start_filter2 = 1;
        
        copy_counter = 0;
        
        select_f1_to_f2 ^= 1;
        
        if(select_f1_to_f2){
            for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
                fo2_buffer_i_read[i] = f1_to_f2_buffer_i_b[i];
                fo2_buffer_q_read[i] = f1_to_f2_buffer_q_b[i];
                fo1_buffer_i_write[i] = f1_to_f2_buffer_i_a[i];
                fo1_buffer_q_write[i] = f1_to_f2_buffer_q_a[i];
            }
        } else {
            for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
                fo2_buffer_i_read[i] = f1_to_f2_buffer_i_a[i];
                fo2_buffer_q_read[i] = f1_to_f2_buffer_q_a[i];
                fo1_buffer_i_write[i] = f1_to_f2_buffer_i_b[i];
                fo1_buffer_q_write[i] = f1_to_f2_buffer_q_b[i];
            }
        }
    }
    adc_buffer_selector ^= 1;

}

void run_filter2_5khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter2 = 0;

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_5KHZ_F2_OUTPUT_SIZE,
                &fo2_buffer_i_write[i][block_counter],
                fo2_buffer_i_read[i],
                &filters_2_i[i],
                PLANALTA_DEC_FACT_F2);
        fir_compressed(PLANALTA_5KHZ_F2_OUTPUT_SIZE,
                fo2_buffer_q_write[i],
                fo2_buffer_q_read[i],
                &filters_2_q[i],
                PLANALTA_DEC_FACT_F2);
        
        fo2_buffer_i_write[i] += PLANALTA_5KHZ_F2_OUTPUT_SIZE;
        fo2_buffer_q_write[i] += PLANALTA_5KHZ_F2_OUTPUT_SIZE;
    }
    
    block_counter += PLANALTA_5KHZ_F2_OUTPUT_SIZE;
    if(block_counter == PLANALTA_5KHZ_F3_INPUT_SIZE){
        start_filter3 = 1;
        block_counter = 0;
        
        select_f2_to_f3 ^= 1;
        
        if(select_f2_to_f3){
            for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
                fo3_buffer_i_read[i] = f2_to_f3_buffer_i_b[i];
                fo3_buffer_q_read[i] = f2_to_f3_buffer_q_b[i];
                fo2_buffer_i_write[i] = f2_to_f3_buffer_i_a[i];
                fo2_buffer_q_write[i] = f2_to_f3_buffer_q_a[i];
            }
        } else {
            for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
                fo3_buffer_i_read[i] = f2_to_f3_buffer_i_a[i];
                fo3_buffer_q_read[i] = f2_to_f3_buffer_q_a[i];
                fo2_buffer_i_write[i] = f2_to_f3_buffer_i_b[i];
                fo2_buffer_q_write[i] = f2_to_f3_buffer_q_b[i];
            }
        }
    }
}
void run_filter3_5khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter3 = 0;

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_5KHZ_F3_OUTPUT_SIZE,
                &fo3_buffer_i_write[i][block_counter],
                fo3_buffer_i_read[i],
                &filters_3_i[i],
                PLANALTA_DEC_FACT_F3);

        fir_compressed(PLANALTA_5KHZ_F3_OUTPUT_SIZE,
                &fo3_buffer_q_write[i][block_counter],
                fo3_buffer_q_read[i],
                &filters_3_q[i],
                PLANALTA_DEC_FACT_F3);
    }
    block_counter += PLANALTA_5KHZ_F2_OUTPUT_SIZE;
    if(block_counter == PLANALTA_5KHZ_F3_INPUT_SIZE){
        start_filter4 = 1;
        block_counter = 0;
        
        select_f3_to_f4 ^= 1;
        
        if(select_f3_to_f4){
            for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
                fo4_buffer_i_read[i] = f3_to_f4_buffer_i_b[i];
                fo4_buffer_q_read[i] = f3_to_f4_buffer_q_b[i];
                fo3_buffer_i_write[i] = f3_to_f4_buffer_i_a[i];
                fo3_buffer_q_write[i] = f3_to_f4_buffer_q_a[i];
            }
        } else {
            for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
                fo4_buffer_i_read[i] = f3_to_f4_buffer_i_a[i];
                fo4_buffer_q_read[i] = f3_to_f4_buffer_q_a[i];
                fo3_buffer_i_write[i] = f3_to_f4_buffer_i_b[i];
                fo3_buffer_q_write[i] = f3_to_f4_buffer_q_b[i];
            }
        }
    }
}

void run_filter4_5khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter4 = 0;

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_5KHZ_F4_OUTPUT_SIZE,
                &fo4_buffer_i_write[i][block_counter],
                fo4_buffer_i_read[i],
                &filters_4_i[i],
                PLANALTA_DEC_FACT_F4);

        fir_compressed(PLANALTA_5KHZ_F4_OUTPUT_SIZE,
                &fo4_buffer_q_write[i][block_counter],
                fo4_buffer_q_read[i],
                &filters_4_q[i],
                PLANALTA_DEC_FACT_F4);
    }
    block_counter += PLANALTA_5KHZ_F4_OUTPUT_SIZE;
    if(block_counter == PLANALTA_5KHZ_FX_INPUT_SIZE){
        block_counter = 0;
        start_filter5 = 1;
    }
}


void run_filter5_5khz(void){
    uint16_t i;
    start_filter5 = 0; 
    
    if(!output_buffer_full){
        for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
            output_buffer_b_i[i] = output_buffer_a_i[i];
            output_buffer_b_q[i] = output_buffer_a_q[i];
        }
        output_buffer_full = true;
    }

    for(i = 0; i < PLANALTA_5KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_5KHZ_FX_OUTPUT_SIZE,
                &output_buffer_a_i[i],
                fo4_buffer_i_read[i],
                &filters_8_i[i],
                PLANALTA_DEC_FACT_F8);

        fir_compressed(PLANALTA_5KHZ_FX_OUTPUT_SIZE,
                &output_buffer_a_q[i],
                fo4_buffer_q_read[i],
                &filters_8_q[i],
                PLANALTA_DEC_FACT_F8);
    }
    
    CLEAR_PORT_BIT(gconfig.int_pin);    
    
    #ifdef ENABLE_DEBUG
        sprintf(print_buffer, "Output sample.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif 
    
}

void planalta_filter_5khz(void) {
    while(gconfig.adc_config.status == ADC_STATUS_ON){
        i2c_detect_stop();
        
        // ping-pong buffered data of in & out
        if(start_filter2){
            run_filter2_5khz();
        }
        
        i2c_detect_stop();
        
        // only input is ping-pong buffered -> filter 4 must be executed before
        // filter 3 is executed again to avoid overwriting data
        if(start_filter3){
            run_filter3_5khz();
        }
        
        i2c_detect_stop();
        
        // make sure that no data is lost of inner filtering (happens often)
        if(start_filter2){
            run_filter2_5khz();
        }
        
        i2c_detect_stop();
        
        if(start_filter4){
            run_filter4_5khz();
        }
        
        i2c_detect_stop();
        
        if(start_filter2){
            run_filter2_5khz();
        }
        
        i2c_detect_stop();
        
        // has to be executed before filter 4 to avoid overwriting data
        if(start_filter5){
            run_filter5_5khz();
        }
    }
}
