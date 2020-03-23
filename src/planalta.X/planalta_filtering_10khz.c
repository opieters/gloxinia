#include "planalta_filtering.h"
#include <fir_common.h>
#include <dsp.h>
#include "planalta.h"

extern volatile uint8_t copy_buffer_selector;
extern uint8_t adc_buffer_selector;

extern volatile uint8_t start_filter_block0;
extern uint8_t start_filter2, start_filter3, start_filter4, start_filter5;

extern fractional fo1_buffer_i_write[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_F2_INPUT_SIZE];
extern fractional fo1_buffer_q_write[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_F2_INPUT_SIZE];
extern fractional fo2_buffer_i_write[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_F3_INPUT_SIZE];
extern fractional fo2_buffer_q_write[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_F3_INPUT_SIZE];
extern fractional f3_to_f4_buffer_i[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_F4_INPUT_SIZE];
extern fractional f3_to_f4_buffer_q[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_F4_INPUT_SIZE];
extern fractional f4_to_fx_buffer_i[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_FX_INPUT_SIZE];
extern fractional f4_to_fx_buffer_q[PLANALTA_N_ADC_CHANNELS][PLANALTA_10KHZ_FX_INPUT_SIZE];
extern fractional output_buffer_a_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_a_q[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_q[PLANALTA_N_ADC_CHANNELS];

extern planalta_config_t gconfig;

extern uint8_t output_buffer_full;

void adc_rx_callback_10khz(void){
    uint16_t i = 0;
    static uint16_t copy_counter = 0;
    fractional conversion_buffer[PLANALTA_10KHZ_F0_OUTPUT_SIZE];
    fractional sample_buffer[PLANALTA_10KHZ_F0_OUTPUT_SIZE];

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        if (adc_buffer_selector == 0) {
            copy_uint_to_fract(PLANALTA_10KHZ_F0_OUTPUT_SIZE,
                    (fractional*) &adc_rx_buffer_a[i],
                    conversion_buffer,
                    PLANALTA_10KHZ_N_ADC_CHANNELS);
        } else {
            copy_uint_to_fract(PLANALTA_10KHZ_F0_OUTPUT_SIZE,
                    (fractional*) &adc_rx_buffer_b[i],
                    conversion_buffer,
                    PLANALTA_10KHZ_N_ADC_CHANNELS);
        }
        void process_filter_block0_10khz(void){
}
        /*fir_compressed(PLANALTA_10KHZ_F0_OUTPUT_SIZE,
            sample_buffer,
            conversion_buffer,
            &filters_0[i],
            PLANALTA_DEC_FACT_F0);*/

        lia_mixer_no_dc(PLANALTA_10KHZ_F0_OUTPUT_SIZE,
            conversion_buffer,
            sample_buffer,
            &sample_buffer[PLANALTA_10KHZ_F1_INPUT_SIZE]);

        fir_compressed(PLANALTA_10KHZ_F1_OUTPUT_SIZE,
                &fo1_buffer_i_write[i][copy_counter],
                sample_buffer,
                &filters_1_i[i],
                PLANALTA_DEC_FACT_F1);
        fir_compressed(PLANALTA_10KHZ_F1_OUTPUT_SIZE,
                &fo1_buffer_q_write[i][copy_counter],
                &sample_buffer[PLANALTA_10KHZ_F1_INPUT_SIZE],
                &filters_1_q[i],
                PLANALTA_DEC_FACT_F1);
    }


    copy_counter += PLANALTA_10KHZ_F1_OUTPUT_SIZE;
    if(copy_counter == PLANALTA_10KHZ_F2_INPUT_SIZE){
        start_filter2 = 1;
        copy_counter = 0;
    }
    adc_buffer_selector ^= 1;

}

void run_filter2_10khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter2 = 0;

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_10KHZ_F2_OUTPUT_SIZE,
                &fo2_buffer_i_write[i][block_counter],
                fo1_buffer_i_write[i],
                &filters_2_i[i],
                PLANALTA_DEC_FACT_F2);
        fir_compressed(PLANALTA_10KHZ_F2_OUTPUT_SIZE,
                &fo2_buffer_q_write[i][block_counter],
                fo1_buffer_q_write[i],
                &filters_2_q[i],
                PLANALTA_DEC_FACT_F2);
    }
    block_counter += PLANALTA_10KHZ_F2_OUTPUT_SIZE;
    if(block_counter == PLANALTA_10KHZ_F3_INPUT_SIZE){
        start_filter3 = 1;
        block_counter = 0;
    }
}
void run_filter3_10khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter3 = 0;

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_10KHZ_F3_OUTPUT_SIZE,
                &f3_to_f4_buffer_i[i][block_counter],
                fo2_buffer_i_write[i],
                &filters_3_i[i],
                PLANALTA_DEC_FACT_F3);

        fir_compressed(PLANALTA_10KHZ_F3_OUTPUT_SIZE,
                &f3_to_f4_buffer_q[i][block_counter],
                fo2_buffer_q_write[i],
                &filters_3_q[i],
                PLANALTA_DEC_FACT_F3);
    }
    block_counter += PLANALTA_10KHZ_F3_OUTPUT_SIZE;
    if(block_counter == PLANALTA_10KHZ_F4_INPUT_SIZE){
        block_counter = 0;
        start_filter4 = 1;
    }
}

void run_filter4_10khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter4 = 0;

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_10KHZ_F4_OUTPUT_SIZE,
                &f4_to_fx_buffer_i[i][block_counter],
                f3_to_f4_buffer_i[i],
                &filters_4_i[i],
                PLANALTA_DEC_FACT_F4);

        fir_compressed(PLANALTA_10KHZ_F4_OUTPUT_SIZE,
                &f4_to_fx_buffer_q[i][block_counter],
                f3_to_f4_buffer_q[i],
                &filters_4_q[i],
                PLANALTA_DEC_FACT_F4);
    }
    block_counter += PLANALTA_10KHZ_F4_OUTPUT_SIZE;
    if(block_counter == PLANALTA_10KHZ_FX_INPUT_SIZE){
        block_counter = 0;
        start_filter5 = 1;
    }
}


void run_filter5_10khz(void){
    uint16_t i;
    start_filter5 = 0;

    for(i = 0; i < PLANALTA_10KHZ_N_ADC_CHANNELS; i++){
        if(output_buffer_full == 0){

            fir_compressed(PLANALTA_10KHZ_FX_OUTPUT_SIZE,
                    &output_buffer_a_i[i],
                    f4_to_fx_buffer_i[i],
                    &filters_8_i[i],
                    PLANALTA_DEC_FACT_F8);

            fir_compressed(PLANALTA_10KHZ_FX_OUTPUT_SIZE,
                    &output_buffer_a_q[i],
                    f4_to_fx_buffer_q[i],
                    &filters_8_q[i],
                    PLANALTA_DEC_FACT_F8);
        } else {

            fir_compressed(PLANALTA_10KHZ_FX_OUTPUT_SIZE,
                    &output_buffer_b_i[i],
                    f4_to_fx_buffer_i[i],
                    &filters_8_i[i],
                    PLANALTA_DEC_FACT_F8);

            fir_compressed(PLANALTA_10KHZ_FX_OUTPUT_SIZE,
                    &output_buffer_b_q[i],
                    f4_to_fx_buffer_q[i],
                    &filters_8_q[i],
                    PLANALTA_DEC_FACT_F8);

        }
    }
    
    output_buffer_full ^= 1;
        
    SET_PORT_BIT(gconfig.int_pin);
}

void planalta_filter_10khz(void) {
    while(gconfig.adc_config.status == ADC_STATUS_ON){
        detect_stop();
        if(start_filter2){
            run_filter2_10khz();
        }
        if(start_filter3){
            run_filter3_10khz();
            continue;
        }
        if(start_filter4){
            run_filter4_10khz();
            continue;
        }
        if(start_filter5){
            run_filter5_10khz();
            continue;
        }
    }
}
