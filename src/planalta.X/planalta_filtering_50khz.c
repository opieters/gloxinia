#include "planalta_filtering.h"
#include <fir_common.h>
#include <dsp.h>
#include "planalta.h"

extern uint8_t adc_buffer_selector;

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

extern fractional f1_to_f2_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F2_INPUT_SIZE];
extern fractional f1_to_f2_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F2_INPUT_SIZE];
extern fractional f2_to_f3_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F3_INPUT_SIZE];
extern fractional f2_to_f3_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F3_INPUT_SIZE];
extern fractional f1_to_f2_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F2_INPUT_SIZE];
extern fractional f1_to_f2_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F2_INPUT_SIZE];
extern fractional f2_to_f3_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F3_INPUT_SIZE];
extern fractional f2_to_f3_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F3_INPUT_SIZE];
extern fractional f3_to_f4_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F4_INPUT_SIZE];
extern fractional f3_to_f4_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F4_INPUT_SIZE];
extern fractional f4_to_fx_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_FX_INPUT_SIZE];
extern fractional f4_to_fx_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_FX_INPUT_SIZE];
extern fractional f3_to_f4_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F4_INPUT_SIZE];
extern fractional f3_to_f4_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_F4_INPUT_SIZE];
extern fractional f4_to_fx_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_FX_INPUT_SIZE];
extern fractional f4_to_fx_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_50KHZ_FX_INPUT_SIZE];
extern fractional output_buffer_a_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_a_q[PLANALTA_N_ADC_CHANNELS];

extern planalta_config_t gconfig;

extern i2c_message_t i2c_channel_message[PLANALTA_N_ADC_CHANNELS];
extern uint8_t i2c_channel_data[PLANALTA_N_ADC_CHANNELS][PLANALTA_I2C_READ_CH_BUFFER_LENGTH];

void adc_rx_callback_50khz(void){
    uint16_t i = 0;
    static uint16_t copy_counter = 0;
    fractional conversion_buffer[PLANALTA_50KHZ_F0_OUTPUT_SIZE];
    fractional sample_buffer[PLANALTA_50KHZ_F0_OUTPUT_SIZE];

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        if (adc_buffer_selector == 0) {
            copy_uint_to_fract(PLANALTA_50KHZ_F0_OUTPUT_SIZE,
                    (fractional*) &adc_rx_buffer_a[i],
                    conversion_buffer,
                    PLANALTA_50KHZ_N_ADC_CHANNELS);
        } else {
            copy_uint_to_fract(PLANALTA_50KHZ_F0_OUTPUT_SIZE,
                    (fractional*) &adc_rx_buffer_b[i],
                    conversion_buffer,
                    PLANALTA_50KHZ_N_ADC_CHANNELS);
        }
        fir_compressed(PLANALTA_50KHZ_F0_OUTPUT_SIZE,
            sample_buffer,
            conversion_buffer,
            &filters_0[i],
            PLANALTA_DEC_FACT_F0);

        lia_mixer_no_dc(PLANALTA_50KHZ_F0_OUTPUT_SIZE,
            sample_buffer,
            conversion_buffer,
            &conversion_buffer[PLANALTA_50KHZ_F1_INPUT_SIZE]);

        fir_compressed(PLANALTA_50KHZ_F1_OUTPUT_SIZE,
                fo1_buffer_i_write[i],
                conversion_buffer,
                &filters_1_i[i],
                PLANALTA_DEC_FACT_F1);
        fir_compressed(PLANALTA_50KHZ_F1_OUTPUT_SIZE,
                fo1_buffer_q_write[i],
                &conversion_buffer[PLANALTA_50KHZ_F1_INPUT_SIZE],
                &filters_1_q[i],
                PLANALTA_DEC_FACT_F1);
        
        fo1_buffer_i_write[i] += PLANALTA_5KHZ_F1_OUTPUT_SIZE;
        fo1_buffer_q_write[i] += PLANALTA_5KHZ_F1_OUTPUT_SIZE;
    }

    copy_counter += PLANALTA_50KHZ_F1_OUTPUT_SIZE;
    
    if(copy_counter == PLANALTA_50KHZ_F2_INPUT_SIZE){
        start_filter2 = 1;
        
        copy_counter = 0;
        
        select_f1_to_f2 ^= 1;
        
        if(select_f1_to_f2){
            for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
                fo2_buffer_i_read[i] = f1_to_f2_buffer_i_b[i];
                fo2_buffer_q_read[i] = f1_to_f2_buffer_q_b[i];
                fo1_buffer_i_write[i] = f1_to_f2_buffer_i_a[i];
                fo1_buffer_q_write[i] = f1_to_f2_buffer_q_a[i];
            }
        } else {
            for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
                fo2_buffer_i_read[i] = f1_to_f2_buffer_i_a[i];
                fo2_buffer_q_read[i] = f1_to_f2_buffer_q_a[i];
                fo1_buffer_i_write[i] = f1_to_f2_buffer_i_b[i];
                fo1_buffer_q_write[i] = f1_to_f2_buffer_q_b[i];
            }
        }
    }
    adc_buffer_selector ^= 1;

}

void run_filter2_50khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter2 = 0;

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_50KHZ_F2_OUTPUT_SIZE,
                &fo2_buffer_i_write[i][block_counter],
                fo2_buffer_i_read[i],
                &filters_2_i[i],
                PLANALTA_DEC_FACT_F2);
        fir_compressed(PLANALTA_50KHZ_F2_OUTPUT_SIZE,
                &fo2_buffer_q_write[i][block_counter],
                fo2_buffer_q_read[i],
                &filters_2_q[i],
                PLANALTA_DEC_FACT_F2);
        
        fo2_buffer_i_write[i] += PLANALTA_5KHZ_F2_OUTPUT_SIZE;
        fo2_buffer_q_write[i] += PLANALTA_5KHZ_F2_OUTPUT_SIZE;
    }
    
    block_counter += PLANALTA_50KHZ_F2_OUTPUT_SIZE;
    if(block_counter == PLANALTA_50KHZ_F3_INPUT_SIZE){
        start_filter3 = 1;
        block_counter = 0;
        
        select_f2_to_f3 ^= 1;
        
        if(select_f2_to_f3){
            for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
                fo3_buffer_i_read[i] = f2_to_f3_buffer_i_b[i];
                fo3_buffer_q_read[i] = f2_to_f3_buffer_q_b[i];
                fo2_buffer_i_write[i] = f2_to_f3_buffer_i_a[i];
                fo2_buffer_q_write[i] = f2_to_f3_buffer_q_a[i];
            }
        } else {
            for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
                fo3_buffer_i_read[i] = f2_to_f3_buffer_i_a[i];
                fo3_buffer_q_read[i] = f2_to_f3_buffer_q_a[i];
                fo2_buffer_i_write[i] = f2_to_f3_buffer_i_b[i];
                fo2_buffer_q_write[i] = f2_to_f3_buffer_q_b[i];
            }
        }
    }
}
void run_filter3_50khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter3 = 0;

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_50KHZ_F3_OUTPUT_SIZE,
                &fo3_buffer_i_write[i][block_counter],
                fo3_buffer_i_read[i],
                &filters_3_i[i],
                PLANALTA_DEC_FACT_F3);

        fir_compressed(PLANALTA_50KHZ_F3_OUTPUT_SIZE,
                &fo3_buffer_q_write[i][block_counter],
                fo3_buffer_q_read[i],
                &filters_3_q[i],
                PLANALTA_DEC_FACT_F3);
    }
    block_counter += PLANALTA_50KHZ_F3_OUTPUT_SIZE;
    if(block_counter == PLANALTA_50KHZ_F4_INPUT_SIZE){
        block_counter = 0;
        start_filter4 = 1;
        
         select_f3_to_f4 ^= 1;
        
        if(select_f3_to_f4){
            for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
                fo4_buffer_i_read[i] = f3_to_f4_buffer_i_b[i];
                fo4_buffer_q_read[i] = f3_to_f4_buffer_q_b[i];
                fo3_buffer_i_write[i] = f3_to_f4_buffer_i_a[i];
                fo3_buffer_q_write[i] = f3_to_f4_buffer_q_a[i];
            }
        } else {
            for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
                fo4_buffer_i_read[i] = f3_to_f4_buffer_i_a[i];
                fo4_buffer_q_read[i] = f3_to_f4_buffer_q_a[i];
                fo3_buffer_i_write[i] = f3_to_f4_buffer_i_b[i];
                fo3_buffer_q_write[i] = f3_to_f4_buffer_q_b[i];
            }
        }
    }
}

void run_filter4_50khz(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    start_filter4 = 0;

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_50KHZ_F4_OUTPUT_SIZE,
                &fo4_buffer_i_write[i][block_counter],
                fo4_buffer_i_read[i],
                &filters_4_i[i],
                PLANALTA_DEC_FACT_F4);

        fir_compressed(PLANALTA_50KHZ_F4_OUTPUT_SIZE,
                &fo4_buffer_q_write[i][block_counter],
                fo4_buffer_q_read[i],
                &filters_4_q[i],
                PLANALTA_DEC_FACT_F4);
    }
    block_counter += PLANALTA_50KHZ_F4_OUTPUT_SIZE;
    if(block_counter == PLANALTA_50KHZ_FX_INPUT_SIZE){
        block_counter = 0;
        start_filter5 = 1;
    }
}


void run_filter5_50khz(void){
    uint16_t i;
    start_filter5 = 0;
    
    _SI2C1IE = 0;
    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        i2c_channel_data[i][1] = (uint8_t) ((output_buffer_a_i[i] >> 8) & 0xff);
        i2c_channel_data[i][2] = (uint8_t) (output_buffer_a_i[i] & 0xff);
        i2c_channel_data[i][3] = (uint8_t) ((output_buffer_a_q[i] >> 8) & 0xff);
        i2c_channel_data[i][4] = (uint8_t) (output_buffer_a_q[i] & 0xff);

        i2c_init_read_message(
            &i2c_channel_message[i],
            i2c_channel_data[i],
            PLANALTA_I2C_READ_CH_BUFFER_LENGTH);
    }
    _SI2C1IE = 1; 

    for(i = 0; i < PLANALTA_50KHZ_N_ADC_CHANNELS; i++){
        fir_compressed(PLANALTA_50KHZ_FX_OUTPUT_SIZE,
                &output_buffer_a_i[i],
                fo4_buffer_i_write[i],
                &filters_5_i[i],
                PLANALTA_DEC_FACT_F5);

        fir_compressed(PLANALTA_50KHZ_FX_OUTPUT_SIZE,
                &output_buffer_a_q[i],
                fo4_buffer_q_write[i],
                &filters_5_q[i],
                PLANALTA_DEC_FACT_F5);
    }
    
    CLEAR_PORT_BIT(gconfig.int_pin);
}

void planalta_filter_50khz(void) {
    while(gconfig.adc_config.status == ADC_STATUS_ON){
        i2c1_detect_stop();
        
        if(start_filter2){
            run_filter2_50khz();
        }
        
        i2c1_detect_stop();
        
        if(start_filter5){
            run_filter5_50khz();
            continue;
        }
        
        i2c1_detect_stop();
        
        if(start_filter4){
            run_filter4_50khz();
            continue;
        }
        
        i2c1_detect_stop();
        
        if(start_filter3){
            run_filter3_50khz();
            continue;
        }
        
        
    }
}
