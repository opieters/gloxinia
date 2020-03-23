#include "planalta_filtering.h"
#include <fir_common.h>
#include <dsp.h>
#include "planalta.h"
#include "planalta_freq_sweep.h"

extern volatile uint8_t copy_buffer_selector;
extern uint8_t adc_buffer_selector;

extern volatile uint8_t start_filter_block0;
extern uint8_t start_filter2, start_filter3, start_filter4, start_filter5;

extern fractional* fo2_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo3_buffer_i_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo3_buffer_q_read[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo1_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo1_buffer_q_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_i_write[PLANALTA_N_ADC_CHANNELS];
extern fractional* fo2_buffer_q_write[PLANALTA_N_ADC_CHANNELS];

extern uint8_t select_f1_to_f2, select_f2_to_f3;

extern fractional f1_to_f2_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f1_to_f2_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f2_to_f3_buffer_i_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f2_to_f3_buffer_q_a[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f1_to_f2_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f1_to_f2_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F2_INPUT_SIZE];
extern fractional f2_to_f3_buffer_i_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f2_to_f3_buffer_q_b[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F3_INPUT_SIZE];
extern fractional f3_to_f4_buffer_i[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F4_INPUT_SIZE];
extern fractional f3_to_f4_buffer_q[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_F4_INPUT_SIZE];
extern fractional f4_to_fx_buffer_i[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_FX_INPUT_SIZE];
extern fractional f4_to_fx_buffer_q[PLANALTA_N_ADC_CHANNELS][PLANALTA_5KHZ_FX_INPUT_SIZE];
extern fractional output_buffer_a_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_a_q[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_q[PLANALTA_N_ADC_CHANNELS];

extern planalta_config_t gconfig;

extern uint8_t output_buffer_full;

planalta_fs_freq_t freq;
volatile uint8_t start_filter_fs1 = 0;
volatile uint8_t start_filter_fs2 = 0;
volatile uint8_t start_filter_fs3 = 0;
volatile uint8_t start_filter_fs4 = 0;

#define BUFFER_SIZE_FS 20

#define PLANALTA_FS_50KHZ_FO1_OUTPUT_SIZE   (ADC_BUFFER_LENGTH / (2 * PLANALTA_DEC_FACT_F1))
#define PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE   (BUFFER_SIZE_FS / PLANALTA_DEC_FACT_F2)

#define PLANALTA_FS_25KHZ_FO1_OUTPUT_SIZE   (ADC_BUFFER_LENGTH / (2 * PLANALTA_DEC_FACT_F1 * PLANALTA_DEC_FACT_F8))
#define PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE   (BUFFER_SIZE_FS / PLANALTA_DEC_FACT_F2)

#define PLANALTA_FS_10KHZ_FO1_OUTPUT_SIZE   (ADC_BUFFER_LENGTH / (2 * PLANALTA_DEC_FACT_F1 * PLANALTA_DEC_FACT_F9))
#define PLANALTA_FS_10KHZ_FO2_OUTPUT_SIZE   (BUFFER_SIZE_FS / PLANALTA_DEC_FACT_F2)

#define PLANALTA_FS_5KHZ_FO1_OUTPUT_SIZE    (ADC_BUFFER_LENGTH / (2 * PLANALTA_DEC_FACT_F1 * PLANALTA_DEC_FACT_F2))
#define PLANALTA_FS_5KHZ_FO2_OUTPUT_SIZE    (BUFFER_SIZE_FS / PLANALTA_DEC_FACT_F3)

#define PLANALTA_FS_2_5KHZ_FO0_OUTPUT_SIZE  (ADC_BUFFER_LENGTH /  PLANALTA_DEC_FACT_F8)
#define PLANALTA_FS_2_5KHZ_FO1_OUTPUT_SIZE  (PLANALTA_FS_2_5KHZ_FO0_OUTPUT_SIZE / PLANALTA_DEC_FACT_F2)
#define PLANALTA_FS_2_5KHZ_FO2_OUTPUT_SIZE  (BUFFER_SIZE_FS / (PLANALTA_DEC_FACT_F1 / 2))

#define PLANALTA_FS_1KHZ_FO0_OUTPUT_SIZE    (ADC_BUFFER_LENGTH /  PLANALTA_DEC_FACT_F9)
#define PLANALTA_FS_1KHZ_FO1_OUTPUT_SIZE    (PLANALTA_FS_1KHZ_FO0_OUTPUT_SIZE / PLANALTA_DEC_FACT_F2)

#define PLANALTA_FS_500HZ_FO0_OUTPUT_SIZE   (ADC_BUFFER_LENGTH /  PLANALTA_DEC_FACT_F2)
#define PLANALTA_FS_500HZ_FO1_OUTPUT_SIZE   (PLANALTA_FS_500HZ_FO0_OUTPUT_SIZE / PLANALTA_DEC_FACT_F3)

#define PLANALTA_FS_250HZ_FO0_OUTPUT_SIZE   (ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F8)
#define PLANALTA_FS_250HZ_FO1_OUTPUT_SIZE   (PLANALTA_FS_250HZ_FO0_OUTPUT_SIZE / PLANALTA_DEC_FACT_F2)
#define PLANALTA_FS_250HZ_FO2_OUTPUT_SIZE   (PLANALTA_FS_250HZ_FO1_OUTPUT_SIZE / PLANALTA_DEC_FACT_F3)

#define PLANALTA_FS_100HZ_FO0_OUTPUT_SIZE   (ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F9)
#define PLANALTA_FS_100HZ_FO1_OUTPUT_SIZE   (PLANALTA_FS_100HZ_FO0_OUTPUT_SIZE / PLANALTA_DEC_FACT_F2)
#define PLANALTA_FS_100HZ_FO2_OUTPUT_SIZE   (PLANALTA_FS_100HZ_FO1_OUTPUT_SIZE / PLANALTA_DEC_FACT_F3)

#define PLANALTA_FS_STD_OUTPUT_SIZE 2


uint8_t buffer_selector_0 = 0;
uint8_t buffer_selector_1 = 0;
uint8_t buffer_selector_2 = 0;
uint8_t buffer_selector_3 = 0;

fractional buffer_0_a_i[BUFFER_SIZE_FS];
fractional buffer_0_a_q[BUFFER_SIZE_FS];
fractional buffer_0_b_i[BUFFER_SIZE_FS];
fractional buffer_0_b_q[BUFFER_SIZE_FS];
fractional* buffer_0_a_i_ptr = buffer_0_a_i;
fractional* buffer_0_a_q_ptr = buffer_0_a_q;
fractional* buffer_0_b_i_ptr = buffer_0_b_i;
fractional* buffer_0_b_q_ptr = buffer_0_b_q;

fractional buffer_1_a_i[BUFFER_SIZE_FS];
fractional buffer_1_a_q[BUFFER_SIZE_FS];
fractional buffer_1_b_i[BUFFER_SIZE_FS];
fractional buffer_1_b_q[BUFFER_SIZE_FS];
fractional* buffer_1_a_i_ptr = buffer_1_a_i;
fractional* buffer_1_a_q_ptr = buffer_1_a_q;
fractional* buffer_1_b_i_ptr = buffer_1_b_i;
fractional* buffer_1_b_q_ptr = buffer_1_b_q;


void adc_rx_callback_fs(void){
    uint16_t i = 0;
    static uint16_t copy_counter = 0;
    fractional conversion_buffer[ADC_BUFFER_LENGTH];
    fractional sample_buffer[ADC_BUFFER_LENGTH];
    
    if (adc_buffer_selector == 0) {
        copy_uint_to_fract(ADC_BUFFER_LENGTH,
                (fractional*) &adc_rx_buffer_a[i],
                conversion_buffer,
                1);
    } else {
        copy_uint_to_fract(ADC_BUFFER_LENGTH,
                (fractional*) &adc_rx_buffer_b[i],
                conversion_buffer,
                1);
    }

    switch(freq){
        case PLANALTA_FS_FREQ_50KHZ:
            // FO0
            fir_compressed(ADC_BUFFER_LENGTH,
                sample_buffer,
                conversion_buffer,
                &filters_0[0],
                PLANALTA_DEC_FACT_F0);

            // MIX
            lia_mixer_no_dc(ADC_BUFFER_LENGTH,
                conversion_buffer,
                sample_buffer,
                &sample_buffer[ADC_BUFFER_LENGTH/2]);

            // FO1
            if(buffer_selector_0 == 0){
                fir_compressed(PLANALTA_FS_50KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_50KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            } else {
                fir_compressed(PLANALTA_FS_50KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_50KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            }
            
            copy_counter += PLANALTA_FS_50KHZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_25KHZ:
            // FO0
            fir_compressed(ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F8,
                sample_buffer,
                conversion_buffer,
                &filters_8_i[0],
                PLANALTA_DEC_FACT_F8);

            // MIX
            lia_mixer_no_dc(ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F8,
                conversion_buffer,
                sample_buffer,
                &sample_buffer[ADC_BUFFER_LENGTH/2]);
            
            // FO1
            if(buffer_selector_0 == 0){
                fir_compressed(PLANALTA_FS_25KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_25KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            } else {
                fir_compressed(PLANALTA_FS_25KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_25KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            }
            
            copy_counter += PLANALTA_FS_25KHZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_10KHZ:
            // FO0
            fir_compressed(ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F9,
                sample_buffer,
                conversion_buffer,
                &filter_9,
                PLANALTA_DEC_FACT_F9);

            // MIX
            lia_mixer_no_dc(ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F9,
                conversion_buffer,
                sample_buffer,
                &sample_buffer[ADC_BUFFER_LENGTH/2]);
            
            // FO1
            if(buffer_selector_0 == 0){
                fir_compressed(PLANALTA_FS_10KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_10KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            } else {
                fir_compressed(PLANALTA_FS_10KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_10KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            }
            
            copy_counter += PLANALTA_FS_10KHZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_5KHZ:
            // FO0
            fir_compressed(ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F2,
                sample_buffer,
                conversion_buffer,
                &filters_2_i[0],
                PLANALTA_DEC_FACT_F2);

            // MIX
            lia_mixer_no_dc(ADC_BUFFER_LENGTH / PLANALTA_DEC_FACT_F2,
                conversion_buffer,
                sample_buffer,
                &sample_buffer[ADC_BUFFER_LENGTH/2]);
            
            // FO1
            if(buffer_selector_0 == 0){
                fir_compressed(PLANALTA_FS_5KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_5KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            } else {
                fir_compressed(PLANALTA_FS_5KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_i_ptr,
                    sample_buffer,
                    &filters_1_i[0],
                    PLANALTA_DEC_FACT_F1);
                fir_compressed(PLANALTA_FS_5KHZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_q_ptr,
                    &sample_buffer[ADC_BUFFER_LENGTH/2],
                    &filters_1_q[0],
                    PLANALTA_DEC_FACT_F1);
            }
            
            copy_counter += PLANALTA_FS_5KHZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_2_5KHZ:
            // FO0
            fir_compressed(PLANALTA_FS_2_5KHZ_FO0_OUTPUT_SIZE,
                sample_buffer,
                conversion_buffer,
                &filters_8_i[0],
                PLANALTA_DEC_FACT_F8);
            
            // FO1
            fir_compressed(PLANALTA_FS_2_5KHZ_FO1_OUTPUT_SIZE,
                conversion_buffer,
                sample_buffer,
                &filters_2_i[0],
                PLANALTA_DEC_FACT_F2);

            // MIX
            if(buffer_selector_0 == 0){
                lia_mixer_no_dc(PLANALTA_FS_2_5KHZ_FO1_OUTPUT_SIZE,
                sample_buffer,
                buffer_0_a_i_ptr,
                buffer_0_a_q_ptr);
            } else {
                lia_mixer_no_dc(PLANALTA_FS_2_5KHZ_FO1_OUTPUT_SIZE,
                sample_buffer,
                buffer_0_b_i_ptr,
                buffer_0_b_q_ptr);
            }
            
            copy_counter += PLANALTA_FS_2_5KHZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_1KHZ:
            // FO0
            fir_compressed(PLANALTA_FS_1KHZ_FO0_OUTPUT_SIZE,
                sample_buffer,
                conversion_buffer,
                &filter_9,
                PLANALTA_DEC_FACT_F9);
            
            // FO1
            fir_compressed(PLANALTA_FS_1KHZ_FO1_OUTPUT_SIZE,
                conversion_buffer,
                sample_buffer,
                &filters_2_i[0],
                PLANALTA_DEC_FACT_F2);

            // MIX
            if(buffer_selector_0 == 0){
                lia_mixer_no_dc(PLANALTA_FS_1KHZ_FO1_OUTPUT_SIZE,
                sample_buffer,
                buffer_0_a_i_ptr,
                buffer_0_a_q_ptr);
            } else {
                lia_mixer_no_dc(PLANALTA_FS_1KHZ_FO1_OUTPUT_SIZE,
                sample_buffer,
                buffer_0_b_i_ptr,
                buffer_0_b_q_ptr);
            }
            
            copy_counter += PLANALTA_FS_1KHZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_500HZ:
            // FO0
            fir_compressed(PLANALTA_FS_500HZ_FO0_OUTPUT_SIZE,
                sample_buffer,
                conversion_buffer,
                &filters_2_i[0],
                PLANALTA_DEC_FACT_F2);
            
            // FO1
            fir_compressed(PLANALTA_FS_500HZ_FO1_OUTPUT_SIZE,
                conversion_buffer,
                sample_buffer,
                &filters_3_i[0],
                PLANALTA_DEC_FACT_F3);

            // MIX
            if(buffer_selector_0 == 0){
                lia_mixer_no_dc(PLANALTA_FS_500HZ_FO1_OUTPUT_SIZE,
                sample_buffer,
                buffer_0_a_i_ptr,
                buffer_0_a_q_ptr);
            } else {
                lia_mixer_no_dc(PLANALTA_FS_500HZ_FO1_OUTPUT_SIZE,
                sample_buffer,
                buffer_0_b_i_ptr,
                buffer_0_b_q_ptr);
            }
            
            copy_counter += PLANALTA_FS_500HZ_FO1_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_250HZ:
            // FO0
            fir_compressed(PLANALTA_FS_250HZ_FO0_OUTPUT_SIZE,
                sample_buffer,
                conversion_buffer,
                &filters_8_i[0],
                PLANALTA_DEC_FACT_F8);
            
            // FO1
            fir_compressed(PLANALTA_FS_250HZ_FO1_OUTPUT_SIZE,
                conversion_buffer,
                sample_buffer,
                &filters_2_i[0],
                PLANALTA_DEC_FACT_F2);

            // FO2
            if(buffer_selector_0 == 0){
                fir_compressed(PLANALTA_FS_250HZ_FO2_OUTPUT_SIZE,
                    buffer_0_a_i_ptr,
                    sample_buffer,
                    &filters_3_i[0],
                    PLANALTA_DEC_FACT_F3);
            } else {
                fir_compressed(PLANALTA_FS_250HZ_FO2_OUTPUT_SIZE,
                    buffer_0_b_i_ptr,
                    sample_buffer,
                    &filters_3_i[0],
                    PLANALTA_DEC_FACT_F3);
            }
            
            copy_counter += PLANALTA_FS_250HZ_FO2_OUTPUT_SIZE;
            break;
            
        case PLANALTA_FS_FREQ_100HZ:
            // FO0
            fir_compressed(PLANALTA_FS_100HZ_FO0_OUTPUT_SIZE,
                sample_buffer,
                conversion_buffer,
                &filter_9,
                PLANALTA_DEC_FACT_F9);
            
            // FO1
            if(buffer_selector_0 == 0){
                fir_compressed(PLANALTA_FS_100HZ_FO1_OUTPUT_SIZE,
                    buffer_0_a_i_ptr,
                    sample_buffer,
                    &filters_2_i[0],
                    PLANALTA_DEC_FACT_F2);
            } else {
                fir_compressed(PLANALTA_FS_100HZ_FO1_OUTPUT_SIZE,
                    buffer_0_b_i_ptr,
                    sample_buffer,
                    &filters_2_i[0],
                    PLANALTA_DEC_FACT_F2);
            }
            
            copy_counter += PLANALTA_FS_100HZ_FO1_OUTPUT_SIZE;
            break;
            
        default:
            report_error("planalta: unknown freq config");
            break;
    }
    
    if(copy_counter == BUFFER_SIZE_FS){
        copy_counter = 0;
        buffer_selector_0 ^= 1;
        start_filter_fs1 = 1;
    }
}



void planalta_filter_fs1(void){
    //uint16_t i;
    static uint16_t block_counter1 = 0;
    start_filter2 = 0;

    switch(freq){
        case PLANALTA_FS_FREQ_50KHZ:
            // FO2
            if(buffer_selector_0 == 0){
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            } else {
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            }
            block_counter1 += PLANALTA_FS_50KHZ_FO2_OUTPUT_SIZE;

            if(block_counter1 == PLANALTA_DEC_FACT_F3){
                buffer_selector_1 ^= 1;
                start_filter_fs2 = 1;
                block_counter1 = 0;
            }

            break;
        case PLANALTA_FS_FREQ_25KHZ:
            // FO2
            if(buffer_selector_0 == 0){
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            } else {
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            }
            block_counter1 += PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE;

            if(block_counter1 == PLANALTA_DEC_FACT_F3){
                buffer_selector_1 ^= 1;
                start_filter_fs2 = 1;
                block_counter1 = 0;
            }
            break;
        case PLANALTA_FS_FREQ_10KHZ:
            // FO2
            if(buffer_selector_0 == 0){
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_STD_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_STD_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_STD_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_STD_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            } else {
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_STD_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            }
            block_counter1 += PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE;

            if(block_counter1 == PLANALTA_DEC_FACT_F3){
                buffer_selector_1 ^= 1;
                start_filter_fs2 = 1;
                block_counter1 = 0;
            }
            break;
        case PLANALTA_FS_FREQ_5KHZ:
            // FO2
            if(buffer_selector_0 == 0){
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_b_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            } else {
                if(buffer_selector_1 == 0) {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_a_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_a_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                } else {
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_i_ptr,
                        buffer_1_b_i_ptr,
                        &filters_2_i[0],
                        PLANALTA_DEC_FACT_F2);
                    fir_compressed(PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE,
                        buffer_0_a_q_ptr,
                        buffer_1_b_q_ptr,
                        &filters_2_q[0],
                        PLANALTA_DEC_FACT_F2);
                }
            }
            block_counter1 += PLANALTA_FS_25KHZ_FO2_OUTPUT_SIZE;

            if(block_counter1 == PLANALTA_DEC_FACT_F3){
                buffer_selector_1 ^= 1;
                start_filter_fs2 = 1;
                block_counter1 = 0;
            }
            break;
        case PLANALTA_FS_FREQ_2_5KHZ:
            break;
        case PLANALTA_FS_FREQ_1KHZ:
            break;
        case PLANALTA_FS_FREQ_500HZ:
            break;
        case PLANALTA_FS_FREQ_250HZ:
            break;
        case PLANALTA_FS_FREQ_100HZ:
            break;
        default:
            report_error("planalta: unknown freq config");
            break;
    }
}
void planalta_filter_fs2(void){

}

void planalta_filter_fs3(void){

}


void planalta_filter_fs4(void){

  
    
}

void planalta_filter_fs(void) {
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
