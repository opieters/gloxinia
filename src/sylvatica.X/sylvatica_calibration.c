#include "sylvatica_calibration.h"
#include <pga.h>
#include <spi.h>
#include <adc.h>
#include <fir_common.h>
#include <math.h>
#include <uart.h>
#include <utilities.h>

extern volatile uint8_t copy_buffer_selector;
extern uint16_t copy_buffers_a[SYLVATICA_N_CHANNELS][SYLVATICA_COPY_BUFFER_SIZE];
extern uint16_t copy_buffers_b[SYLVATICA_N_CHANNELS][SYLVATICA_COPY_BUFFER_SIZE];
extern sylvatica_config_t gconfig;


void sylvatica_run_calibration(sylvatica_config_t* config){
    pga_config_t pga_backup[SYLVATICA_N_CHANNELS];
    uint16_t i;
    adc_config_t adc_config = config->adc_config;
    
    // ADC in manual mode
    adc_config.trigger_select = ADC_TRIGGER_SELECT_MANUAL;
    adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_MANUAL;
    adc_config.nap_powerdown = ADC_AUTO_NAP_POWERDOWN_DISABLE;
    adc_config.deep_powerdown = ADC_DEEP_POWERDOWN_DISABLE;
    adc_config.tag_output = ADC_TAG_OUTPUT_DISABLE;
    
    // save PGA configuration and turn all PGAs off
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        pga_backup[i] = config->pga_config[i];
        
        config->pga_config[i].status = PGA_STATUS_OFF;
        
        update_pga_status(&config->pga_config[i]);
    }
    
    // configure ADC
    update_adc(&adc_config);
    
    delay_ms(100);
    
    
    // run calibration for each channel
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        // turn PGA on
        config->pga_config[i].status = PGA_STATUS_ON;
        config->pga_config[i].channel = PGA_CAL3;
        config->pga_config[i].gain = PGA_GAIN_1;
        update_pga_status(&config->pga_config[i]);
        
        // allow circuit to settle
        delay_ms(1);
        
        // select correct channel
        adc_config.channel = i;
        config->adc_config.channel_offset[i] = run_calibration(&adc_config, 0);
        
#ifdef ENABLE_DEBUG
        sprintf(print_buffer, "Calibration offset CH%d: %04x.", i, config->adc_config.channel_offset[i]);
        uart_print(print_buffer, strlen(print_buffer));
#endif
        
        // turn PGA off
        config->pga_config[i].status = PGA_STATUS_OFF;
        update_pga_status(&config->pga_config[i]);
    }
    
    
    // restore previous PGA configuration
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        config->pga_config[i] = pga_backup[i];
        
        update_pga_status(&config->pga_config[i]);
    }
    
    // restore ADC to previous configuration
    update_adc(&config->adc_config);
}

void sylvatica_gain_calibration(sylvatica_config_t* config, const uint8_t channel_n){
    bool pga_config_done;
    uint32_t scaled_value;
    const uint16_t max = 0xffff;
    adc_config_t adc_config = config->adc_config;
    uint16_t max_value, min_value, mean, gain, next_gain, n_reads;
    
    // do not perform the calibration if PGA turn off
    if(config->pga_config[channel_n].status != PGA_STATUS_ON){
        return;
    }
    
    // ADC in manual mode
    adc_config.trigger_select = ADC_TRIGGER_SELECT_MANUAL;
    adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_MANUAL;
    adc_config.nap_powerdown = ADC_AUTO_NAP_POWERDOWN_DISABLE;
    adc_config.deep_powerdown = ADC_DEEP_POWERDOWN_DISABLE;
    adc_config.tag_output = ADC_TAG_OUTPUT_DISABLE;
    
    // configure ADC
    update_adc(&adc_config);
    
    delay_ms(100);
    
    // initial setting
    config->pga_config[channel_n].gain = PGA_GAIN_1;
    
    // run calibration for each channel
    pga_config_done = false;
    n_reads = 0;
    while(!pga_config_done && (n_reads < SYLGATICA_MAX_N_CALIBRATIONS)){
        // configure PGA
        update_pga_status(&config->pga_config[channel_n]);
        
        switch(config->pga_config[channel_n].gain){
            case PGA_GAIN_1:
                gain = 1;
                next_gain = 2;
                break;
            case PGA_GAIN_2:
                gain = 2;
                next_gain = 5;
                break;
            case PGA_GAIN_5:
                gain = 5;
                next_gain = 10;
                break;
            case PGA_GAIN_10:
                gain = 10;
                next_gain = 20;
                break;
            case PGA_GAIN_20:
                gain = 20;
                next_gain = 50;
                break;
            case PGA_GAIN_50:
                gain = 50;
                next_gain = 100;
                break;
            case PGA_GAIN_100:
                gain = 100;
                next_gain = 200;
                break;
            case PGA_GAIN_200:
                gain = 200;
                next_gain = 200;
                break;
            default:
                report_error("Calibration: incorrect PGA gain setting");
                break;
        }
        
        // allow circuit to settle
        delay_ms(1);
        
        // select correct channel and get values
        adc_config.channel = channel_n;
        run_max_var(&adc_config, &max_value, &min_value, &mean);
        
        // check value
        scaled_value = (SYLVATICA_SCALE_MARGIN_PCT/100 + 1) * (uint32_t) max_value;
        
        // check if gain setting not too high
        if(scaled_value > max){
            if(config->pga_config[channel_n].gain == PGA_GAIN_1){
                pga_config_done = true;
            } else {
                config->pga_config[channel_n].gain--;
            }
            continue;
        }
   
        // check if gain can be increased
        scaled_value = (SYLVATICA_SCALE_MARGIN_PCT/100 + 1) * (next_gain / gain) * (uint32_t) max_value;
        
        if(scaled_value > max){
            // increasing gain will violate margin condition -> stop
            pga_config_done = true;
            continue;
        } else {
            
            if(config->pga_config[channel_n].gain != PGA_GAIN_200){
                // we can increase the gain
                config->pga_config[channel_n].gain++;
            } else {
                // highest gain applied -> stop
                pga_config_done = true;
            }
        }
        
        n_reads++;
    }
    
    // conversion failed due to large differences on large timescale
    if(n_reads == SYLGATICA_MAX_N_CALIBRATIONS){
        config->pga_config[channel_n].gain = PGA_GAIN_1;
        update_pga_status(&config->pga_config[channel_n]);
        
#ifdef ENABLE_DEBUG
        sprintf(print_buffer, "Conversion error for CH%d.", channel_n);
        uart_print(print_buffer, strlen(print_buffer));
#endif
    }
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Gain selected for CH%d: %01x (%d).", channel_n, config->pga_config[channel_n].gain, gain);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    // restore ADC to previous configuration
    update_adc(&config->adc_config);
}

void get_max_min_mean(uint16_t* buffer, const size_t length, uint16_t* max, 
        uint16_t* min, uint16_t* mean){
    size_t i;
    uint64_t sum = 0;
    
    for(i = 0; i < length; i++){
        sum += buffer[i];
    }
    
    *mean = (uint16_t) sum / length;
    
    *max = 0;
    for(i = 0; i < length; i++){
        *max = MAX(*max, buffer[i]);
    }
    
    *min = 0xffff;
    for(i = 0; i < length; i++){
        *min = MIN(*min, buffer[i]);
    }
}

void init_calibration_timer(void){
    uint32_t period;
    
    // configure timer to trigger perform gain calibrations
    T6CONbits.TON = 0; 
    T7CONbits.TON = 0; 
    
    T6CONbits.T32 = 1; // 32-bit timer (T6 and T7)
    
    T6CONbits.TCS = 0; // use internal instruction cycle as clock source
    T6CONbits.TGATE = 0; // disable gated timer
    T6CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR6 = 0; // clear timer register
    
    // set period: once every hour
    period = ((uint32_t) 60) * 60 * FCY / 256;
    PR6 = period & 0xffff;
    PR7 = (period >> 16) & 0xffff;
    
    // clear counters
    TMR7HLD = 0; // MSW
    TMR6 = 0;    // LSW
    
    _T6IF = 0;
    _T6IE = 0;
    
    _T7IF = 0; // clear interrupt flag
    _T7IE = 1; // enable interrupt
    
    // start timer
    T6CONbits.TON = 1; 
    
#ifdef ENABLE_DEBUG
    _T7IF = 1;
#endif
}

void __attribute__((__interrupt__, no_auto_psv)) _T6Interrupt(void) {
    _T6IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T7Interrupt(void) {
    uint16_t i;
    uint16_t max_value, min_value, mean, gain, next_gain;
    uint32_t scaled_value;
    
    if(gconfig.adc_config.status != ADC_STATUS_ON){
        _T7IF = 0;
        return;
    }
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){        
        switch(gconfig.pga_config[i].gain){
            case PGA_GAIN_1:
                gain = 1;
                next_gain = 2;
                break;
            case PGA_GAIN_2:
                gain = 2;
                next_gain = 5;
                break;
            case PGA_GAIN_5:
                gain = 5;
                next_gain = 10;
                break;
            case PGA_GAIN_10:
                gain = 10;
                next_gain = 20;
                break;
            case PGA_GAIN_20:
                gain = 20;
                next_gain = 50;
                break;
            case PGA_GAIN_50:
                gain = 50;
                next_gain = 100;
                break;
            case PGA_GAIN_100:
                gain = 100;
                next_gain = 200;
                break;
            case PGA_GAIN_200:
                gain = 200;
                next_gain = 200;
                break;
            default:
                report_error("Calibration: incorrect PGA gain setting");
                break;
        }

        if(copy_buffer_selector == 0){
            get_max_min_mean(copy_buffers_b[i], SYLVATICA_COPY_BUFFER_SIZE, 
                    &max_value, &min_value, &mean);
        } else {
            get_max_min_mean(copy_buffers_a[i], SYLVATICA_COPY_BUFFER_SIZE, 
                    &max_value, &min_value, &mean);
        }
        
        // check value
        scaled_value = (SYLVATICA_SCALE_MARGIN_PCT/100 + 1) * (uint32_t) max_value;
        
        // check if gain setting not too high
        if(scaled_value > 0xFFFF){
            if(gconfig.pga_config[i].gain != PGA_GAIN_1){
                gconfig.pga_config[i].gain--;
            }
        }
   
        // check if gain can be increased
        scaled_value = (SYLVATICA_SCALE_MARGIN_PCT/100 + 1) * (next_gain / gain) * (uint32_t) max_value;
        
        if(scaled_value < 0xffff){
            if(gconfig.pga_config[i].gain != PGA_GAIN_200){
                // we can increase the gain
                gconfig.pga_config[i].gain++;
            }
        }
        
        update_pga_status(&gconfig.pga_config[i]);
        
    }
    
    _T7IF = 0;
}
