#include "planalta_calibration.h"
#include <uart.h>
#include <dac.h>

void planalta_input_calibration(planalta_config_t* config){
    pga_config_t pga_backup[PLANALTA_N_CHANNELS];
    uint16_t i, j;
    adc_config_t adc_config = config->adc_config;
    
    // ADC in manual mode
    adc_config.trigger_select = ADC_TRIGGER_SELECT_MANUAL;
    adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_MANUAL;
    adc_config.nap_powerdown = ADC_AUTO_NAP_POWERDOWN_DISABLE;
    adc_config.deep_powerdown = ADC_DEEP_POWERDOWN_DISABLE;
    adc_config.tag_output = ADC_TAG_OUTPUT_DISABLE;
    
    // save PGA configuration and turn all PGAs off
    for(i = 0; i < PLANALTA_N_CHANNELS; i++){
        pga_backup[i] = config->pga_config[i];
        
        config->pga_config[i].status = PGA_STATUS_OFF;
        
        init_pga(&config->pga_config[i]);
    }
    
    // configure ADC
    init_adc(&adc_config);
    
    delay_ms(100);
    
    // run calibration for each channel
    for(i = 0; i < PLANALTA_N_CHANNELS; i++){
        // ADC channel index is in reversed order
        j = N_ADC_CHANNELS - 1 - 2*i;
        
        // turn PGA on
        config->pga_config[i].status = PGA_STATUS_ON;
        config->pga_config[i].channel = PGA_CAL3;
        config->pga_config[i].gain = PGA_GAIN_1;
        init_pga(&config->pga_config[i]);
        
        // allow circuit to settle
        delay_ms(1);
        
        // select correct channel
        adc_config.channel = j;
        config->adc_config.channel_offset[j] = run_calibration(&adc_config, 0);
        
#ifdef ENABLE_DEBUG
        sprintf(print_buffer, "Calibration offset CH%d: %04x.", j, config->adc_config.channel_offset[j]);
        uart_print(print_buffer, strlen(print_buffer));
#endif
        
        // turn PGA off
        config->pga_config[i].status = PGA_STATUS_OFF;
        init_pga(&config->pga_config[i]);
    }
    
    
    // restore previous PGA configuration
    for(i = 0; i < PLANALTA_N_CHANNELS; i++){
        config->pga_config[i] = pga_backup[i];
        
        init_pga(&config->pga_config[i]);
    }
    
    // restore ADC to previous configuration
    init_adc(&config->adc_config);
}
void planalta_output_calibration(planalta_config_t* config){
    uint16_t i, j;
    adc_config_t adc_config = config->adc_config;
    
    // ADC in manual mode
    adc_config.trigger_select = ADC_TRIGGER_SELECT_MANUAL;
    adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_MANUAL;
    adc_config.nap_powerdown = ADC_AUTO_NAP_POWERDOWN_DISABLE;
    adc_config.deep_powerdown = ADC_DEEP_POWERDOWN_DISABLE;
    adc_config.tag_output = ADC_TAG_OUTPUT_DISABLE;
    
    //init_dac(config, true);
    
    // configure ADC
    init_adc(&adc_config);
    
    delay_ms(100);
    
    // run calibration for each channel
    for(i = 0; i < PLANALTA_N_CHANNELS; i++){
        // ADC channel index is in reversed order
        j = N_ADC_CHANNELS - 2 - 2*i;
        
        // configure output generation
        planalta_channel_config(i, PLANALTA_CHANNEL_ENABLED, PGA_GAIN_1);
        
        // select correct channel
        adc_config.channel = j;
        config->adc_config.channel_offset[j] = run_calibration(&adc_config, 0);
        
#ifdef ENABLE_DEBUG
        sprintf(print_buffer, "Calibration offset CH%d: %04x.", j, config->adc_config.channel_offset[j]);
        uart_print(print_buffer, strlen(print_buffer));
#endif
    }
    
    init_dac(config, false);
    
    // restore ADC to previous configuration
    init_adc(&config->adc_config);
}