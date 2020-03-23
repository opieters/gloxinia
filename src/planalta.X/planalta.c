#include "planalta.h"
#include <uart.h>
#include <spi.h>
#include "filters_planalta.h"
#include "fir_coeffs.h"
#include <dsp.h>
#include <fir_common.h>
#include "planalta_filtering.h"
#include "planalta_calibration.h"

extern volatile uint8_t start_filter_block0;
extern uint8_t start_filter_block1, start_filter3;
extern uint8_t start_filter4, start_filter5, start_filter6;
extern uint16_t n_valid_output_values[PLANALTA_N_ADC_CHANNELS];
extern uint16_t filter_output_index[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_a_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_a_q[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_i[PLANALTA_N_ADC_CHANNELS];
extern fractional output_buffer_b_q[PLANALTA_N_ADC_CHANNELS];
extern bool output_buffer_full;

#define PLANALTA_ADC { \
    .channel_select = ADC_CHANNEL_SELECT_MODE_AUTO,\
    .conversion_clock_source = ADC_CONVERSION_CLOCK_SOURCE_INTERNAL,\
    .trigger_select = ADC_TRIGGER_SELECT_MANUAL,\
    .auto_trigger_rate = ADC_SAMPLE_RATE_AUTO_TRIGGER_500KSPS,\
    .pin10_polarity = ADC_PIN10_POLARITY_SELECT_ACTIVE_LOW,\
    .pin10_output = ADC_PIN10_OUTPUT_EOC,\
    .pin10_io = ADC_PIN10_IO_SELECT_EOC_INT,\
    .auto_nap = ADC_AUTO_NAP_POWERDOWN_DISABLE,\
    .nap_powerdown = ADC_NAP_POWERDOWN_DISABLE,\
    .deep_powerdown = ADC_DEEP_POWERDOWN_DISABLE,\
    .tag_output = ADC_TAG_OUTPUT_DISABLE,\
    .sw_reset = ADC_NORMAL_OPERATION,\
    .channel = ADC_CH0,\
    .sample_frequency = PLANALTA_ADC_SAMPLE_FREQUENCY,\
    .rx_callback = adc_rx_callback_5khz,\
    .spi_module = SPI_MODULE_SELECTOR_2,\
    .rst_pin = PIN_INIT(B, 3),\
    .cs_pin = PIN_INIT(E, 5),\
    .conv_pin = PIN_INIT(E, 7)} 

#define PLANALTA_PGA0 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 14), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define PLANALTA_PGA1 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 13), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define PLANALTA_PGA2 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 0), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define PLANALTA_PGA3 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 1), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define PLANALTA_I2C_CONFIG {\
    .i2c_address = PLANALTA_I2C_BASE_ADDRESS,\
    .status = I2C_STATUS_SLAVE_ON,\
    .mw_sr_cb = i2c_mw_sr_cb_planalta,\
    .mr_sw_cb = i2c_mr_sw_cb_planalta,\
    .scl_pin = PIN_INIT(G, 2),\
    .sda_pin = PIN_INIT(G, 3)}

#define PLANALTA_DAC_CONFIG {       \
    .signal_period = 64, \
    .pwm_period = 1000, \
    .status = DAC_STATUS_ON}

planalta_config_t gconfig = {
    .adc_config = PLANALTA_ADC,
    .pga_config = {
        PLANALTA_PGA0,
        PLANALTA_PGA1,
        PLANALTA_PGA2,
        PLANALTA_PGA3,
    },
    .channel_status = {
        PLANALTA_CHANNEL_ENABLED,
        PLANALTA_CHANNEL_ENABLED,
        PLANALTA_CHANNEL_ENABLED,
        PLANALTA_CHANNEL_ENABLED,
    },
    .i2c_config = PLANALTA_I2C_CONFIG,
    .address_selection = {
        PIN_INIT(B, 15),
        PIN_INIT(B, 10),
        PIN_INIT(B, 9),
        PIN_INIT(B, 12),
        PIN_INIT(F, 6),
        PIN_INIT(D, 8),
        PIN_INIT(D, 9),
        PIN_INIT(D, 10),
    },
    .blinky_pin = PIN_INIT(F, 0),
    .dac_config =  PLANALTA_DAC_CONFIG,
    .signal_frequency = PLANALTA_FILTER_SEL_5KHZ,
    .filter_selection_pins = {
        PIN_INIT(B, 6),
        PIN_INIT(B, 7),
    },
    .int_pin = PIN_INIT(D, 0),
};



i2c_message_t i2c_mr_message;
uint8_t i2c_mr_message_data[PLANALTA_I2C_READ_BUFFER_LENGTH];

volatile planalta_reg_t i2c_write_reg = PLANALTA_REG_STATUS;

uint8_t i2c_get_address_planalta(planalta_config_t* config){
    uint16_t address = 0x00, i;
    for(i = 0; i < PLANALTA_N_ADDRESS_SEL_PINS; i++){
        address |= (GET_BIT(config->address_selection[i].port_r, config->address_selection[i].n) << i);
    }
    address = ~address;
    address &= 0x7f;
    
    // if the address is not valid, use the default one
    if(!i2c_check_address(address)){
        address = PLANALTA_I2C_BASE_ADDRESS;
    }
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised I2C slave address to 0x%x.", gconfig.i2c_config.i2c_address);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    return address;
}

void init_pins_planalta(void){
    // I2C configuration
    _ODCG2 = 1; // configure I2C pins as open drain output
    _ODCG3 = 1; // configure I2C pins as open drain output
    _TRISG2 = 0;
    _TRISG3 = 0;
    _ODCD0 = 1; // nINT
    _TRISD0 = 0;
    _RD0 = 1;
    
    // UART
    _TRISD3 = 1; // U2RX
    _U2RXR = 67;            
    _TRISD2 = 0; // RTS
    _RP66R = _RPOUT_U2RTS;
    _TRISD1 = 0; // U2TX
    _RP65R = _RPOUT_U2TX;
    _TRISD4 = 1; // CTS
    _U2CTSR = 68;
    
    // SPI for ADC
    _TRISG6 = 0; // SCK2 is output
    _ANSG6 = 0;
    _TRISG7 = 1; // SDI2 is input
    _ANSG7 = 0;
    _TRISG8 = 0; // SDO2 is output
    _ANSG8 = 0;
    _TRISE5 = 0;     // CS pin
    _ANSE5 = 0;
    _RP85R = _RPOUT_OC4;
    //_RP85R = _RPOUT_SS2;
    _ANSE7 = 0;     // CONV pin
    _TRISE7 = 0;
    _RP87R = _RPOUT_OC15;
    _ANSB4 = 0;     // nEOC
    _TRISB4 = 1;
    _IC1R = 36;
    _ANSB3 = 0;     // nRESET pin
    _TRISB3 = 0;
    
    // SPI to PGA
    _ANSD6 = 0;
    _TRISD6 = 0;
    _RP69R = _RPOUT_SDO1;
    _TRISD5 = 0;
    _RP70R = _RPOUT_SCK1;
    
    // drive signals
    _TRISF2 = 0;
    _RP98R = _RPOUT_OC2;   // DRV1 signal
    _TRISF3 = 0;
    _RP99R = _RPOUT_OC2;   // DRV2 signal
    _TRISF4 = 0;
    _RP101R = _RPOUT_OC2;  // DRV3 signal
    _TRISF5 = 0;
    _RP100R = _RPOUT_OC2;  // DRV4 signal
    
    // filter selection
    _ANSB6 = 0;
    _TRISB6 = 0;
    _LATB6 = 0;
    _ANSB7 = 0;
    _TRISB7 = 0;
    _LATB7 = 0;
    
    // 4V LDO
    _ANSE1 = 0;
    _TRISE1 = 0;
    _RE1 = 1;
    
    // blinky
    _TRISF0 = 0;
    _LATF0 = 0;
    
    // 3.3V reference
    _ANSE2 = 0;
    _TRISE2 = 0;
    _RE2 = 1;
    
    // 3.3V ref to PGA
    _ANSE4 = 0;
    _TRISE4 = 0;
    _RE4 = 1;
    
    // address selection
    _ANSB15 = 0;    // A0
    _TRISB15 = 1;
    _ANSB10 = 0;    // A1
    _TRISB10 = 1;
    _ANSB9 = 0;     // A2
    _TRISB9 = 1;
    _ANSB12 = 0;    // A3sample_buffer
    _TRISB12 = 1;
    _TRISF6 = 1;    // A4
    _TRISD8 = 1;    // A5
    _TRISD9 = 1;    // A6
    _TRISD10 = 1;   // A7
}

    // check if configuration option supported
bool check_planalta_config(planalta_config_t* config){
    uint32_t n_channels = 0, i;
    
    switch(config->signal_frequency){
        case PLANALTA_FILTER_SEL_50KHZ:
            if((config->channel_status[0] == PLANALTA_CHANNEL_ENABLED)
                && ((config->channel_status[1] == PLANALTA_CHANNEL_ENABLED) 
                    || (config->channel_status[2] == PLANALTA_CHANNEL_ENABLED) 
                    || (config->channel_status[3] == PLANALTA_CHANNEL_ENABLED))){
                return false;
            }
            if((config->channel_status[1] == PLANALTA_CHANNEL_ENABLED)
                && ((config->channel_status[0] == PLANALTA_CHANNEL_ENABLED) 
                    || (config->channel_status[2] == PLANALTA_CHANNEL_ENABLED) 
                    || (config->channel_status[3] == PLANALTA_CHANNEL_ENABLED))){
                return false;
            }
            break;
        case PLANALTA_FILTER_SEL_25KHZ:
        case PLANALTA_FILTER_SEL_10KHZ:
            // check if only two active
            if((config->channel_status[2] != PLANALTA_CHANNEL_DISABLED) 
                    || (config->channel_status[3] != PLANALTA_CHANNEL_DISABLED)){
                    return false;
            }
            break;
        case PLANALTA_FILTER_SEL_5KHZ:
            // always OK
            break;
        default:
            report_error("planalta: configuration error signal frequency not supported.");
            return false;
            break;
    }
    
    for(i = 0; i < PLANALTA_N_CHANNELS; i++){
        if(config->channel_status[i] == PLANALTA_CHANNEL_ENABLED){
            n_channels++;
        }
    }
    
    if(n_channels > 0){
        uint32_t fs = 0;
        switch(config->signal_frequency){
            case PLANALTA_FILTER_SEL_50KHZ:
                fs = n_channels*4*50000;
                break;
            case PLANALTA_FILTER_SEL_25KHZ:
                fs = n_channels*2*4*25000;
                break;
            case PLANALTA_FILTER_SEL_10KHZ:
                fs = n_channels*2*4*10000;
                break;
            case PLANALTA_FILTER_SEL_5KHZ:
                fs= n_channels*2*4*5000;
                break;
            default:
                report_error("planalta: configuration error signal frequency not supported.");
                return false;
                break;
        }
        if(config->adc_config.sample_frequency != fs){
            return false;
        }
    }
    
    if(config->adc_config.sample_frequency > 250000){
        return false;
    }
    
    if(!i2c_check_address(config->i2c_config.i2c_address)){
        return false;
    }
    
    switch(config->signal_frequency){
        case PLANALTA_FILTER_SEL_50KHZ:
            // only one channel can be active
            gconfig.adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_MANUAL;
            if(config->channel_status[0] == PLANALTA_CHANNEL_ENABLED){
                gconfig.adc_config.channel = ADC_CH7;
            } else if(config->channel_status[1] == PLANALTA_CHANNEL_ENABLED) {
                gconfig.adc_config.channel = ADC_CH5;
            } else if((config->channel_status[2] == PLANALTA_CHANNEL_ENABLED) ||
                    (config->channel_status[3] == PLANALTA_CHANNEL_ENABLED)) {
                report_error("planalta: channel cannot be configured in 50kHz mode.");
            } else {
                // TODO: turn ADC off
            }
            break;
        case PLANALTA_FILTER_SEL_25KHZ:
            gconfig.adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_AUTO;
            if(config->channel_status[0] == PLANALTA_CHANNEL_ENABLED){
                gconfig.adc_config.channel = ADC_CH6;
            } else if((config->channel_status[1] == PLANALTA_CHANNEL_ENABLED) ||
                    (config->channel_status[2] == PLANALTA_CHANNEL_ENABLED) ||
                    (config->channel_status[3] == PLANALTA_CHANNEL_ENABLED)) {
                report_error("planalta: channel cannot be configured in 25kHz mode.");
            } else {
                // TODO: turn ADC off
            }
            break;
        case PLANALTA_FILTER_SEL_10KHZ:
            gconfig.adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_AUTO;
            if((config->channel_status[0] == PLANALTA_CHANNEL_ENABLED) 
                    && (config->channel_status[1] == PLANALTA_CHANNEL_ENABLED)){
                gconfig.adc_config.channel = ADC_CH4;
            } else if((config->channel_status[2] == PLANALTA_CHANNEL_ENABLED) 
                    || (config->channel_status[3] == PLANALTA_CHANNEL_ENABLED)) {
                report_error("planalta: channel cannot be configured in 50kHz mode.");
            } else {
                // TODO: turn ADC off
            }
            break;
        case PLANALTA_FILTER_SEL_5KHZ:
            gconfig.adc_config.channel_select = ADC_CHANNEL_SELECT_MODE_AUTO;
            if(config->channel_status[0] == PLANALTA_CHANNEL_ENABLED){
                gconfig.adc_config.channel = ADC_CH0;   
            } else if(config->channel_status[1] == PLANALTA_CHANNEL_ENABLED){
                gconfig.adc_config.channel = ADC_CH2;
            } else if(config->channel_status[2] == PLANALTA_CHANNEL_ENABLED){
                gconfig.adc_config.channel = ADC_CH4;
            } else if(config->channel_status[3] == PLANALTA_CHANNEL_ENABLED){
                gconfig.adc_config.channel = ADC_CH6;
            } else {
                // TODO: turn ADC off
            }
            break;
        default:
            report_error("planalta: configuration error signal frequency not supported.");
            return false;
            break;
    }
    
    return true;
}

void init_planalta(void){  
    uint16_t i;
    
    // error function
    set_error_loop_fn(i2c_detect_stop);
    set_error_pin(&gconfig.blinky_pin);
    
    init_filtering();
    planalta_clear_buffers();
    
    init_pins_planalta();
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised pins.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    blinky_init(&gconfig.blinky_pin, 1);
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised blinky.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    // read I2C slave address (sets slave address for communication with dicio)
    gconfig.i2c_config.i2c_address = i2c_get_address_planalta(&gconfig);
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised I2C slave address to 0x%x.", gconfig.i2c_config.i2c_address);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    i2c1_init(&gconfig.i2c_config);
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised I2C.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    if(check_planalta_config(&gconfig)){
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Configuration OK .");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    } else {
        report_error("planalta: configuration error.");
    }
    
    spi1_init();
    spi2_init();
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised SPI2.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    for(i = 0; i < PLANALTA_N_CHANNELS; i++){
        gconfig.pga_config[i].status = PGA_STATUS_ON;
        init_pga(&gconfig.pga_config[i]);
    }
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised PGAs.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    init_filters();
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised filters.");
    uart_print(print_buffer, strlen(print_buffer));
#endif    
    
    init_dac(&gconfig, false);
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised DAC.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
       
    
    
    init_adc(&gconfig.adc_config);  
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised ADC.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Calibrating input channels...");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    planalta_input_calibration(&gconfig);
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Calibrating output channels...");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    planalta_output_calibration(&gconfig);
    
#ifdef ENABLE_DEBUG
    adc_start(&gconfig.adc_config);
#endif
}


void loop_planalta(void){
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Entering loop...");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    while(1){
        
        while(gconfig.adc_config.status != ADC_STATUS_ON){
            i2c_detect_stop();
        }
        
        switch(gconfig.signal_frequency){
            case PLANALTA_FILTER_SEL_50KHZ:
                //planalta_filter_50khz();
                break;
            case PLANALTA_FILTER_SEL_25KHZ:
                //planalta_filter_25khz();
                break;
            case PLANALTA_FILTER_SEL_10KHZ:
                //planalta_filter_10khz();
                break;
            case PLANALTA_FILTER_SEL_5KHZ:
                planalta_filter_5khz();
                break;
            default:
                report_error("planalta: configuration error signal frequency not supported.");
                break;
        }
    }
}

void i2c_mr_sw_cb_planalta(i2c_message_t* m){
    
}

void i2c_mw_sr_cb_planalta(i2c_message_t* m){
  
    // master sends data to slave (last bit is 0)
    if(m->data_length < 1){
        return;
    }
    if(m->data[0] >= PLANALTA_N_REG){
        return;
    }
        
    i2c_write_reg = (planalta_reg_t) m->data[0];
    
    switch(i2c_write_reg){
        case PLANALTA_REG_STATUS:
            if(m->data_length >= 2){      
                if((m->data[1] & PLANALTA_STATUS_OFF) == PLANALTA_STATUS_OFF){
                    // TODO: go into low power mode
                }
                
                if((m->data[1] & PLANALTA_STATUS_RESET_BUFFER) == PLANALTA_STATUS_RESET_BUFFER){
                    planalta_clear_buffers();
                }
                
                if((m->data[1] & PLANALTA_STATUS_RESET) == PLANALTA_STATUS_RESET){
                    asm ("RESET");
                }
            }
            break;                
        case PLANALTA_REG_ADC:
            if(m->data_length >= 2){ 
                if((m->data[1] & PLANALTA_ADC_ON) == PLANALTA_ADC_ON){
                    adc_start(&gconfig.adc_config);
                } else {
                    gconfig.adc_config.status = ADC_STATUS_IDLE;
                    adc_stop(&gconfig.adc_config);
                }
            }
            break;
        case PLANALTA_REG_CONFIG_CH0:
            planalta_i2c_channel_config(0, m->data);
            break;
        case PLANALTA_REG_CONFIG_CH1:
            planalta_i2c_channel_config(1, m->data);
            break;
        case PLANALTA_REG_CONFIG_CH2:
            planalta_i2c_channel_config(2, m->data);
            break;
        case PLANALTA_REG_CONFIG_CH3:
            planalta_i2c_channel_config(3, m->data);
            break;
        case PLANALTA_REG_DATA_I0:
            planalta_i2c_read_copy_buffer_data(7);
            break;
        case PLANALTA_REG_DATA_O0:
            planalta_i2c_read_copy_buffer_data(6);
            break;
        case PLANALTA_REG_DATA_I1:
            planalta_i2c_read_copy_buffer_data(5);
            break;
        case PLANALTA_REG_DATA_O1:
            planalta_i2c_read_copy_buffer_data(4);
            break;
        case PLANALTA_REG_DATA_I2:
            planalta_i2c_read_copy_buffer_data(3);
            break;
        case PLANALTA_REG_DATA_O2:
            planalta_i2c_read_copy_buffer_data(2);
            break;
        case PLANALTA_REG_DATA_I3:
            planalta_i2c_read_copy_buffer_data(1);
            break;
        case PLANALTA_REG_DATA_O3:
            planalta_i2c_read_copy_buffer_data(0);
            break;
        case PLANALTA_REG_CONFIG_T0:
            if(m->data_length >= 3){
                //TODO
                //gconfig.dac_config.channels[0].pwm_period = (m->data[1] << 8) + m->data[2];
            } 
            break;
        case PLANALTA_REG_CONFIG_T1:
            if(m->data_length >= 3){
                //TODO
                //gconfig.dac_config.channels[1].pwm_period = (m->data[1] << 8) + m->data[2];
            }
            break;
        default:
            break;
    }
}

void planalta_i2c_read_copy_buffer_data(uint8_t channel_n){
    fractional output_i, output_q;
    
    if((!output_buffer_full) || (channel_n > 7) || (channel_n < 0)){
        i2c_init_read_message(
            &i2c_mr_message,
            i2c_mr_message_data,
            0);
    }    
    
    output_i = output_buffer_b_i[channel_n];
    output_q = output_buffer_b_q[channel_n];

    i2c_mr_message_data[0] = 4;
    
    // read I value
    i2c_mr_message_data[1] = (output_i >> 8) & 0xff;
    i2c_mr_message_data[2] = output_i & 0xff;
    // read Q value
    i2c_mr_message_data[3] = (output_q >> 8) & 0xff;
    i2c_mr_message_data[4] = output_q & 0xff;
    
    i2c_init_read_message(
        &i2c_mr_message,
        i2c_mr_message_data,
        4+1);
    
    i2c_set_read_message(&i2c_mr_message);
    
    output_buffer_full = false;
    SET_PORT_BIT(gconfig.int_pin);
}

void planalta_channel_config(uint8_t channel_n, 
        planalta_channel_status_t status, pga_gain_t gain){    
        // update configuration fields
    if(status == PLANALTA_CHANNEL_ENABLED){
        gconfig.channel_status[channel_n] = PLANALTA_CHANNEL_ENABLED;
        gconfig.pga_config[channel_n].status = PGA_STATUS_ON;
    } else {
        gconfig.channel_status[channel_n] = PLANALTA_CHANNEL_DISABLED;
        gconfig.pga_config[channel_n].status = PGA_STATUS_OFF;
    }
    
    gconfig.pga_config[channel_n].gain = gain; 
    
    // update actual hardware
    update_pga_status(&gconfig.pga_config[channel_n]);
    
    if(gconfig.channel_status[channel_n] == PLANALTA_CHANNEL_ENABLED){
        switch(channel_n){
            case 0:
                _RP98R = _RPOUT_OC2;   // DRV1 signal
            case 1:
                _RP99R = _RPOUT_OC2;   // DRV2 signal
            case 2:
                _RP101R = _RPOUT_OC2;  // DRV3 signal
            case 3:
                _RP100R = _RPOUT_OC2;  // DRV4 signal
            default:
                break;
        }
    } else {
        switch(channel_n){
            case 0:
                _RP98R = _RPOUT_PORT;   // DRV1 signal
                _RF2 = 0;
            case 1:
                _RP99R = _RPOUT_PORT;   // DRV2 signal
                _RF3 = 0;
            case 2:
                _RP101R = _RPOUT_PORT;  // DRV3 signal
                _RF4 = 0;
            case 3:
                _RP100R = _RPOUT_PORT;  // DRV4 signal
                _RF5 = 0;
            default:
                break;
        }
    }
}

void planalta_i2c_channel_config(uint8_t channel_n, uint8_t* data){
    pga_gain_t gain; 
    planalta_channel_status_t status; 

    if(gconfig.adc_config.status == ADC_STATUS_ON){
        return;
    }
    
    gain = (pga_gain_t) ((data[0] & PLANALTA_CH_CONFIG_GAIN) >> 4); 
    
        // update configuration fields
    if((data[0] & PLANALTA_CH_CONFIG_ON) == PLANALTA_CH_CONFIG_ON){
        status = PLANALTA_CHANNEL_ENABLED;
    } else {
        status = PLANALTA_CHANNEL_DISABLED;
    }
    
    planalta_channel_config(channel_n, status, gain);
    
    // prepare read message
    switch(status){
        case PLANALTA_CHANNEL_ENABLED:
            i2c_mr_message_data[0] = 1;
            break;
        case PLANALTA_CHANNEL_DISABLED:
            i2c_mr_message_data[0] = 0;
            break;
    }
    
    i2c_mr_message_data[0] = (i2c_mr_message_data[0] << 3) | PLANALTA_CH_CONFIG_SET_GAIN(gain);
    i2c_mr_message_data[0] = i2c_mr_message_data[0] << 4;
    
    i2c_init_read_message(
        &i2c_mr_message,
        i2c_mr_message_data,
        1);
    
    i2c_set_read_message(&i2c_mr_message);
}

void planalta_clear_buffers(void){
    planalta_clear_filter_buffers();
    
    i2c_init_read_message(
        &i2c_mr_message,
        i2c_mr_message_data,
        0);
    
    i2c_set_read_message(&i2c_mr_message);
}

void planalta_set_filters(planalta_signal_frequency_t config){
    switch(config){
        case PLANALTA_FILTER_SEL_50KHZ:
            CLEAR_PORT_BIT(gconfig.filter_selection_pins[0]);
            CLEAR_PORT_BIT(gconfig.filter_selection_pins[1]);
            break;
        case PLANALTA_FILTER_SEL_25KHZ:
            SET_PORT_BIT(gconfig.filter_selection_pins[0]);
            CLEAR_PORT_BIT(gconfig.filter_selection_pins[1]);
            break;
        case PLANALTA_FILTER_SEL_10KHZ:
            CLEAR_PORT_BIT(gconfig.filter_selection_pins[0]);
            SET_PORT_BIT(gconfig.filter_selection_pins[1]);
            break;
        case PLANALTA_FILTER_SEL_5KHZ:
            SET_PORT_BIT(gconfig.filter_selection_pins[0]);
            SET_PORT_BIT(gconfig.filter_selection_pins[1]);
            break;
        default:
            report_error("planalta: filter config not supported.");
            break;
    }
}
