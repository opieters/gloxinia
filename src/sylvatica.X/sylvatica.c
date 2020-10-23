#include <sylvatica.h>
#include <uart.h>
#include <spi.h>
#include <filters_sylvatica.h>
#include <fir_coeffs.h>
#include <dsp.h>
#include <fir_common.h>
#include "sylvatica_calibration.h"
#include <stdbool.h>

#define SYLVATICA_ADC { \
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
    .sample_frequency = SYLVATICA_ADC_SAMPLE_FREQUENCY,\
    .rx_callback = adc_rx_callback,\
    .spi_module = SPI_MODULE_SELECTOR_2,\
    .rst_pin = PIN_INIT(B, 15),\
    .cs_pin = PIN_INIT(E, 5),\
    .conv_pin = PIN_INIT(F, 3)} 

#define SYLVATICA_PGA0 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(E, 3), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA1 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(D, 11), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA2 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(E, 6), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA3 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(E, 7), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA4 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 11), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA5 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 10), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA6 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 9), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_PGA7 {      \
    .channel = PGA_MUX_CH1,   \
    .gain = PGA_GAIN_1,       \
    .cs_pin = PIN_INIT(B, 8), \
    .spi_message_handler = spi1_send_message,\
    .status = PGA_STATUS_OFF}

#define SYLVATICA_I2C_CONFIG {\
    .i2c_address = SYLVATICA_I2C_BASE_ADDRESS,\
    .status = I2C_STATUS_SLAVE_ON,\
    .mw_sr_cb = i2c_mw_sr_cb_sylvatica,\
    .mr_sw_cb = i2c_mr_sw_cb_sylvatica,\
    .scl_pin = PIN_INIT(G, 2),\
    .sda_pin = PIN_INIT(G, 3)}

sylvatica_config_t gconfig = {
    .adc_config = SYLVATICA_ADC,
    .pga_config = {
        SYLVATICA_PGA0,
        SYLVATICA_PGA1,
        SYLVATICA_PGA2,
        SYLVATICA_PGA3,
        SYLVATICA_PGA4,
        SYLVATICA_PGA5,
        SYLVATICA_PGA6,
        SYLVATICA_PGA7,
    },
    .channel_status = {
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
        SYLVATICA_CHANNEL_DISABLED,
    },
    .i2c_config = SYLVATICA_I2C_CONFIG,
    .address_selection = {
        PIN_INIT(E, 0),
        PIN_INIT(E, 1),
        PIN_INIT(F, 1),
        PIN_INIT(B, 0),
        PIN_INIT(B, 1),
        PIN_INIT(B, 2),
        PIN_INIT(B, 3),
        PIN_INIT(B, 4),
    },
    .blinky_pin = PIN_INIT(C, 13),
    .int_pin = PIN_INIT(D, 0),
};

volatile uint8_t copy_buffer_selector = 0;
uint8_t adc_buffer_selector = 0;
volatile uint8_t start_filter_block0 = 0;
uint8_t start_filter_block1 = 0, start_filter_block2 = 0, start_print_values = 0;
uint8_t start_filter_block3 = 0;

uint16_t copy_buffers_a[SYLVATICA_N_CHANNELS][SYLVATICA_COPY_BUFFER_SIZE];
uint16_t copy_buffers_b[SYLVATICA_N_CHANNELS][SYLVATICA_COPY_BUFFER_SIZE];
fractional* output_block0_buffer[SYLVATICA_N_CHANNELS];
fractional* output_block1_buffer[SYLVATICA_N_CHANNELS];
fractional* output_block2_buffer[SYLVATICA_N_CHANNELS];
fractional* input_block1_buffer[SYLVATICA_N_CHANNELS];
fractional* input_block2_buffer[SYLVATICA_N_CHANNELS];
fractional* input_block3_buffer[SYLVATICA_N_CHANNELS];
uint8_t output_buffer0_select = 0, output_buffer1_select = 0, output_buffer2_select = 0;
fractional output_block0_buffers_a[SYLVATICA_N_CHANNELS][SYLVATICA_BLOCK1_INPUT_SIZE];
fractional output_block1_buffers_a[SYLVATICA_N_CHANNELS][SYLVATICA_BLOCK2_INPUT_SIZE];
fractional output_block2_buffers_a[SYLVATICA_N_CHANNELS][SYLVATICA_BLOCK3_INPUT_SIZE];
fractional output_block0_buffers_b[SYLVATICA_N_CHANNELS][SYLVATICA_BLOCK1_INPUT_SIZE];
fractional output_block1_buffers_b[SYLVATICA_N_CHANNELS][SYLVATICA_BLOCK2_INPUT_SIZE];
fractional output_block2_buffers_b[SYLVATICA_N_CHANNELS][SYLVATICA_BLOCK3_INPUT_SIZE];

fractional output_buffer_a[SYLVATICA_N_CHANNELS];

fractional raw_output_buffer_a[SYLVATICA_N_CHANNELS];

i2c_message_t i2c_mr_message;
uint8_t i2c_mr_message_data[SYLVATICA_I2C_READ_BUFFER_LENGTH];

volatile sylvatica_reg_t i2c_write_reg = SYLVATICA_REG_STATUS;

i2c_message_t i2c_channel_message[SYLVATICA_N_CHANNELS];
i2c_message_t i2c_raw_channel_message[SYLVATICA_N_CHANNELS];
uint8_t i2c_channel_data[SYLVATICA_N_CHANNELS][SYLVATICA_I2C_READ_CH_BUFFER_LENGTH];
uint8_t i2c_raw_channel_data[SYLVATICA_N_CHANNELS][SYLVATICA_I2C_READ_CH_BUFFER_LENGTH];

sylvatica_status_t sylvatica_status = SYLVATICA_STATUS_INIT;

void sylvatica_init_i2c_message_buffers(void){
    uint16_t i;
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        i2c_channel_message[i].data = i2c_channel_data[i];
        i2c_raw_channel_message[i].data = i2c_raw_channel_data[i];
        
        i2c_channel_data[i][0] = 2;
        i2c_raw_channel_data[i][0] = 2;
    }
    
}

uint8_t i2c_get_address_sylvatica(sylvatica_config_t* config){
    uint16_t address = 0x00, i;
    for(i = 0; i < SYLVATICA_N_ADDRESS_SEL_PINS; i++){
        address |= (GET_BIT(config->address_selection[i].port_r, config->address_selection[i].n) << i);
    }

    address = ~address;
    address &= 0x7f;
    
    // if the address is not valid, use the default one
    if(!i2c_check_address(address)){
        address = SYLVATICA_I2C_BASE_ADDRESS;
    }   
    
    return address;
}

void init_pins_sylvatica(void){
    // I2C
    _ODCG2 = 1; // configure I2C pins as open drain output
    _ODCG3 = 1; // configure I2C pins as open drain output
    _TRISG2 = 0;
    _TRISG3 = 0;
    _ODCD0 = 1; // nINT
    _TRISD0 = 0; 
    _RD0 = 1;
    
    // SPI1 for PGA
    _TRISF5 = 0;            // SCK
    _RP101R = _RPOUT_SCK1;
    _ANSE2 = 0;             // SCK
    _TRISE2 = 0;
    _RP82R = _RPOUT_SCK1;
    _ANSE4 = 0;             // SDO
    _TRISE4 = 0;
    _RP84R = _RPOUT_SDO1;
    _ANSE3 = 0;             // CS1
    _TRISE3 = 0;
    _RE3 = 1;
    _TRISD11 = 0;           // CS2
    _RD11 = 1;
    _ANSE6 = 0;             // CS3
    _TRISE6 = 0;
    _RE6 = 1;
    _ANSE7 = 0;             // CS4
    _TRISE7 = 0;
    _RE7 = 1;
    _ANSB11 = 0;            // CS5
    _TRISB11 = 0;
    _RB11 = 1;
    _ANSB10 = 0;            // CS6
    _TRISB10 = 0;
    _RB10 = 1;
    _ANSB9 = 0;             // CS7
    _TRISB9 = 0;
    _RB9 = 1;
    _ANSB8 = 0;             // CS8
    _TRISB8 = 0;
    _RB8 = 1;
    
    // SPI2 for ADC
    _ANSE5 = 0;         // CS ADC
    _TRISE5 = 0;
    _RP85R = _RPOUT_OC4;
    _ANSG6 = 0;         // SCK
    _TRISG6 = 0;
    _ANSG7 = 0;         // SDI
    _TRISG7 = 1;
    _ANSG8 = 0;         // SDO
    _TRISG8 = 0;
    
    // ADC signals
    _ANSB14 = 0;        // INT ADC
    _TRISB14 = 0;
    _IC1R = 46;
    _ANSB15 = 0;        // RST ADC
    _TRISB15 = 0;
    _TRISF3 = 0;        // nCONV 
    _RP99R = _RPOUT_OC15;   
    
    // address readout
    _ANSE0 = 0;
    _TRISE0 = 1;
    _CNPUE0 = 1; // A0
    _ANSE1 = 0;
    _TRISE1 = 1;
    _CNPUE1 = 1; // A1
    _TRISF1 = 1;
    _CNPUF1 = 1; // A2
    _ANSB0 = 0;
    _TRISB0 = 1;
    _CNPUB0 = 1; // A3
    _ANSB1 = 0;
    _TRISB1 = 1;
    _CNPUB1 = 1; // A4
    _ANSB2 = 0;
    _TRISB2 = 1;
    _CNPUB2 = 1; // A5
    _ANSB3 = 0;
    _TRISB3 = 1;
    _CNPUB3 = 1; // A6
    _ANSB4 = 0;
    _TRISB4 = 1;
    _CNPUB4 = 1; // A7

    // UART configuration
    _ANSD6 = 0; // RXD
    _TRISD6 = 1;
    _U2RXR = 70;
    _TRISD5 = 0; // RTS
    _RP69R = _RPOUT_U2RTS;
    _TRISD4 = 1; // CTS
    _U2CTSR = 68;
    _TRISD3 = 0; // TXD
    _RP67R = _RPOUT_U2TX;
    
    // PGA calibration
    _ANSC14 = 0;  // REF enable
    _TRISC14 = 0;
    _LATC14 = 1;
    _TRISD2 = 0; // REF C1
    _LATD2 = 1;

    // blinky and error (shared)
    _ANSC13 = 0;
    _TRISC13 = 0;
    _LATC13 = 0;
    
    // 4V LDO control
    _ANSB5 = 0;
    _TRISB5 = 0;
    _LATB5 = 1;
}

void init_sylvatica(void){
    uint16_t i;
    
    // error function
    set_error_loop_fn(i2c1_detect_stop);
   
    init_pins_sylvatica();
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised pins.");
    uart_print(print_buffer, strlen(print_buffer));
#endif    

    // read I2C slave address (sets slave address for communication with dicio)
    gconfig.i2c_config.i2c_address = i2c_get_address_sylvatica(&gconfig);
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised I2C slave address to 0x%x.", gconfig.i2c_config.i2c_address);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    sylvatica_init_i2c_message_buffers();
    
    i2c1_init(&gconfig.i2c_config);
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised I2C.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    spi1_init();
    spi2_init();
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised SPI.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        gconfig.pga_config[i].status = PGA_STATUS_ON;
        init_pga(&gconfig.pga_config[i]);
    }
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised PGAs.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    blinky_init(&gconfig.blinky_pin, 1);
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised blinky.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    init_filters();
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised filters.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    init_adc(&gconfig.adc_config);
        
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Initialised ADC.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
#ifdef ENABLE_DEBUG        
    for(i = 0; i < 8; i++){
        uint8_t pga_config_data = SYLVATICA_CH_CONFIG_ON;

        sylvatica_i2c_channel_config(i, &pga_config_data);
    }
#endif
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Running calibration...");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    sylvatica_run_calibration(&gconfig);
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Running gain calibration...");
    uart_print(print_buffer, strlen(print_buffer));

    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        sylvatica_gain_calibration(&gconfig, i);
    }
#endif
    
    sylvatica_status = SYLVATICA_STATUS_READY;
    
/*#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Starting ADC.");
    uart_print(print_buffer, strlen(print_buffer));
    
    adc_start(&gconfig.adc_config);
#endif*/
    
#ifdef ENABLE_DEBUG
    //sprintf(print_buffer, "Initialising calibration timer.");
    //uart_print(print_buffer, strlen(print_buffer));
#endif
    //init_calibration_timer();
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        output_block0_buffer[i] = output_block0_buffers_b[i];
        output_block1_buffer[i] = output_block1_buffers_b[i];
        output_block2_buffer[i] = output_block2_buffers_b[i];
        
        input_block1_buffer[i] = output_block0_buffers_a[i];
        input_block2_buffer[i] = output_block1_buffers_a[i];
        input_block3_buffer[i] = output_block2_buffers_a[i];
    }
}


void loop_sylvatica(void){  
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "Executing main loop.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    while(1){
        
        while(gconfig.adc_config.status != ADC_STATUS_ON){
            i2c1_detect_stop();
        }      

        sylvatica_status = SYLVATICA_STATUS_RUNNING;
        
        while(gconfig.adc_config.status == ADC_STATUS_ON){

            i2c1_detect_stop();

            if(start_filter_block0){
                process_filter_block0(); 
            }
            
            i2c1_detect_stop();

            if(start_filter_block1){
                process_filter_block1();
                continue;
            }
            
            i2c1_detect_stop();

            if(start_filter_block2){
                process_filter_block2();
                continue;
            }
            
            i2c1_detect_stop();

            if(start_filter_block3){
                process_filter_block3();
                continue;
            }
        }
    }
}

void adc_rx_callback(void){
    uint16_t i;
    static uint16_t copy_counter = 0;
    
    if (adc_buffer_selector == 0) {
        if(copy_buffer_selector == 0){
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                copy_adc_data(ADC_BUFFER_LENGTH / SYLVATICA_N_CHANNELS, (fractional*) &copy_buffers_a[i][copy_counter], (fractional*) &adc_rx_buffer_a[i]);
            }
        } else {
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                copy_adc_data(ADC_BUFFER_LENGTH / SYLVATICA_N_CHANNELS, (fractional*) &copy_buffers_b[i][copy_counter], (fractional*) &adc_rx_buffer_a[i]);
            }
        }
    } else {
        if(copy_buffer_selector == 0){
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                copy_adc_data(ADC_BUFFER_LENGTH / SYLVATICA_N_CHANNELS, (fractional*) &copy_buffers_a[i][copy_counter], (fractional*) &adc_rx_buffer_b[i]);
            }
        } else {
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                copy_adc_data(ADC_BUFFER_LENGTH / SYLVATICA_N_CHANNELS, (fractional*) &copy_buffers_b[i][copy_counter], (fractional*) &adc_rx_buffer_b[i]);
            }
        }
    }
    copy_counter += ADC_BUFFER_LENGTH / SYLVATICA_N_CHANNELS;
    
    if(copy_counter == SYLVATICA_COPY_BUFFER_SIZE){
        start_filter_block0 = 1;
        copy_buffer_selector ^= 1;
        copy_counter = 0;
    }
    
    adc_buffer_selector ^= 1;
}

void process_filter_block0(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    static uint16_t sample_counter = 0;
    fractional sample_buffer[SYLVATICA_COPY_BUFFER_SIZE];
    
    start_filter_block0 = 0;
    
    sample_counter += SYLVATICA_COPY_BUFFER_SIZE;
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){   
        if(copy_buffer_selector == 0){
            covert_uint_to_fract(SYLVATICA_COPY_BUFFER_SIZE, copy_buffers_b[i], sample_buffer);
        } else {
            covert_uint_to_fract(SYLVATICA_COPY_BUFFER_SIZE, copy_buffers_a[i], sample_buffer);
        }
        
        
        if(sample_counter == 10000){
            _SI2C1IE = 0;
            i2c_raw_channel_data[i][1] = (uint8_t) ((sample_buffer[0] >> 8) & 0xff);
            i2c_raw_channel_data[i][2] = (uint8_t) (sample_buffer[0] & 0xff);
            
            i2c_init_read_message(
                &i2c_raw_channel_message[i],
                i2c_raw_channel_data[i],
                SYLVATICA_I2C_READ_CH_BUFFER_LENGTH);
            
            _SI2C1IE = 1;
        }
            
        fir_compressed(SYLVATICA_BLOCK0_OUTPUT_SIZE, 
                &output_block0_buffer[i][block_counter], 
                sample_buffer, 
                &filters_0[i], 
                SYLVATICA_DEC_FACT_F0);
    }
    
    if(sample_counter == 10000){
        sample_counter = 0;
    }
    
    block_counter += SYLVATICA_BLOCK0_OUTPUT_SIZE;
    if(block_counter == SYLVATICA_BLOCK1_INPUT_SIZE){
        start_filter_block1 = 1;
        block_counter = 0;
        
        output_buffer0_select ^= 1;
        if(output_buffer0_select == 0){
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                output_block0_buffer[i] = output_block0_buffers_b[i];
                input_block1_buffer[i] = output_block0_buffers_a[i];
            }
        } else {
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                output_block0_buffer[i] = output_block0_buffers_a[i];
                input_block1_buffer[i] = output_block0_buffers_b[i];
            }
        }
    }
}

void process_filter_block1(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    
    start_filter_block1 = 0;
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){   
            
        fir_compressed(SYLVATICA_BLOCK1_OUTPUT_SIZE, 
                &output_block1_buffer[i][block_counter], 
                input_block1_buffer[i], 
                &filters_1[i], 
                SYLVATICA_DEC_FACT_F1);
    }
    block_counter += SYLVATICA_BLOCK1_OUTPUT_SIZE;
    if(block_counter == SYLVATICA_BLOCK2_INPUT_SIZE){
        start_filter_block2 = 1;
        block_counter = 0;
        
        output_buffer1_select ^= 1;
        if(output_buffer1_select == 0){
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                output_block1_buffer[i] = output_block1_buffers_b[i];
                input_block2_buffer[i] = output_block1_buffers_a[i];
            }
        } else {
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                output_block1_buffer[i] = output_block1_buffers_a[i];
                input_block2_buffer[i] = output_block1_buffers_b[i];
            }
        }
        
    }
}

void process_filter_block2(void){
    uint16_t i;
    static uint16_t block_counter = 0;
    
    start_filter_block2 = 0;
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){   
        fir_compressed(SYLVATICA_BLOCK2_OUTPUT_SIZE, 
                &output_block2_buffer[i][block_counter], 
                input_block2_buffer[i], 
                &filters_2[i], 
                SYLVATICA_DEC_FACT_F2);
    }
    block_counter += SYLVATICA_BLOCK2_OUTPUT_SIZE;
    if(block_counter == SYLVATICA_BLOCK3_INPUT_SIZE){
        block_counter = 0;
        start_filter_block3 = 1;
        
        output_buffer2_select ^= 1;
        if(output_buffer2_select == 0){
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                output_block2_buffer[i] = output_block2_buffers_b[i];
                input_block3_buffer[i] = output_block2_buffers_a[i];
            }
        } else {
            for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
                output_block2_buffer[i] = output_block2_buffers_a[i];
                input_block3_buffer[i] = output_block2_buffers_b[i];
            }
        }
        
    }
}


void process_filter_block3(void){
    uint16_t i;
    
    start_filter_block3 = 0;
    
    _SI2C1IE = 0;
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        i2c_channel_data[i][1] = (uint8_t) ((output_buffer_a[i] >> 8) & 0xff);
        i2c_channel_data[i][2] = (uint8_t) (output_buffer_a[i] & 0xff);

        i2c_init_read_message(
            &i2c_channel_message[i],
            i2c_channel_data[i],
            SYLVATICA_I2C_READ_CH_BUFFER_LENGTH);
    }
    CLEAR_PORT_BIT(gconfig.int_pin);
    _SI2C1IE = 1;
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){   
        fir_compressed(SYLVATICA_BLOCK3_OUTPUT_SIZE, 
                &output_buffer_a[i], 
                input_block3_buffer[i], 
                &filters_3[i], 
                SYLVATICA_DEC_FACT_F3);
    }
}

// called after full message transmitted
void i2c_mr_sw_cb_sylvatica(i2c_message_t* m){
    // HERE WE CAN DO ERROR HANDLING, but optional
}

// called after full write message received
void i2c_mw_sr_cb_sylvatica(i2c_message_t* m){
    
    // master sends data to slave (last bit is 0)
    if(m->data_length < 1){
        return;
    }
    if(m->data[0] >= SYLVATICA_N_REG){
        return;
    }
        
    i2c_write_reg = (sylvatica_reg_t) m->data[0];
    
    
    if((i2c_write_reg >= SYLVATICA_REG_DATA_CH0) && (i2c_write_reg <= SYLVATICA_REG_DATA_CH7)){
        sylvatica_i2c_read_copy_channel_data(i2c_write_reg - SYLVATICA_REG_DATA_CH0);
    } else if((i2c_write_reg >= SYLVATICA_REG_RAW_CH0) && (i2c_write_reg <= SYLVATICA_REG_RAW_CH7)) {
        sylvatica_i2c_read_copy_raw_data(i2c_write_reg - SYLVATICA_REG_RAW_CH0);
    } else if((i2c_write_reg >= SYLVATICA_REG_CONFIG_CH0) && (i2c_write_reg <= SYLVATICA_REG_CONFIG_CH7)){
        if(m->data_length >= 2){ 
            sylvatica_i2c_channel_config(i2c_write_reg - SYLVATICA_REG_CONFIG_CH0, &m->data[1]);
        }
        sylvatica_i2c_read_config(i2c_write_reg);
    } else if(i2c_write_reg == SYLVATICA_REG_STATUS){
        if(m->data_length >= 2){      
            if((m->data[1] & SYLVATICA_STATUS_OFF) == SYLVATICA_STATUS_OFF){
                // TODO: go into low power mode
            }

            if((m->data[1] & SYLVATICA_STATUS_RESET_BUFFER) == SYLVATICA_STATUS_RESET_BUFFER){
                sylvatica_clear_buffers();
            }

            if((m->data[1] & SYLVATICA_STATUS_RESET) == SYLVATICA_STATUS_RESET){
                asm ("RESET");
            }
        }
        sylvatica_i2c_read_config(i2c_write_reg);
    } else if(i2c_write_reg == SYLVATICA_REG_ADC) {
        if(m->data_length >= 2){ 
            if((m->data[1] & SYLVATICA_ADC_ON) == SYLVATICA_ADC_ON){
                adc_start(&gconfig.adc_config);
            } else {
                gconfig.adc_config.status = ADC_STATUS_IDLE;
                adc_stop(&gconfig.adc_config);
            }
        }
        sylvatica_i2c_read_config(i2c_write_reg);
    }
}

void sylvatica_i2c_read_config(const sylvatica_reg_t reg){
    if(reg == SYLVATICA_REG_ADC){
        i2c_mr_message_data[0] = 1;
        
        if(gconfig.adc_config.status == ADC_STATUS_IDLE){
            i2c_mr_message_data[1] = 0;
        } else {
            i2c_mr_message_data[1] = 1;
        }

        i2c_init_read_message(
            &i2c_mr_message,
            i2c_mr_message_data,
            1+1);
    } else if((reg >= SYLVATICA_REG_CONFIG_CH0) && (reg <= SYLVATICA_REG_CONFIG_CH7)){
        uint8_t channel_n = reg - SYLVATICA_REG_CONFIG_CH0;
        
        // set number of bytes
        i2c_mr_message_data[0] = 1;
        
        // configuration byte
        switch(gconfig.channel_status[channel_n]){
            case SYLVATICA_CHANNEL_ENABLED:
                i2c_mr_message_data[1] = 1;
                break;
            case SYLVATICA_CHANNEL_DISABLED:
                i2c_mr_message_data[1] = 0;
                break;
        }

        i2c_mr_message_data[1] = (i2c_mr_message_data[0] << 3) | SYLVATICA_CH_CONFIG_SET_GAIN(gconfig.pga_config[channel_n].gain);
        
        i2c_init_read_message(
            &i2c_mr_message,
            i2c_mr_message_data,
            1+1);
    } else if(reg == SYLVATICA_REG_STATUS) {
        
        i2c_mr_message_data[0] = 1;
        i2c_mr_message_data[1] = 0;
        i2c_mr_message_data[1] |= sylvatica_status;
        
        i2c_init_read_message(
            &i2c_mr_message,
            i2c_mr_message_data,
            1+1);
        
    } else {
        return;
    }
    
    i2c_set_read_message(&i2c_mr_message);

    SET_PORT_BIT(gconfig.int_pin);
}

void sylvatica_i2c_read_copy_channel_data(const uint8_t channel_n){  
    if((channel_n > 7) || (channel_n < 0)){
        i2c_init_read_message(
            &i2c_mr_message,
            i2c_mr_message_data,
            0);
        
        i2c_set_read_message(&i2c_mr_message);
    } else {
        i2c_set_read_message(&i2c_channel_message[channel_n]);
    }    

    SET_PORT_BIT(gconfig.int_pin);
}

void sylvatica_i2c_read_copy_raw_data(const uint8_t channel_n){
    if((channel_n > 7) || (channel_n < 0)){
        i2c_init_read_message(
            &i2c_mr_message,
            i2c_mr_message_data,
            0);
        
        i2c_set_read_message(&i2c_mr_message);
    } else {
        i2c_set_read_message(&i2c_raw_channel_message[channel_n]);
    }
    
    SET_PORT_BIT(gconfig.int_pin);
}

void sylvatica_i2c_channel_config(const uint8_t channel_n, uint8_t* data){
    uint8_t gain;
    
    if(gconfig.adc_config.status == ADC_STATUS_ON){
        return;
    }
    
    // update configuration fields
    if((data[0] & SYLVATICA_CH_CONFIG_ON) == SYLVATICA_CH_CONFIG_ON){
        gconfig.channel_status[channel_n] = SYLVATICA_CHANNEL_ENABLED;
        gconfig.pga_config[channel_n].status = PGA_STATUS_ON;
    } else {
        gconfig.channel_status[channel_n] = SYLVATICA_CHANNEL_DISABLED;
        gconfig.pga_config[channel_n].status = PGA_STATUS_OFF;
    }
    
    gain = ((data[0] & SYLVATICA_CH_CONFIG_GAIN) >> 3);
    
    if(gain > PGA_GAIN_200){
        sylvatica_gain_calibration(&gconfig, channel_n);
    } else {
        gconfig.pga_config[channel_n].gain = (pga_gain_t) gain; 
    }
    
    // update actual hardware
    update_pga_status(&gconfig.pga_config[channel_n]);
}

void sylvatica_clear_buffers(void){
    uint16_t i, j;
    
    for(i = 0; i < ARRAY_LENGTH(output_block0_buffers_a); i++){
        for(j = 0; j < ARRAY_LENGTH(output_block0_buffers_a[0]); j++){
            output_block0_buffers_a[i][j] = 0;
            output_block0_buffers_b[i][j] = 0;
        }
    }
    for(i = 0; i < ARRAY_LENGTH(output_block1_buffers_a); i++){
        for(j = 0; j < ARRAY_LENGTH(output_block1_buffers_a[0]); j++){
            output_block1_buffers_a[i][j] = 0;
            output_block1_buffers_b[i][j] = 0;
        }
    }
    for(i = 0; i < ARRAY_LENGTH(output_block2_buffers_a); i++){
        for(j = 0; j < ARRAY_LENGTH(output_block2_buffers_a[0]); j++){
            output_block2_buffers_a[i][j] = 0;
            output_block2_buffers_b[i][j] = 0;
        }
    }
    for(i = 0; i < ARRAY_LENGTH(output_buffer_a); i++){
        output_buffer_a[i] = 0;
    }
    for(i = 0; i < ARRAY_LENGTH(copy_buffers_a); i++){
        for(j = 0; j < ARRAY_LENGTH(copy_buffers_a[0]); j++){
            copy_buffers_a[i][j] = 0;
            copy_buffers_b[i][j] = 0;
        }
    }
    
    i2c_init_read_message(
        &i2c_mr_message,
        i2c_mr_message_data,
        0);
    
    i2c_set_read_message(&i2c_mr_message);
}
