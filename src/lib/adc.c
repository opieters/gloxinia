#include <adc.h>
#include <utilities.h>
#include <device_configuration.h>
#include <dsp.h>
#include <fir_common.h>

#include <spi.h>

#ifdef ENABLE_DEBUG
#include <uart.h>
#include <stdio.h>
#include <string.h>
#endif

unsigned int adc_tx_buffer[ADC_BUFFER_LENGTH] __attribute__((space(dma), eds));
unsigned int adc_rx_buffer_a[ADC_BUFFER_LENGTH]__attribute__((space(dma), eds, address(0xD9C0)));
unsigned int adc_rx_buffer_b[ADC_BUFFER_LENGTH] __attribute__((space(dma), eds, address(0xD830)));

unsigned int adc_debug_buffer_a[ADC_BUFFER_LENGTH];
unsigned int adc_debug_buffer_b[ADC_BUFFER_LENGTH];

void (*rx_callback)(void) = rx_callback_dummy;

void rx_callback_dummy(void){
    
}

uint16_t adc_parse_cfr_write(adc_config_t* config){
    uint16_t word = 0xE000;

    word |= (config->channel_select << 11) 
            | (config->conversion_clock_source << 10) 
            | (config->trigger_select << 9) 
            | (config->auto_trigger_rate << 8)
            | (config->pin10_polarity << 7)
            | (config->pin10_output << 6)
            | (config->pin10_io << 5)
            | (config->auto_nap << 4)
            | (config->nap_powerdown << 3)
            | (config->deep_powerdown << 2)
            | (config->tag_output << 1)
            | (config->sw_reset);

    return word;
}

void update_adc(adc_config_t* config){
    spi_message_t m;
    uint16_t write_data[1];
    uint16_t read_data[1];
    void (*send_spi_message) (spi_message_t* m);
    
    send_spi_message = spi_get_handler(config->spi_module);
  
    // update callback
    rx_callback = config->rx_callback;
        
    // disable sampling
    OC15CON1bits.OCM = 0b000;
    
    // wake ADC
    write_data[0] = 0xB000;
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    send_spi_message(&m);
    
    // select actual channel in config
    write_data[0] = ADC_COMMAND_TO_BITS(config->channel);
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    send_spi_message(&m);
    
    // update status
    write_data[0] = adc_parse_cfr_write(config);
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    send_spi_message(&m);
    
    if(config->deep_powerdown == ADC_DEEP_POWERDOWN_ENABLE){
        config->status = ADC_STATUS_OFF;
    } else {
        config->status = ADC_STATUS_IDLE;
    }
    
    OC15CON1bits.OCM = 0b000;          // Disable Output Compare Module
    OC15R = (FCY / config->sample_frequency) - 5;   // Write the duty cycle for the PWM pulse
    OC15RS = (FCY / config->sample_frequency) -1;   // Write the PWM frequency
    
    // configure the CS pin    
    OC4CON1bits.OCM = 0b000; // Disable Output Compare Module
    OC4R = (FCY / config->sample_frequency) - 120;   // Write the duty cycle for the PWM pulse
    OC4RS = (FCY / config->sample_frequency) - 1;   // frequency
}

void init_adc(adc_config_t* config){
    uint16_t i, eds_read;
    spi_message_t m;
    uint16_t write_data[2];
    uint16_t read_data[2];
    read_data[0] = 0x0;
    void (*send_spi_message) (spi_message_t* m);
    adc_channel_select_mode_t selection_config;
    adc_channel_t original_channel = config->channel;
    
    // configure reset pin and set high)
    CLEAR_BIT(config->rst_pin.tris_r, config->rst_pin.n);
    SET_BIT(config->rst_pin.lat_r, config->rst_pin.n);
    
    // hardware reset of ADC
    delay_us(100);
    CLEAR_BIT(config->rst_pin.lat_r, config->rst_pin.n);
    delay_us(100);
    SET_BIT(config->rst_pin.lat_r, config->rst_pin.n);
    
    // configure DMA4 to write data over SPI
    _DMA4IF = 0;
    _DMA4IE = 0;
    
    DMA4CONbits.SIZE = 0; // word data transfer
    DMA4CONbits.HALF = 0; // interrupt when all data is transfered
    DMA4CONbits.NULLW = 0; // normal operation
    DMA4CONbits.DIR = 1; // read from RAM to peripheral
    DMA4CONbits.AMODE = 0; // register indirect with post increment
    DMA4CONbits.MODE = 0; // continuous ping-pong mode
    
    DMA4REQbits.IRQSEL = 0b00011010; // output compare 4
    //DMA4REQbits.IRQSEL = 0b00000001; // select input compare 1 as trigger source
    
    DMA4STAL = __builtin_dmaoffset(adc_tx_buffer);
    DMA4STAH = __builtin_dmapage(adc_tx_buffer);
    
    switch(config->spi_module){
        case SPI_MODULE_SELECTOR_1:
            DMA4PAD = (volatile unsigned int) &SPI1BUF; // peripheral address
            break;
        case SPI_MODULE_SELECTOR_2:
            DMA4PAD = (volatile unsigned int) &SPI2BUF; // peripheral address
            break;
        case SPI_MODULE_SELECTOR_3:
            DMA4PAD = (volatile unsigned int) &SPI3BUF; // peripheral address
            break;
        default:
            report_error("ADC: unsupported SPI module.");
            break;
    }
    
    DMA4CNT = ADC_BUFFER_LENGTH-1;
   
    _DMA4IE = 0;
    
    // configure DMA5 to read data over SPI
    _DMA5IF = 0;
    _DMA5IE = 0;
    
    DMA5CONbits.SIZE = 0; // word data transfer
    DMA5CONbits.HALF = 0; // interrupt when all data is transfered
    DMA5CONbits.NULLW = 0; // normal operation
    DMA5CONbits.DIR = 0; // read from peripheral to RAM
    DMA5CONbits.AMODE = 0; // register indirect with post increment
    DMA5CONbits.MODE = 2; // continuous ping-pong mode
    
    switch(config->spi_module){
        case SPI_MODULE_SELECTOR_1:
            DMA5REQbits.IRQSEL = 0b00001010; // select SPI1 transfer done as trigger source
            DMA5PAD = (volatile unsigned int) &SPI1BUF; // peripheral address
            break;
        case SPI_MODULE_SELECTOR_2:
            DMA5REQbits.IRQSEL = 0b00100001; // select SPI2 transfer done as trigger source
            DMA5PAD = (volatile unsigned int) &SPI2BUF; // peripheral address
            break;
        case SPI_MODULE_SELECTOR_3:
            DMA5REQbits.IRQSEL = 0b01011011; // select SPI3 transfer done as trigger source
            DMA5PAD = (volatile unsigned int) &SPI3BUF; // peripheral address
            break;
        default:
            report_error("ADC: unsupported SPI module.");
            break;
    }
    
    DMA5STAL = __builtin_dmaoffset(adc_rx_buffer_a); // buffer A
    DMA5STAH = __builtin_dmapage(adc_rx_buffer_a); // buffer A
    DMA5STBL = __builtin_dmaoffset(adc_rx_buffer_b); // buffer B
    DMA5STBH = __builtin_dmapage(adc_rx_buffer_b);
    
    DMA5CNT = ADC_BUFFER_LENGTH-1;
    
    _DMA5IE = 1;
    
    // update callback
    rx_callback = config->rx_callback;
    
    // fill buffers with zeros and read command
    eds_read = DSRPAG;
    DSRPAG = __builtin_edspage(adc_rx_buffer_a);
    for(i = 0; i < ADC_BUFFER_LENGTH; i++){
        adc_tx_buffer[i] = ADC_COMMAND_TO_BITS(ADC_READ_DATA);
        
        adc_rx_buffer_a[i] = 0;
        adc_rx_buffer_b[i] = 0;
    }
    DSRPAG = eds_read;
    
    // configure input capture module 1 to detect nEOC edge and start
    // SPI data transfer
    
    _IC1IF = 0;
    _IC1IE = 0;
    
    IC1CON1bits.ICSIDL = 0; // continue operation in CPU idle mode
    IC1CON1bits.ICTSEL = 0b111; // peripheral clock is clock source
    IC1CON1bits.ICI = 0b00; // interrupt after every match
    IC1CON1bits.ICM = 0b010; // capture every falling edge
    
    IC1CON2bits.IC32 = 0; // disable cascade mode
    IC1CON2bits.TRIGSTAT = 0; // no trigger has occurred
    IC1CON2bits.ICTRIG = 1; // trigger mode
    IC1CON2bits.SYNCSEL = 0; // no sync or trigger source
    IC1CON2 = 0;
    _IC1IE = 0;
    
    // update the configuration and prepare for conversion
    config->status = ADC_STATUS_IDLE;
    
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    
    send_spi_message = spi_get_handler(config->spi_module);
        
    // disable sampling
    OC15CON1bits.OCM = 0b000;
    
    // wake ADC
    m.status = SPI_TRANSFER_PENDING;
    write_data[0] = 0xB000;
    send_spi_message(&m);
    
    // store channel selectioin setting
    selection_config = config->channel_select;
    config->channel_select = ADC_CHANNEL_SELECT_MODE_MANUAL;
    
    // update configuration to read all registers once in manual mode
    write_data[0] = adc_parse_cfr_write(config);
    m.status = SPI_TRANSFER_PENDING;
    send_spi_message(&m);
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "ADC config set: %04x.", m.write_data[0]);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    write_data[0] = ADC_COMMAND_TO_BITS(ADC_READ_CFR);
    m.status = SPI_TRANSFER_PENDING;
    send_spi_message(&m);
        
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "ADC config read: %04x.", m.read_data[0]);
    uart_print(print_buffer, strlen(print_buffer));
#endif
        
    // read all channels once in manual mode
    
    for(i = 0; i < N_ADC_CHANNELS; i++){
        config->channel = i;
        read_data[0] = read_adc_channel(config);
        
        covert_uint_to_fract(1, &m.read_data[0], (fractional*) &m.read_data[1]);
    
#ifdef ENABLE_DEBUG
        sprintf(print_buffer, "ADC read %x: %04x, %.6f (%.6f)", i, m.read_data[0], ((double) m.read_data[0]) / 0x10000, (double) Fract2Float((fractional) m.read_data[1]));
        uart_print(print_buffer, strlen(print_buffer));
#endif
    
        delay_ms(10); 
    }
    
    config->channel = original_channel;
    
    // select actual channel in config
    write_data[0] = ADC_COMMAND_TO_BITS(config->channel);
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    send_spi_message(&m);
    
    // update ADC channel to final trigger mode
    config->channel_select = selection_config;
    
    // update status config
    write_data[0] = adc_parse_cfr_write(config);
    
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "ADC config set: %04x.", m.write_data[0]);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    send_spi_message(&m);
    
    write_data[0] = ADC_COMMAND_TO_BITS(ADC_READ_CFR);
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    send_spi_message(&m);
        
#ifdef ENABLE_DEBUG
    sprintf(print_buffer, "ADC config read: %04x.", m.read_data[0]);
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    
    if(config->deep_powerdown == ADC_DEEP_POWERDOWN_ENABLE){
        config->status = ADC_STATUS_OFF;
    }
    
    // configure OC15 module to generate nCONVST signal to trigger sampling
    // minimum pulse width = 40ns (or 3 clock cycles) and there must be 1500ns 
    // (96 cycli) between conversions
    
    OC15CON1bits.OCM = 0b000;          // Disable Output Compare Module
    OC15R = (FCY / config->sample_frequency) - 5;   // Write the duty cycle for the PWM pulse
    OC15RS = (FCY / config->sample_frequency) -1;   // Write the PWM frequency
    OC15CON1bits.OCTSEL = 0b111;       // Select peripheral clock as output compare time base
    OC15CON2bits.SYNCSEL=0b11111;      // no sync or trigger source
    
    //OC15CON2bits.OCINV = 1; // invert output
    
    _OC15IF = 0;              // clear the OC15 interrupt flag
    _OC15IE = 0;              // disable OC155 interrupt
    
    // configure the CS pin    
    OC4CON1bits.OCM = 0b000; // Disable Output Compare Module
    // SPI configured to operate at 64MHz/7 -> 16 periods needed -> 16*7=112 cycles needed
    OC4R = (FCY / config->sample_frequency) - 120;   // Write the duty cycle for the PWM pulse
    OC4RS = (FCY / config->sample_frequency) - 1;   // frequency
    OC4CON1bits.OCSIDL = 0; // continue operation in CPU idle mode
    OC4CON1bits.OCTSEL = 0b111; // user peripheral clock as clock source
    OC4CON2bits.SYNCSEL = 0b11111; // no sync or trigger source
    
    OC4CON2bits.OCINV = 1; // invert output
    
    _OC4IF = 0;              // clear the OC4 interrupt flag
    _OC4IE = 0;              // disable OC4 interrupt        
}

void __attribute__((__interrupt__, no_auto_psv)) _OC15Interrupt(void) {    
    _OC15IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _OC4Interrupt(void) {    
    _OC4IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void) {    
    _IC1IF = 0;
}

void adc_start(adc_config_t* config){
    // reset DMA transfer lengths (just in case)
    DMA5CNT = ADC_BUFFER_LENGTH-1;
    DMA4CNT = ADC_BUFFER_LENGTH-1;
    
    config->status = ADC_STATUS_ON;
    
    // update SPI transfer mode
    switch(config->spi_module){
        case SPI_MODULE_SELECTOR_1:
            adc_spi1_init();
            break;
        case SPI_MODULE_SELECTOR_2:
            adc_spi2_init();
            break;
        case SPI_MODULE_SELECTOR_3:
            adc_spi3_init();
            break;
        default:
            report_error("ADC: unsupported SPI module.");
            break;
    }

    // start SPI transfers
    DMA5CONbits.CHEN = 1; // enable DMA channel
    DMA4CONbits.CHEN = 1; // enable DMA channel

    asm volatile (
    "mov OC4CON1, w0  \n"
    "mov OC15CON1, w1 \n"
    "ior #0b110, w0 \n" // Select the Output Compare mode: EDGE-ALIGNED PWM MODE
    "ior #0b110, w1 \n" // Select the Output Compare mode: EDGE-ALIGNED PWM MODE
    "mov #112, w2\n"
    //"mov OC4R, w3\n"
    //"sub w3, w2, w2\n"
    "sub #4, w2\n"
    "mov w1, OC15CON1   \n"
    "repeat w2 \n"
    "nop \n"
    "mov w0, OC4CON1    \n"
    );
}

void adc_stop(adc_config_t* config){    
    config->status = ADC_STATUS_OFF;
    
    OC15CON1bits.OCM = 0b000;          // disable OC1 module

    // disable DMA transfer registers
    DMA5CONbits.CHEN = 0; // enable DMA channel
    DMA4CONbits.CHEN = 0; // enable DMA channel

    SET_BIT(config->cs_pin.lat_r, config->cs_pin.n);
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA5Interrupt(void) {  
    rx_callback();
    _DMA5IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA4Interrupt(void){
    _DMA4IF = 0;
}

void adc_spi1_init(void){
    _SPI1IE = 0;
}

void adc_spi2_init(void){
    _SPI2IE = 0;
}

void adc_spi3_init(void){
    _SPI3IE = 0;
}

uint16_t read_adc_channel(adc_config_t* config){
    uint16_t write_data[2], read_data[2];
    spi_message_t m;
    spi_handler_t spi_handler;
    
    
    // get the SPI handler
    spi_handler = spi_get_handler(config->spi_module);
    
    // select channel
    write_data[0] = ADC_COMMAND_TO_BITS(config->channel);
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    spi_handler(&m);

    // sample 
    CLEAR_BIT(config->conv_pin.lat_r, config->conv_pin.n);
    delay_us(10);
    SET_BIT(config->conv_pin.lat_r, config->conv_pin.n);
    delay_us(10);

    // read sample
    write_data[0] = ADC_COMMAND_TO_BITS(ADC_READ_DATA);
    write_data[1] = 0x0000;
    spi_init_message(&m, write_data, read_data, 1, &config->cs_pin);
    spi_handler(&m);
    
    return m.read_data[0];
}

int16_t run_calibration(adc_config_t* config, 
        const uint16_t reference_value){
    
    uint16_t n;
    uint64_t sample_data;
    void (*send_spi_message) (spi_message_t* m);
    
    switch(config->spi_module){
        case SPI_MODULE_SELECTOR_1:
            send_spi_message = spi1_send_message;
            break;
        case SPI_MODULE_SELECTOR_2:
            send_spi_message = spi2_send_message;
            break;
        case SPI_MODULE_SELECTOR_3:
            send_spi_message = spi3_send_message;
            break;
        default:
            report_error("ADC: unsupported SPI module.");
            break;
    }
    
    sample_data = 0;

    // read all data
    for(n = 0; n < ADC_N_CALIBRATION_SAMPLES; n++){
        sample_data += read_adc_channel(config);
    }

    // average data
    sample_data /= ADC_N_CALIBRATION_SAMPLES;

    // check to reference value
    return sample_data - reference_value;
}

void run_max_var(adc_config_t* config,  uint16_t* const max_value, 
        uint16_t* const min_value, uint16_t* const mean){
    
    uint64_t sum_value;
    uint16_t n, sample_value;
    void (*send_spi_message) (spi_message_t* m);
    
    switch(config->spi_module){
        case SPI_MODULE_SELECTOR_1:
            send_spi_message = spi1_send_message;
            break;
        case SPI_MODULE_SELECTOR_2:
            send_spi_message = spi2_send_message;
            break;
        case SPI_MODULE_SELECTOR_3:
            send_spi_message = spi3_send_message;
            break;
        default:
            report_error("ADC: unsupported SPI module.");
            break;
    }
    
    // default values
    *max_value = 0;
    *min_value = 0;

    // read all data
    for(n = 0; n < ADC_N_CALIBRATION_SAMPLES; n++){
        sample_value = read_adc_channel(config);
        sum_value += sample_value;
        
        *max_value = MAX(*max_value, sample_value);
        *min_value = MIN(*min_value, sample_value);
    }

    // average data
    *mean = (uint16_t) (sum_value / ADC_N_CALIBRATION_SAMPLES);
    
    // values are passed by reference using the arguments
}
