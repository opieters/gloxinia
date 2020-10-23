#ifndef __SYLVATICA_H__
#define	__SYLVATICA_H__

#include <xc.h> 
#include <adc.h>
#include <pga.h>
#include <i2c.h>
#include <filters_sylvatica.h>

#define SYLVATICA_N_CHANNELS               8
#define SYLVATICA_CHANNEL_BUFFER_SIZE      8
#define SYLVATICA_I2C_BASE_ADDRESS         0b1000000

#define SYLVATICA_ADC_SAMPLE_FREQUENCY     80000
#define SYLVATICA_COPY_BUFFER_SIZE         (10*SYLVATICA_DEC_FACT_F0)
#define SYLVATICA_BLOCK1_INPUT_SIZE        (10*SYLVATICA_DEC_FACT_F1)
#define SYLVATICA_BLOCK2_INPUT_SIZE        (10*SYLVATICA_DEC_FACT_F2)
#define SYLVATICA_BLOCK3_INPUT_SIZE        (SYLVATICA_DEC_FACT_F3)
#define SYLVATICA_BLOCK0_OUTPUT_SIZE       (SYLVATICA_COPY_BUFFER_SIZE  / SYLVATICA_DEC_FACT_F0)
#define SYLVATICA_BLOCK1_OUTPUT_SIZE       (SYLVATICA_BLOCK1_INPUT_SIZE / SYLVATICA_DEC_FACT_F1)
#define SYLVATICA_BLOCK2_OUTPUT_SIZE       (SYLVATICA_BLOCK2_INPUT_SIZE / SYLVATICA_DEC_FACT_F2)
#define SYLVATICA_BLOCK3_OUTPUT_SIZE       (SYLVATICA_BLOCK3_INPUT_SIZE / SYLVATICA_DEC_FACT_F3)

#define SYLVATICA_I2C_READ_CH_BUFFER_LENGTH 3

// length in words/samples per channel of the buffer to save the output data into after filtering

// length in bytes of the I2C read buffer
#define SYLVATICA_I2C_READ_BUFFER_LENGTH   (2*4+1)

// length in bytes of the I2C (master) write buffer, i.e. the max length a
// write message to this slave can be
#define SYLVATICA_I2C_WRITE_BUFFER_LENGTH  (5)

#define SYLVATICA_STATUS_ON           (0 << 7)
#define SYLVATICA_STATUS_OFF          (1 << 7)
#define SYLVATICA_STATUS_RESET_BUFFER (1 << 6)
#define SYLVATICA_STATUS_RESET        (1 << 5)

#define SYLVATICA_ADC_ON (1<<7)

#define SYLVATICA_CH_CONFIG_ON (1<<7)
#define SYLVATICA_CH_CONFIG_SET_GAIN(X) (((X)-PGA_GAIN_1) << 3)
#define SYLVATICA_CH_CONFIG_GAIN (0b1111 << 3)

#define SYLVATICA_N_ADDRESS_SEL_PINS 8

typedef enum {
    SYLVATICA_REG_STATUS = 0,
    SYLVATICA_REG_ADC = 1,
    SYLVATICA_REG_CONFIG_CH0 = 2,
    SYLVATICA_REG_CONFIG_CH1 = 3,
    SYLVATICA_REG_CONFIG_CH2 = 4,
    SYLVATICA_REG_CONFIG_CH3 = 5,
    SYLVATICA_REG_CONFIG_CH4 = 6,
    SYLVATICA_REG_CONFIG_CH5 = 7,
    SYLVATICA_REG_CONFIG_CH6 = 8,
    SYLVATICA_REG_CONFIG_CH7 = 9,
    SYLVATICA_REG_DATA_CH0 = 10,
    SYLVATICA_REG_DATA_CH1 = 11,
    SYLVATICA_REG_DATA_CH2 = 12,
    SYLVATICA_REG_DATA_CH3 = 13,
    SYLVATICA_REG_DATA_CH4 = 14,
    SYLVATICA_REG_DATA_CH5 = 15,
    SYLVATICA_REG_DATA_CH6 = 16,
    SYLVATICA_REG_DATA_CH7 = 17,
    SYLVATICA_REG_RAW_CH0 = 18,
    SYLVATICA_REG_RAW_CH1 = 19,
    SYLVATICA_REG_RAW_CH2 = 20,
    SYLVATICA_REG_RAW_CH3 = 21,
    SYLVATICA_REG_RAW_CH4 = 22,
    SYLVATICA_REG_RAW_CH5 = 23,
    SYLVATICA_REG_RAW_CH6 = 24,
    SYLVATICA_REG_RAW_CH7 = 25,
    SYLVATICA_N_REG
} sylvatica_reg_t;

typedef enum {
    SYLVATICA_CHANNEL_ENABLED,
    SYLVATICA_CHANNEL_DISABLED,
} sylvatica_channel_status_t;

typedef enum {
    SYLVATICA_STATUS_INIT,
    SYLVATICA_STATUS_READY,
    SYLVATICA_STATUS_RUNNING,        
} sylvatica_status_t;

typedef struct {
    adc_config_t adc_config;
    pga_config_t pga_config[SYLVATICA_N_CHANNELS];
    sylvatica_channel_status_t channel_status[SYLVATICA_N_CHANNELS];
    i2c_config_t i2c_config;
    const pin_t address_selection[SYLVATICA_N_ADDRESS_SEL_PINS];
    pin_t blinky_pin;
    pin_t int_pin;
} sylvatica_config_t;

#ifdef	__cplusplus
extern "C" {
#endif
    
    void init_sylvatica(void);
    
    void loop_sylvatica(void);
    
    void update_sylvatica_channel(uint8_t channel_n, sylvatica_channel_status_t status, sylvatica_config_t* config);

    uint8_t i2c_get_address_sylvatica(sylvatica_config_t* config);
    
    void init_pins_sylvatica(void);
    
    /*
     * I2C communication protocol to read measurement data and configure the 
     * sensor.
     * 
     * To write a certain register use the following procedure:
     * | address + W | register | byte 1 | ... | byte n |
     * 
     * To read a certain register 2 procedures can be used.
     * 1. | address + W | register | address + R | n bytes | byte 1 | ... | byte n |
     * 2. | address + R | n bytes | byte 1 | ... | byte n | 
     * 
     * The following convention is always used: the last register that received 
     * a write will be read.
     * 
     * @attention
     * The write operation initiates a copy of this specific register to the
     * read buffer after the write completed. Subsequent reads to the same
     * register are not possible. A second read operation will result into all
     * zero replies.
     * 
     * 
     * The following registers are defined: 
     *  - 0: status register
     *  - 1: adc register
     *  - 2: channel 0 config
     *  - 3: channel 1 config
     *  - 4: channel 2 config
     *  - 5: channel 3 config
     *  - 6: channel 4 config
     *  - 7: channel 5 config
     *  - 8: channel 6 config
     *  - 9: channel 7 config
     *  - 10: channel 0 data (read only)
     *  - 11: channel 1 data (read only)
     *  - 12: channel 2 data (read only)
     *  - 13: channel 3 data (read only)
     *  - 14: channel 4 data (read only)
     *  - 15: channel 5 data (read only)
     *  - 16: channel 6 data (read only)
     *  - 17: channel 7 data (read only)
     * 
     * Register 0
     * 
     * | power down (low-power mode) | reset buffers | unimplemented |
     * 
     * Register 1
     * 
     * | adc on (starts measurement) | unimplemented |
     * 
     * If the ADC is runnig, then the channels cannot be reconfigured. The ADC
     * must first be turned off to change the channel configuration.
     * 
     * Register 2 - 9
     * 
     * | power bit ch x | gain setting ch x |  unimplemented |
     * 
     * Each gain setting consists of 3 bits: to select a gain from 1 (0) to 
     * 200 (7). Values: 1, 2, 5, 10, 20, 50, 100, 200
     */
    void i2c_mw_sr_cb_sylvatica(i2c_message_t* m);
    void i2c_mr_sw_cb_sylvatica(i2c_message_t* m);
    
    void adc_rx_callback(void);

    void process_filter_block0(void);
    void process_filter_block1(void);
    void process_filter_block2(void);
    void process_filter_block3(void);

    void print_values_uart();
    
    void init_uart_messages(void);
    
    void sylvatica_clear_buffers(void);
    
    void sylvatica_i2c_read_copy_channel_data(const uint8_t channel_n);
    void sylvatica_i2c_read_copy_raw_data(const uint8_t channel_n);
    
    void sylvatica_i2c_channel_config(const uint8_t channel_n, uint8_t* data);
    
    void sylvatica_i2c_read_config(const sylvatica_reg_t reg);
    
#ifdef	__cplusplus
}
#endif 

#endif

