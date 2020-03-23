
#ifndef __PLANALTA_H__
#define	__PLANALTA_H__

#include <xc.h> // include processor files - each processor file is guarded.  
#include <adc.h>
#include <pga.h>
#include <i2c.h>
#include "filters_planalta.h"
#include "dac.h"
#include <planalta_filtering.h>

#define PLANALTA_CHANNEL_BUFFER_SIZE 16
#define PLANALTA_N_CHANNELS 4
#define PLANALTA_N_ADC_CHANNELS 8
#define PLANALTA_I2C_BASE_ADDRESS  0b1011000

#define PLANALTA_ADC_SAMPLE_FREQUENCY     160000

// length in bytes of the I2C read buffer
#define PLANALTA_I2C_READ_BUFFER_LENGTH   (4*PLANALTA_N_ADC_CHANNELS+1)

// length in bytes of the I2C (master) write buffer, i.e. the max length a
// write message to this slave can be
#define PLANALTA_I2C_WRITE_BUFFER_LENGTH  (5)

#define PLANALTA_PRINT_VALUES_BUFFER_SIZE PLANALTA_BLOCK4_INPUT_SIZE // must be at least the size of PLANALTA_BLOCK5_INPUT_SIZE

#define PLANALTA_STATUS_ON           (0 << 7)
#define PLANALTA_STATUS_OFF          (1 << 7)
#define PLANALTA_STATUS_RESET_BUFFER (1 << 6)
#define PLANALTA_STATUS_RESET        (1 << 5)

#define PLANALTA_ADC_ON (1<<7)

#define PLANALTA_CH_CONFIG_ON (1<<7)
#define PLANALTA_CH_CONFIG_SET_GAIN(X) (((X)-PGA_GAIN_1) << 4)
#define PLANALTA_CH_CONFIG_GAIN (0b111 << 4)

#define PLANALTA_N_ADDRESS_SEL_PINS 8

#define PLANALTA_50KHZ_N_ADC_CHANNELS 1
#define PLANALTA_25KHZ_N_ADC_CHANNELS 2
#define PLANALTA_10KHZ_N_ADC_CHANNELS 4
#define PLANALTA_5KHZ_N_ADC_CHANNELS  8

typedef enum {
    PLANALTA_REG_STATUS = 0,
    PLANALTA_REG_ADC = 1,
    PLANALTA_REG_CONFIG_CH0 = 2,
    PLANALTA_REG_CONFIG_CH1 = 3,
    PLANALTA_REG_CONFIG_CH2 = 4,
    PLANALTA_REG_CONFIG_CH3 = 5,
    PLANALTA_REG_DATA_I0 = 6,
    PLANALTA_REG_DATA_O0 = 7,
    PLANALTA_REG_DATA_I1 = 8,
    PLANALTA_REG_DATA_O1 = 9,
    PLANALTA_REG_DATA_I2 = 10,
    PLANALTA_REG_DATA_O2 = 11,
    PLANALTA_REG_DATA_I3 = 12,    
    PLANALTA_REG_DATA_O3 = 13,
    PLANALTA_REG_CONFIG_T0 = 14,
    PLANALTA_REG_CONFIG_T1 = 15,
    PLANALTA_N_REG
} planalta_reg_t;

typedef enum {
    PLANALTA_FILTER_SEL_50KHZ,
    PLANALTA_FILTER_SEL_25KHZ,
    PLANALTA_FILTER_SEL_10KHZ,
    PLANALTA_FILTER_SEL_5KHZ,
} planalta_signal_frequency_t;

typedef enum {
    PLANALTA_CHANNEL_ENABLED,
    PLANALTA_CHANNEL_DISABLED,
} planalta_channel_status_t;

typedef struct planalta_config_s {
    adc_config_t adc_config;
    dac_config_t dac_config;
    pga_config_t pga_config[PLANALTA_N_CHANNELS];
    planalta_channel_status_t channel_status[PLANALTA_N_CHANNELS];
    i2c_config_t i2c_config;
    const pin_t address_selection[PLANALTA_N_ADDRESS_SEL_PINS];
    pin_t blinky_pin;
    planalta_signal_frequency_t signal_frequency;
    pin_t filter_selection_pins[2];
    pin_t int_pin;
} planalta_config_t;

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
    /*
     * Initialises all the relevant hardware that does not depend upon user
     * configuration. All user specific configuration options should be set 
     * after this function
     */
    void init_planalta(void);
    
    /*
     * Executes the main function of the application. This includes reading 
     * incoming I2C data that changes the configuration of the sensor, starts 
     * the operation (ADC sampling + filtering) and reads the generated data.
     * 
     */
    void loop_planalta(void);
    
    /*
     * Reads the user specified I2C address from the dip-switch on the board
     * and returns a valid I2C address.
     * 
     * When the I2C address that the user entered is not valid, a fallback 
     * address is used. This address is defined in `PLANALTA_I2C_BASE_ADDRESS`. 
     * Reading the address required correct initialisation of the corresponding
     * pins defined in `planalta_config_t.address_selection`. Only the least 
     * significant 7 bits are used. The most significant bit is reserved for
     * future and should be configured to 0.
     * 
     * The address selection uses inverted logic to make it easier to configure 
     * the address without using a voltage meter. Setting a bit in the ON-state
     * will turn the corresponding bit in the I2C-address to 1.
     * 
     * This function is called in `init_planalta`. 
     */
    uint8_t i2c_get_address_planalta(planalta_config_t* config);
    
    /*
     * Initialises all hardware pins on the board to the correct configuration.
     * This includes all the peripheral pins that are needed for various 
     * protocols (I2C, SPI, UART) and application specific functions (wave 
     * generation, ADC triggering...).
     * 
     * This function is called when appropriate by `init_planalta`. 
     */
    void init_pins_planalta(void);
    
    /*
     * I2C callback after completion of a master write (slave read) operation.
     * 
     * Note that this callback is executed after the execution of the message 
     * specific callback function.
     */
    void i2c_mw_sr_cb_planalta(i2c_message_t* m);
    
    /*
     * I2C callback after completion of a master read (slave write) operation.
     * 
     * Note that this callback is executed after the execution of the message 
     * specific callback function.
     */
    void i2c_mr_sw_cb_planalta(i2c_message_t* m);
    
    /*
     * Clears all sampling-related buffers. This includes the temporary storage
     * buffers, delay buffers and I2C message buffers.
     */
    void planalta_clear_buffers(void);
    
    
    void planalta_i2c_channel_config(uint8_t channel_n, uint8_t* data);
    
    void planalta_i2c_read_copy_buffer_data(uint8_t channel_n);
    
    
    void planalta_set_filters(planalta_signal_frequency_t config);
    
    void planalta_channel_config(uint8_t channel_n, 
        planalta_channel_status_t status, pga_gain_t gain);

#ifdef	__cplusplus
}
#endif

#endif

