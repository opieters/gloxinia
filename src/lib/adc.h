/* 
 * File: adc.h  
 * Author: opieters
 * Comments:
 * Revision history: 0.0.1
 */
 
#ifndef __ADC_H__
#define	__ADC_H__

#include <xc.h>
#include <utilities.h>
#include <spi.h>

#define ADC_BUFFER_LENGTH         160
#define ADC_COMMAND_TO_BITS(CMD)  (((uint16_t) (CMD & 0xf)) << 12)
#define ADC_N_CALIBRATION_SAMPLES 64

/**
 * @brief Number of output samples produced at once.
 */
#define N_ADC_CHANNELS 8

typedef enum {
    ADC_SELECT_CH0   = 0x0,
    ADC_SELECT_CH1   = 0x1,
    ADC_SELECT_CH2   = 0x2,
    ADC_SELECT_CH3   = 0x3,
    ADC_SELECT_CH4   = 0x4,
    ADC_SELECT_CH5   = 0x5,
    ADC_SELECT_CH6   = 0x6,
    ADC_SELECT_CH7   = 0x7,
    ADC_WAKE_UP      = 0xB,
    ADC_READ_CFR     = 0xC,
    ADC_READ_DATA    = 0xD,
    ADC_WRITE_CFR    = 0xE,
    ADC_DEFAULT_MODE = 0xF,
} adc_command_t;

typedef enum {
    ADC_CH0 = 0,
    ADC_CH1 = 1,
    ADC_CH2 = 2,
    ADC_CH3 = 3,
    ADC_CH4 = 4,
    ADC_CH5 = 5,
    ADC_CH6 = 6,
    ADC_CH7 = 7,
} adc_channel_t;

typedef enum {
    ADC_CHANNEL_SELECT_MODE_MANUAL = 0,
    ADC_CHANNEL_SELECT_MODE_AUTO = 1,
} adc_channel_select_mode_t;

typedef enum {
    ADC_CONVERSION_CLOCK_SOURCE_SCLK = 0,
    ADC_CONVERSION_CLOCK_SOURCE_INTERNAL = 1
} adc_conversion_clock_source_t;

typedef enum {
    ADC_TRIGGER_SELECT_AUTO = 0,
    ADC_TRIGGER_SELECT_MANUAL = 1
} adc_trigger_select_t;

typedef enum {
    ADC_SAMPLE_RATE_AUTO_TRIGGER_500KSPS = 0,
    ADC_SAMPLE_RATE_AUTO_TRIGGER_250KSPS = 1
} adc_sample_rate_auto_trigger_t;

typedef enum {
    ADC_PIN10_POLARITY_SELECT_ACTIVE_HIGH = 0,
    ADC_PIN10_POLARITY_SELECT_ACTIVE_LOW  = 1,
} adc_pin10_polarity_select_t;

typedef enum {
    ADC_PIN10_OUTPUT_INT = 0,
    ADC_PIN10_OUTPUT_EOC  = 1,
} adc_pin10_output_t;

typedef enum {
    ADC_PIN10_IO_SELECT_CDI = 0,
    ADC_PIN10_IO_SELECT_EOC_INT = 1,
} adc_pin10_io_select_t;

typedef enum {
    ADC_AUTO_NAP_POWERDOWN_ENABLE  = 0,
    ADC_AUTO_NAP_POWERDOWN_DISABLE = 1,
} adc_auto_nap_powerdown_t;

typedef enum {
    ADC_NAP_POWERDOWN_ENABLE  = 0,
    ADC_NAP_POWERDOWN_DISABLE = 1,
} adc_nap_powerdown_t;

typedef enum {
    ADC_DEEP_POWERDOWN_ENABLE  = 0,
    ADC_DEEP_POWERDOWN_DISABLE = 1,
} adc_deep_powerdown_t;

typedef enum {
    ADC_TAG_OUTPUT_DISABLE = 0,
    ADC_TAG_OUTPUT_ENABLE  = 1,
} adc_tag_output_t;

typedef enum {
    ADC_SOFTWARE_RESET  = 0,
    ADC_NORMAL_OPERATION = 1,
} adc_software_reset_t;

typedef enum {
    ADC_STATUS_ON,
    ADC_STATUS_IDLE,
    ADC_STATUS_OFF,
} adc_status_t;

typedef struct {
    adc_channel_select_mode_t channel_select;
    adc_conversion_clock_source_t conversion_clock_source;
    adc_trigger_select_t trigger_select;
    adc_sample_rate_auto_trigger_t auto_trigger_rate;
    adc_pin10_polarity_select_t pin10_polarity;
    adc_pin10_output_t pin10_output;
    adc_pin10_io_select_t pin10_io;
    adc_auto_nap_powerdown_t auto_nap;
    adc_nap_powerdown_t nap_powerdown;
    adc_deep_powerdown_t deep_powerdown;
    adc_tag_output_t tag_output;
    adc_software_reset_t sw_reset;
    adc_channel_t channel;
    adc_status_t status;
    uint32_t sample_frequency;
    spi_module_selector_t spi_module;
    const pin_t rst_pin;
    const pin_t cs_pin;
    const pin_t conv_pin;
    uint16_t channel_offset[N_ADC_CHANNELS];
    void (*rx_callback)(void);
} adc_config_t;


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
    extern unsigned int adc_tx_buffer[ADC_BUFFER_LENGTH] __attribute__((space(dma), eds));
    
    extern unsigned int adc_rx_buffer_a[ADC_BUFFER_LENGTH] __attribute__((space(dma), eds));
    extern unsigned int adc_rx_buffer_b[ADC_BUFFER_LENGTH] __attribute__((space(dma), eds));
    
    extern unsigned int adc_debug_buffer_a[ADC_BUFFER_LENGTH];
    extern unsigned int adc_debug_buffer_b[ADC_BUFFER_LENGTH];

    /**
     * @brief Initialisation function for ADC.
     * 
     * @detials Initialises the ADC to operate in 12-bit mode.
     * 
     * @attention This needs to be done BEFORE using the ADC and before initialising
     * the DMA channel.
     * 
     */
    void init_adc(adc_config_t* config);
    void update_adc(adc_config_t* config);
    void adc_start(adc_config_t* config);
    void adc_stop(adc_config_t* config);
    
    void rx_callback_dummy(void);
    
    void adc_spi1_init(void);
    void adc_spi2_init(void);
    void adc_spi3_init(void);
    
    uint16_t read_adc_channel(adc_config_t* config);
    
    int16_t run_calibration(adc_config_t* config,  const uint16_t ref);
    
    void run_max_var(adc_config_t* config,  uint16_t* const max_value, 
        uint16_t* const min_value, uint16_t* const mean);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __ADC_H__ */
