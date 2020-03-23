#ifndef __SENSOR_PLANALTA_H__
#define	__SENSOR_PLANALTA_H__

#include <xc.h>
#include <dsp.h>
#include <utilities.h>
#include <i2c.h>
#include "sensor_common.h"
#include "dicio.h"
#include <planalta.h>
#include <filters_planalta.h>

/**
 * Write addresses of I2C sensors and modules
 */
#define I2C_ADDRESS_0_SENSOR_PLANALTA (PLANALTA_I2C_BASE_ADDRESS | 0b00)
#define I2C_ADDRESS_1_SENSOR_PLANALTA (PLANALTA_I2C_BASE_ADDRESS | 0b01)
#define I2C_ADDRESS_2_SENSOR_PLANALTA (PLANALTA_I2C_BASE_ADDRESS | 0b10)
#define I2C_ADDRESS_3_SENSOR_PLANALTA (PLANALTA_I2C_BASE_ADDRESS | 0b11)
#define SENSOR_PLANALTA_CAN_DATA_LENGTH 8

/**
 * @brief Indicate which ports are input and output ports.
 * 
 * @details `0` indicates an output port and `1` indicates an input port.
 */


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct sensor_planalta_config_s sensor_planalta_config_t;
    
typedef struct sensor_planalta_channel_config_s {
    uint8_t local_id;
    
    i2c_message_t m_write, m_read;
    
    sensor_staus_t status;
    
    planalta_signal_frequency_t freq;
    pga_gain_t gain;
    
    i2c_message_t m_ch_config;
    uint8_t m_ch_config_data[2];
    
    uint8_t m_write_data[1];
    uint8_t m_read_data[PLANALTA_I2C_READ_BUFFER_LENGTH];
  
    uint8_t readout_data[UART_HEADER_SIZE + PLANALTA_I2C_READ_BUFFER_LENGTH];
    
    sensor_log_t dlog;
    sensor_planalta_config_t* sensor_config;
} sensor_planalta_channel_config_t;
    
struct sensor_planalta_config_s {
    sensor_general_config_t general;

    i2c_message_t m_status_config;
    uint8_t m_status_config_data[2];
    
    i2c_message_t m_adc_config;
    uint8_t m_adc_config_data[2];
    
    sensor_planalta_channel_config_t channels[PLANALTA_N_ADC_CHANNELS];

};


/**
 * @brief Initialisation of BH1721FVC sensor.
 * 
 * @details Resets the BH1721FVC sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
void planalta_sensor_init_channel(sensor_planalta_channel_config_t* config, sensor_planalta_config_t* sensor_config);
i2c_error_t planalta_sensor_init(sensor_planalta_config_t* config);

void planalta_sensor_meas_read_cb(i2c_message_t* m);
void planalta_sensor_read(sensor_planalta_config_t* config);

void planalta_sensor_i2c1_write_read_controller(i2c_message_t* m);
void planalta_sensor_i2c2_write_read_controller(i2c_message_t* m);

void planalta_sensor_start(sensor_planalta_config_t* config);


#ifdef	__cplusplus
}
#endif


#endif

