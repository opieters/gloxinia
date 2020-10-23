#ifndef __SENSOR_SYLVATICA_H__
#define	__SENSOR_SYLVATICA_H__

#include <xc.h>
#include <dsp.h>
#include <utilities.h>
#include <i2c.h>
#include "sensor_common.h"
#include "dicio.h"
#include <sylvatica.h>

/**
 * Write addresses of I2C sensors and modules
 */
#define I2C_ADDRESS_0_SENSOR_SYLVATICA (SYLVATICA_I2C_BASE_ADDRESS | 0b00)
#define I2C_ADDRESS_1_SENSOR_SYLVATICA (SYLVATICA_I2C_BASE_ADDRESS | 0b01)
#define I2C_ADDRESS_2_SENSOR_SYLVATICA (SYLVATICA_I2C_BASE_ADDRESS | 0b10)
#define I2C_ADDRESS_3_SENSOR_SYLVATICA (SYLVATICA_I2C_BASE_ADDRESS | 0b11)
#define SENSOR_SYLVATICA_CAN_DATA_LENGTH 8

/**
 * @brief Indicate which ports are input and output ports.
 * 
 * @details `0` indicates an output port and `1` indicates an input port.
 */


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct sensor_sylvatica_config_s sensor_sylvatica_config_t;
    
typedef struct sensor_sylvatica_channel_config_s {
    uint8_t local_id;
    
    i2c_message_t m_write, m_read;
    
    sensor_staus_t status;
    
    pga_config_t gain;
    
    i2c_message_t m_ch_config;
    uint8_t m_ch_config_data[2];
    
    uint8_t m_write_data[1];
    uint8_t* m_read_data;
  
    uint8_t readout_data[SYLVATICA_I2C_READ_BUFFER_LENGTH];
    
    sensor_log_t dlog;
    sensor_sylvatica_config_t* sensor_config;
} sensor_sylvatica_channel_config_t;

typedef struct sensor_sylvatica_raw_channel_config_s {
    uint8_t local_id;
    
    i2c_message_t m_write, m_read;
    
    sensor_staus_t status;
    
    uint8_t m_write_data[1];
    uint8_t* m_read_data;
  
    uint8_t readout_data[SYLVATICA_I2C_READ_BUFFER_LENGTH];
    
    sensor_log_t dlog;
    sensor_sylvatica_config_t* sensor_config;
} sensor_sylvatica_raw_channel_config_t;


    
struct sensor_sylvatica_config_s {
    sensor_general_config_t general;

    i2c_message_t m_status_config;
    uint8_t m_status_config_data[2];
    
    i2c_message_t m_adc_config;
    uint8_t m_adc_config_data[2];
    
    sensor_sylvatica_channel_config_t channels[SYLVATICA_N_CHANNELS];
    sensor_sylvatica_raw_channel_config_t raw_channels[SYLVATICA_N_CHANNELS];
    
    uint16_t sample_time;
};


/**
 * @brief Initialisation of BH1721FVC sensor.
 * 
 * @details Resets the BH1721FVC sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
void sylvatica_sensor_init_channel(sensor_sylvatica_channel_config_t* config, 
        sensor_sylvatica_config_t* sensor_config,
        const uint8_t channel_n);
void sylvatica_sensor_init_raw_channel(sensor_sylvatica_raw_channel_config_t* config, 
        sensor_sylvatica_config_t* sensor_config, 
        const uint8_t channel_n);
i2c_error_t sylvatica_sensor_init(sensor_sylvatica_config_t* config);

void sylvatica_measurement_read_cb(i2c_message_t* m);
void sylvatica_measurement_raw_read_cb(i2c_message_t* m);
void sylvatica_sensor_read(sensor_sylvatica_config_t* config);

void sylvatica_i2c1_write_read_controller(i2c_message_t* m);
void sylvatica_i2c2_write_read_controller(i2c_message_t* m);

void sylvatica_sensor_start(sensor_sylvatica_config_t* config);

void sylvatica_read_channel_config(sensor_sylvatica_channel_config_t* config, 
        sensor_sylvatica_config_t* sensor_config,
        const uint8_t channel_n);

#ifdef	__cplusplus
}
#endif


#endif

