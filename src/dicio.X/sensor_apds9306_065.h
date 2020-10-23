#ifndef __SENSOR_APDS9306_H__
#define	__SENSOR_APDS9306_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include <sensor_common.h>

#define I2C_ADDRESS_SENSOR_APDS9306     0x52
#define SENSOR_APDS3906_CAN_DATA_LENGTH 3

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum {
    S_APDS9306_R_MAIN_CTRL        = 0x00,
    S_APDS9306_R_ALS_MEAS_RATE    = 0x04,
    S_APDS9306_R_ALS_GAIN         = 0x05,
    S_APDS9306_R_PART_ID          = 0x06,
    S_APDS9306_R_MAIN_STATUS      = 0x07,
    S_APDS9306_R_CLEAR_DATA_0     = 0x0A,
    S_APDS9306_R_CLEAR_DATA_1     = 0x0B,
    S_APDS9306_R_CLEAR_DATA_2     = 0x0C,
    S_APDS9306_R_ALS_DATA_0       = 0x0D,
    S_APDS9306_R_ALS_DATA_1       = 0x0E,
    S_APDS9306_R_ALS_DATA_2       = 0x0F,
    S_APDS9306_R_INT_CFG          = 0x19,
    S_APDS9306_R_INT_PERSISTENCE  = 0x1A,
    S_APDS9306_R_ALS_THRES_UP_0   = 0x21,
    S_APDS9306_R_ALS_THRES_UP_1   = 0x22,
    S_APDS9306_R_ALS_THRES_UP_2   = 0x23,
    S_APDS9306_R_ALS_THRES_LOW_0  = 0x24,
    S_APDS9306_R_ALS_THRES_LOW_1  = 0x25,
    S_APDS9306_R_ALS_THRES_LOW_2  = 0x26,
    S_APDS9306_R_ALS_THRES_VAR    = 0x27,
} apds9306_register_t;

typedef enum {
    apds9306_als_meas_rate_25ms = 0b000,
    apds9306_als_meas_rate_50ms = 0b001,
    apds9306_als_meas_rate_100ms = 0b010,
    apds9306_als_meas_rate_200ms = 0b011,
    apds9306_als_meas_rate_500ms = 0b100,
    apds9306_als_meas_rate_1000ms = 0b101,
    apds9306_als_meas_rate_2000ms = 0b110,   
} apds9306_als_meas_rate_t;

typedef enum {
    apds9306_als_resolution_13bit = 0b101,
    apds9306_als_resolution_16bit = 0b100,
    apds9306_als_resolution_17bit = 0b011,
    apds9306_als_resolution_18bit = 0b010,
    apds9306_als_resolution_19bit = 0b001,
    apds9306_als_resolution_20bit = 0b000,
} apds9306_als_resolution_t;

typedef enum {
    apds9306_als_gain_1 = 0b000,
    apds9306_als_gain_3 = 0b001,
    apds9306_als_gain_6 = 0b010,
    apds9306_als_gain_9 = 0b011,
    apds9306_als_gain_18 = 0b100,
} apds9306_als_gain_t;

typedef enum {
    apds9306_als_status_active,
    apds9306_als_status_standby,
} apds9306_als_status_t;

typedef enum {
    apds9306_als_iv_8_counts = 0b000,
    apds9306_als_iv_16_counts = 0b001,    
    apds9306_als_iv_32_counts = 0b010,  
    apds9306_als_iv_64_counts = 0b011,  
    apds9306_als_iv_128_counts = 0b100,  
    apds9306_als_iv_256_counts = 0b101,  
    apds9306_als_iv_512_counts = 0b110,  
    apds9306_als_iv_1024_counts = 0b111,  
} apds9306_als_interrupt_variance_t;

typedef struct sensor_apds9306_config_s {
    sensor_general_config_t general;
    
    apds9306_als_meas_rate_t meas_rate;
    apds9306_als_resolution_t meas_resolution;
    apds9306_als_gain_t gain;
    
    uint32_t apds9306_als_threshold_high;
    uint32_t apds9306_als_threshold_low;

    i2c_message_t m_read_setup;
    i2c_message_t m_read;
    
    uint8_t m_read_address[1];
    uint8_t m_read_data[3];
} sensor_apds9306_config_t;

/**
 * @brief Initialisation of APDS9306-065 sensor.
 * 
 * @details Resets the APDS9306-065 sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
i2c_error_t apds9306_init_sensor(sensor_apds9306_config_t* config);


/**
 * @brief Send APDS9306-065 I2C data over CAN bus.
 * 
 * @details Processes I2C data received by APDS9306-065 sensor and prepares CAN 
 * message for transmission.
 * 
 * @param m: I2C message containing data received from I2C exchange
 */
void apds9306_065_i2c_cb(i2c_message_t* m);

#ifdef	__cplusplus
}
#endif 

#endif

