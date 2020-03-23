#ifndef __SENSOR_APDS9306_H__
#define	__SENSOR_APDS9306_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include <sensor_common.h>

#define I2C_ADDRESS_SENSOR_APDS9306     0x52
#define SENSOR_APDS3906_CAN_DATA_LENGTH 2

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
    apds9306_als_meas_rate_25ms,
    apds9306_als_meas_rate_50ms,
    apds9306_als_meas_rate_100ms,
    apds9306_als_meas_rate_200ms,
    apds9306_als_meas_rate_500ms,
    apds9306_als_meas_rate_1000ms,
    apds9306_als_meas_rate_2000ms,   
} apds9306_als_meas_rate_t;

typedef enum {
    apds9306_als_resolution_13bit,
    apds9306_als_resolution_16bit,
    apds9306_als_resolution_17bit,
    apds9306_als_resolution_18bit,
    apds9306_als_resolution_19bit,
    apds9306_als_resolution_20bit,
} apds9306_als_resolution_t;

typedef enum {
    apds9306_als_gain_1,
    apds9306_als_gain_3,
    apds9306_als_gain_6,
    apds9306_als_gain_9,
    apds9306_als_gain_18,
} apds9306_als_gain_t;

typedef struct sensor_apds9306_config_s {
    sensor_general_config_t general;
    
    apds9306_als_meas_rate_t meas_rate;
    apds9306_als_resolution_t meas_resolution;
    apds9306_als_gain_t gain;

    i2c_message_t m_control;
    i2c_message_t m_read_setup;
    i2c_message_t m_read;
} sensor_apds9306_config_t;

/**
 * @brief Initialisation of OPT3001-Q1 sensor.
 * 
 * @details Resets the OPT3001-Q1 sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
void apds9306_init_sensor(sensor_apds9306_config_t* config);

#ifdef	__cplusplus
}
#endif 

#endif

