#ifndef __SENSOR_OPT3001Q1_H__
#define	__SENSOR_OPT3001Q1_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include "sensor_common.h"

#define I2C_ADDRESS_0_SENSOR_OPT3001Q1     0b1000100
#define I2C_ADDRESS_1_SENSOR_OPT3001Q1     0b1000101
#define I2C_ADDRESS_2_SENSOR_OPT3001Q1     0b1000110
#define I2C_ADDRESS_3_SENSOR_OPT3001Q1     0b1000111

#define SENSOR_OPT3001Q1_DATA_LENGTH       2
#define SENSOR_OPT3001Q1_CAN_DATA_LENGTH   SENSOR_OPT3001Q1_DATA_LENGTH
#define SENSOR_OPT3001Q1_CONTROL_LENGTH    3
#define SENSOR_OPT3001Q1_READ_SETUP_LENGTH 1

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
   
typedef struct sensor_opt3001q1_config_s {
    sensor_general_config_t general;

    i2c_message_t m_control;
    i2c_message_t m_read_setup;
    i2c_message_t m_read;
    
    uint8_t m_control_data[SENSOR_OPT3001Q1_CONTROL_LENGTH];
    uint8_t m_read_setup_data[SENSOR_OPT3001Q1_READ_SETUP_LENGTH];
    uint8_t m_read_data[SENSOR_OPT3001Q1_DATA_LENGTH];
} sensor_opt3001q1_config_t;


/**
 * @brief Initialisation of OPT3001-Q1 sensor.
 * 
 * @details Resets the OPT3001-Q1 sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
i2c_error_t opt3001q1_init_sensor(sensor_opt3001q1_config_t* config);

/**
 * Configure sensor 
 */
void opt3001q1_config_sensor(sensor_opt3001q1_config_t* config);

/**
 * @brief Send OPT3001-Q1 I2C data over CAN bus.
 * 
 * @details Processes I2C data received by OPT3001-Q1 sensor and prepares CAN 
 * message for transmission.
 * 
 * @param m: I2C message containing data received from I2C exchange
 */
void opt3001q1_i2c_cb_read(i2c_message_t* m);


/**
 * @brief Configure sensor for reading.
 * 
 * @detials Configure all OPT3001-Q1 sensor for reading when a sample is 
 * triggered. If the sensor is working correctly and not deactivated by the 
 * user, this method will queue the I2C data transfer message needed to obtain
 * the sensor data.
 * 
 * @attention: This method should only be called once and cannot be undone. 
 */
void opt3001q1_read_sensor(void);


//void i2c_processor_opt3001q1(i2c_message_t* m);
void opt3001q1_i2c_cb_control(i2c_message_t* m);

#ifdef	__cplusplus
}
#endif


#endif

