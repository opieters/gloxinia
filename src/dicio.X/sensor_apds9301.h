#ifndef __SENSOR_APDS9301_H__
#define	__SENSOR_APDS9301_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include <uart.h>
#include "sensor_common.h"
#include <can.h>

/**
 * 7-bit addresses of I2C sensors and modules
 */
#define I2C_ADDRESS_0_SENSOR_APDS_9301   0b0101001
#define I2C_ADDRESS_1_SENSOR_APDS_9301   0b1001001
#define I2C_ADDRESS_2_SENSOR_APDS_9301   0b0111001



/**
 * Number of sensors of this sensor type. Other sensors are assumed to be 
 * present only once.
 */
#define N_SENSOR_APDS9301   1

#define SENSOR_APDS3901_DATA_LENGTH 2
#define SENSOR_APDS3901_CAN_DATA_LENGTH (SENSOR_APDS3901_DATA_LENGTH*2)
#define SENSOR_APDS3901_DATA_CMD_LENGTH 1

/*
 * Variables containing the index associated to the sensor. This index is needed
 * by the data processor that sends the CAN message to identify the sensor that 
 * generated the data. 
 * 
 * The local_id should be used as identifier.
 */
#define PROCESSOR_DATA_LENGTH_APDS9301  1

/**
 * CAN buffer identifier for each sensor and message type
 */
#define CAN_CHANNEL_SENSOR_APDS_9301  3


/**
 * ECAN module status bits for each sensor
 */
#define CAN_STATUS_APDS_9301  C1TR01CONbits.TXREQ1


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define APDS9301_CONFIG_M0_DATA_LENGTH 1    
#define APDS9301_CONFIG_M1_DATA_LENGTH 3
#define APDS9301_CONFIG_M0C_DATA_LENGTH 2
#define APDS9301_CONFIG_PROCESSOR_DATA_LENGTH 1

typedef struct sensor_apds9301_config_s {
    sensor_general_config_t general;

    i2c_message_t m0;
    i2c_message_t m1;
    i2c_message_t m0_connected;
    i2c_message_t m1_connected;
    i2c_message_t conf_m0;
    i2c_message_t conf_m1;
    i2c_message_t conf_m01_connected;
    
    uint8_t m0_data[SENSOR_APDS3901_DATA_CMD_LENGTH];
    uint8_t m1_data[SENSOR_APDS3901_DATA_CMD_LENGTH];
    uint8_t m0c_data[SENSOR_APDS3901_DATA_LENGTH];
    uint8_t m1c_data[SENSOR_APDS3901_DATA_LENGTH];
    uint8_t conf_m0_data[APDS9301_CONFIG_M0_DATA_LENGTH];
    uint8_t conf_m0c_data[APDS9301_CONFIG_M0C_DATA_LENGTH];
    uint8_t conf_m1_data[APDS9301_CONFIG_M1_DATA_LENGTH];
    uint8_t conf_m01c_processor_data[APDS9301_CONFIG_PROCESSOR_DATA_LENGTH];
} sensor_apds9301_config_t;



/**
 * @brief Initialisation of a specific APDS-9301 sensor.
 * 
 * @details Resets the APDS-9301 sensor and initialises it.
 * 
 * @param sensor_id indicates which sensor to initialise. It should be smaller 
 * than N_SENSOR_APDS_9301.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
i2c_error_t apds9301_init_sensor(sensor_apds9301_config_t* config);

/**
 * @brief Send APDS-9301 I2C data over CAN bus.
 * 
 * @details Processes I2C data received by APDS-9301 sensor and prepares CAN 
 * message for transmission.
 * 
 * @param m: I2C message containing data received from I2C exchange
 */
void apds9301_i2c_cb_m0(i2c_message_t* m);
void apds9301_i2c_cb_m1(i2c_message_t* m);

#ifdef	__cplusplus
}
#endif


#endif

