#ifndef __SENSOR_BH1721FVC_H__
#define	__SENSOR_BH1721FVC_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include "sensor_common.h"



/**
 * Write addresses of I2C sensors and modules
 */
#define I2C_ADDRESS_SENSOR_BH1721FVC     0b0100011

#define SENSOR_BH1721FVC_DATA_LENGTH 2
#define SENSOR_BH1721FVC_CAN_DATA_LENGTH SENSOR_BH1721FVC_DATA_LENGTH

#define CAN_CHANNEL_SENSOR_BH1721     5
#define BH1721FVC_CONFIG_M01_DATA_LENGTH 1


/**
 * ECAN module status bits for each sensor
 */

#define CAN_STATUS_BH1721     C1TR23CONbits.TXREQ2


#define SENSOR_BH1721FVC_RESET_PIN 4

/**
 * @brief Indicate which ports are input and output ports.
 * 
 * @details `0` indicates an output port and `1` indicates an input port.
 */


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct sensor_bh1721fvc_config_s {
    sensor_general_config_t general;

    i2c_message_t m, mc0, mc1;
    
    uint8_t m_data[SENSOR_BH1721FVC_DATA_LENGTH];
    uint8_t mc0_data[BH1721FVC_CONFIG_M01_DATA_LENGTH];
    uint8_t mc1_data[BH1721FVC_CONFIG_M01_DATA_LENGTH];
} sensor_bh1721fvc_config_t;



/**
 * @brief Initialisation of BH1721FVC sensor.
 * 
 * @details Resets the BH1721FVC sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
void bh1721fvc_init_sensor(sensor_bh1721fvc_config_t* config);

/**
 * Configure sensor 
 */
void bh1721fvc_config_sensor(sensor_bh1721fvc_config_t* config);


i2c_mstatus_t __bh1721fvc_init_sensor(sensor_bh1721fvc_config_t* config);
void __bh1721fvc_i2c_init_sampling(sensor_bh1721fvc_config_t* config);


/**
 * @brief Send BH1721FVC I2C data over CAN bus.
 * 
 * @details Processes I2C data received by BH1721FVC sensor and prepares CAN 
 * message for transmission.
 * 
 * @param m: I2C message containing data received from I2C exchange
 */
void bh1721fvc_i2c_cb(i2c_message_t* m);


/**
 * @brief Add OPT3001-Q1 measurement to measurement queue.
 * 
 * @details If the sensor is working correctly and not deactivated by the 
 * user, this method will queue the I2C data transfer message needed to obtain
 * the sensor data.
 */
void bh1721fvc_read_sensor();

void bh1721fvc_i2c_cb_mc0(i2c_message_t* m);


#ifdef	__cplusplus
}
#endif


#endif

