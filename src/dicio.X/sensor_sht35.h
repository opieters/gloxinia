#ifndef __SENSOR_SHT35_H__
#define	__SENSOR_SHT35_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include "sensor_common.h"

/**
 * Number of sensors of this sensor type. Other sensors are assumed to be 
 * present only once.
 */


#define SENSOR_SHT35_CONFIG_DATA_LENGTH 2
#define SENSOR_SHT35_DATA_LENGTH 6
#define SENSOR_SHT35_CAN_DATA_LENGTH SENSOR_SHT35_DATA_LENGTH

#define SENSOR_SHT35_FETCH_DATA_LENGTH 2

#define SHT35_CRC_POLY 0x31

#define I2C_ADDRESS_0_SENSOR_SHT35 0x44
#define I2C_ADDRESS_1_SENSOR_SHT35 0x45

/**
 * CAN buffer identifier for each sensor and message type
 */
#define CAN_CHANNEL_SENSOR_SHT35      4

/**
 * ECAN module status bits for each sensor
 */
#define CAN_STATUS_SHT35      C1TR01CONbits.TXREQ0


    
#define SENSOR_SHT35_RESET_PIN 6


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
   
    
/**
 * Enum listing configuration options of SHT35 sensor.
 */
typedef enum {
    S_SHT35_PERIODICITY_NONE,
    S_SHT35_SINGLE_SHOT, 
    S_SHT35_PERIODIC, 
} sensor_sht35_periodicity_t;

typedef enum {
    S_SHT35_REPEATABILITY_NONE,
    S_SHT35_HIGH_REPEATABILIBTY, 
    S_SHT35_MEDIUM_REPEATABILITY, 
    S_SHT35_LOW_REPEATABILITY, 
} sensor_sht35_repeatability_t;

typedef enum {
    S_SHT35_CLOCK_NONE,
    S_SHT35_ENABLE_CLOCK_STRETCHING, 
    S_SHT35_DISABLE_CLOCK_STRETCHING,
} sensor_sht35_clock_stretching_t;

typedef enum {
    S_SHT35_MPS_NONE,
    S_SHT35_0_5_MPS,
    S_SHT35_1_MPS,
    S_SHT35_2_MPS,
    S_SHT35_4_MPS,
    S_SHT35_10_MPS
} sensor_sht35_sample_rate_t;

typedef struct sensor_sht35_config_s {
    sensor_general_config_t general;

    sensor_sht35_repeatability_t repeatability;
    sensor_sht35_clock_stretching_t clock;
    sensor_sht35_sample_rate_t rate;
    sensor_sht35_periodicity_t periodicity;
    
    i2c_message_t m_config;
    i2c_message_t m_read;
    i2c_message_t m_fetch;
    
    uint8_t m_config_data[SENSOR_SHT35_CONFIG_DATA_LENGTH];
    uint8_t m_read_data[SENSOR_SHT35_DATA_LENGTH];
    uint8_t m_fetch_data[SENSOR_SHT35_FETCH_DATA_LENGTH];
} sensor_sht35_config_t;

/**
 * @brief Initialisation of a SHT35 sensor.
 * 
 * @details Resets the SHT35 sensor and initialises it to operate in the
 * periodic data acquisition mode.
 * 
 * @return byte indicating status of the initialisation. If the bit 
 * corresponding to the sensor number is high, initialisation was successful.
 * Otherwise an error occurred.
 */
i2c_error_t sht35_init(sensor_sht35_config_t* config);

/**
 * Configure sensor 
 */
void sht35_config_sensor(sensor_sht35_config_t* config);
void sht35_config_single_shot(sensor_sht35_config_t* config);
void sht35_config_periodic(sensor_sht35_config_t* config);
void sht35_init_messages_single_shot(sensor_sht35_config_t* config);
void sht35_init_messages_periodic(sensor_sht35_config_t* config);


/**
 * @brief Hardware reset of SHT35 sensor.
 * 
 * @details Handles a full hardware reset of the SHT35 sensor. This is done by 
 * toggling a pin of the GPIO expander of the sensor board.
 * 
 */
void sht35_toggle_reset_pin(void);


/**
 * @brief Send SHT35 I2C data over CAN bus.
 * 
 * @details Processes I2C data received by SHT35 sensor and prepares CAN 
 * message for transmission.
 * 
 * @param m: I2C message containing data received from I2C exchange
 */
void sht35_i2c_cb_periodic_m_read(i2c_message_t* m);
void sht35_i2c_cb_periodic_m_fetch(i2c_message_t* m);
void sht35_i2c_cb_single_shot_m_config(i2c_message_t* m);
void sht35_i2c_cb_single_shot_m_read(i2c_message_t* m);

uint8_t sht35_calculate_crc(uint8_t b, uint8_t crc, uint8_t poly);

void sht35_init_read_cb(void (*cb) (void));

#ifdef	__cplusplus
}
#endif


#endif

