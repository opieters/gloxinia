#ifndef __ACTUATOR_RELAY_BOARD__
#define	__ACTUATOR_RELAY_BOARD__

#include <xc.h>
#include <i2c.h>
#include "actuator_common.h"

#define ACTUATOR_RELAY_BOARD_MAX_N_BOARDS 4
#define ACTUATOR_RELAY_BOARD_I2C_ADDRESS_0 0b0100000
#define ACTUATOR_RELAY_BOARD_I2C_ADDRESS_1 0b0100001
#define ACTUATOR_RELAY_BOARD_I2C_ADDRESS_2 0b0100010
#define ACTUATOR_RELAY_BOARD_I2C_ADDRESS_3 0b0100011
#define ACTUATOR_RELAY_BOARD_DATA_LENGTH 1

typedef struct {
    actuator_general_config_t general;
    
    uint8_t i2c_address;
    i2c_bus_t i2c_bus;
    
    i2c_message_t m_set_data;
    uint8_t set_data[2];
    
    i2c_message_t m_config;
    uint8_t config_data[2];
    
    uint8_t new_output;
    uint8_t output;
} actuator_relay_board_t;


#ifdef	__cplusplus
extern "C" {
#endif

    void actuator_relay_board_init(actuator_relay_board_t* board);
    
    void actuator_relay_board_update(actuator_relay_board_t* board, uint8_t output);
    
    uint8_t actuator_relay_board_read(actuator_relay_board_t* board);
    
    void actuator_relay_board_config_i2c_cb(i2c_message_t* m);
    void actuator_relay_board_set_data_i2c_cb(i2c_message_t* m);
    
    void actuator_relay_board_callback(actuator_relay_board_t* board);

#ifdef	__cplusplus
}
#endif

#endif

