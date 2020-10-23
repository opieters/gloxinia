#ifndef __ACTUATOR_COMMON_H__
#define __ACTUATOR_COMMON_H__

#include <xc.h>
#include <can.h>
#include <uart.h>
#include <sensor_common.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
typedef struct actuator_log_s {
    can_message_t can_message;
    uart_message_t uart_message;
} actuator_log_t;

typedef struct actuator_elog_s {
    can_message_t can_message;
    uart_message_t uart_message;
    uint16_t n_errors;
} actuator_elog_t;

typedef enum {
    ACTUATOR_STATE_INACTIVE,
    ACTUATOR_STATE_IDLE,
    ACTUATOR_STATE_ACTIVE,
    ACTUATOR_STATE_ERROR,
    N_ACTUATOR_STATE,
} actuator_state_t;
    
typedef struct actuator_general_config_s {
    uint8_t local_id;
    uint8_t global_id;
        
    actuator_state_t status;
    
    actuator_log_t dlog;
    actuator_elog_t elog;

    uint8_t tx_data[CAN_MAX_N_BYTES];
    uint8_t error_data[ERROR_DATA_SIZE];
} actuator_general_config_t;


void update_actuator_config_status(actuator_general_config_t* config, const i2c_error_t i2c_error);
void send_actuator_error(actuator_general_config_t* config, i2c_message_t* m);
void send_actuator_data(actuator_general_config_t* config, uint8_t* data, uint8_t length);
void send_actuator_data_no_copy(actuator_general_config_t* config);
uint8_t get_actuator_local_id(void);
void reset_actuator_local_id(void);
void actuator_init_common_config(actuator_general_config_t* general, uint8_t length);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif