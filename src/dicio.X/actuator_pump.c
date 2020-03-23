#include "actuator_pump.h"
#include <address.h>

void init_actuator_pump(pump_config_t* config){
    // configure pump pin
    CLEAR_BIT(config->pin.tris_r, config->pin.n); // configure as output
    CLEAR_BIT(config->pin.port_r, config->pin.n); // set pin low
    
    reset_actuator_local_id();
    config->general.local_id = get_actuator_local_id();
    config->general.global_id = CAN_DATA_CMD_PUMP;
    config->general.status = ACTUATOR_STATE_ACTIVE;
    
    config->status = PUMP_STATUS_OFF;
    config->timer_value = config->period - config->on_time;
    
    can_init_message(&config->general.dlog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_DATA_CMD_PUMP, config->general.local_id),
            &config->general.tx_data[UART_HEADER_SIZE-1],
            ACTUATOR_PUMP_DATA_LENGTH);
    
    // init message UART message
    uart_init_message(&config->general.dlog.uart_message, 
            SERIAL_SENSOR_DATA_CMD,
            controller_address,
            CAN_HEADER(CAN_DATA_CMD_PUMP, config->general.local_id),
            config->general.tx_data, 
            ACTUATOR_PUMP_DATA_LENGTH);
    
    can_init_message(&config->general.elog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_DATA_CMD_PUMP, config->general.local_id),
            config->general.error_data,
            ERROR_DATA_SIZE);
    
    // init message UART message
    uart_init_message(&config->general.elog.uart_message, 
            SERIAL_SENSOR_STATUS_CMD,
            controller_address,
            CAN_HEADER(CAN_DATA_CMD_PUMP, config->general.local_id),            
            config->general.error_data, 
            ERROR_DATA_SIZE);
    
    config->general.status = ACTUATOR_STATE_ACTIVE;
}

void start_pump(pump_config_t* config){
    SET_BIT(config->pin.port_r, config->pin.n); // set pin high
}

void stop_pump(pump_config_t* config){
    CLEAR_BIT(config->pin.port_r, config->pin.n); // set pin low
}