#include "actuator_common.h"
#include <can.h>
#include "address.h"

static uint8_t local_id = 0;

void update_actuator_config_status(actuator_general_config_t* config, const i2c_error_t i2c_error){
    if(i2c_error != I2C_NO_ERROR){
        config->status = ACTUATOR_STATE_ERROR;
    } else {
        config->status = ACTUATOR_STATE_IDLE;
    }
}

void send_actuator_error(actuator_general_config_t* config, i2c_message_t* m){
    if(controller_address == 0) {
        while(config->elog.uart_message.status != UART_MSG_TRANSFERRED);
        config->elog.uart_message.data[2] = m->status;
        config->elog.uart_message.data[3] = m->error;
        uart_reset_message(&config->elog.uart_message);
        uart_queue_message(&config->elog.uart_message);
    } else {
        config->elog.can_message.data[2] = m->status;
        config->elog.can_message.data[3] = m->error;
        can_send_message_any_ch(&config->elog.can_message);
    }
}

void send_actuator_data(actuator_general_config_t* config, uint8_t* data, uint8_t length){
    uint16_t i, j;
    
    if(controller_address == 0){
        uart_message_t* m = &config->dlog.uart_message;
        
        for(i = 0; i < length; i++){
            m->data[i] = data[i];
        }
        m->length = length;
        
        uart_reset_message(m);
        uart_queue_message(m);
    } else {
        can_message_t* m = &config->dlog.can_message;
        
        for(j = 0; j < length; j+= CAN_MAX_N_BYTES){
            m->data_length = MAX(CAN_MAX_N_BYTES, length - j);
            
            for(i = j; i <  j+m->data_length; i++){
                m->data[i] = data[i];
            }

            can_send_message_any_ch(m);
        }
    }   
}

void send_actuator_data_no_copy(actuator_general_config_t* config){
    if(controller_address == 0){
        uart_reset_message(&config->dlog.uart_message);        
        uart_queue_message(&config->dlog.uart_message);
    } else {
        can_send_message_any_ch(&config->dlog.can_message);
    }   
}

uint8_t get_actuator_local_id(void){
    return local_id++;
}

void reset_actuator_local_id(void){
    local_id = 0;
}

void actuator_init_common_config(actuator_general_config_t* general, uint8_t length) {
    can_init_message(&general->dlog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(general->global_id, general->local_id),
            general->tx_data,
            length);
    
    uart_init_message(&general->dlog.uart_message, 
            SERIAL_ACTUATOR_DATA_CMD,
            controller_address,
            CAN_HEADER(general->global_id, general->local_id),
            general->tx_data, 
            length);
    
    can_init_message(&general->elog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_MSG_ACTUATOR_ERROR, 0),
            general->error_data,
            ERROR_DATA_SIZE);
    
    uart_init_message(&general->elog.uart_message, 
            SERIAL_ACTUATOR_ERROR_CMD,
            controller_address,
            CAN_HEADER(CAN_MSG_ACTUATOR_ERROR, 0),            
            general->error_data, 
            ERROR_DATA_SIZE);
    
    general->error_data[0] = general->global_id;
    general->error_data[1] = general->local_id;
}