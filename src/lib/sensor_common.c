#include "sensor_common.h"
#include <can.h>
#include "address.h"

static uint8_t local_id = 0;

void sensor_init_common_config(sensor_general_config_t* general, uint8_t length) {
    can_init_message(&general->dlog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(general->global_id, general->local_id),
            general->tx_data,
            length);
    
    uart_init_message(&general->dlog.uart_message, 
            SERIAL_SENSOR_DATA_CMD,
            controller_address,
            CAN_HEADER(general->global_id, general->local_id),
            general->tx_data, 
            length);
    
    can_init_message(&general->elog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_MSG_SENSOR_ERROR, 0),
            general->error_data,
            ERROR_DATA_SIZE);
    
    uart_init_message(&general->elog.uart_message, 
            SERIAL_SENSOR_ERROR_CMD,
            controller_address,
            CAN_HEADER(CAN_MSG_SENSOR_ERROR, 0),            
            general->error_data, 
            ERROR_DATA_SIZE);
    
    general->error_data[0] = general->global_id;
    general->error_data[1] = general->local_id;
}

void sensor_update_status(sensor_general_config_t* config, const i2c_error_t i2c_error){
    if(i2c_error != I2C_NO_ERROR){
        config->status = SENSOR_STATUS_ERROR;
    } else {
        if(config->status != SENSOR_STATUS_ERROR){
            config->status = SENSOR_STATUS_ACTIVE;
        }
    }
}

void sensor_send_error(sensor_elog_t* elog, i2c_message_t* m){
    if(controller_address == 0) {
        if(elog->uart_message.status == UART_MSG_SENT){
            elog->uart_message.data[2] = m->status;
            elog->uart_message.data[3] = m->error;
            uart_reset_message(&elog->uart_message);
            uart_queue_message(&elog->uart_message);
        }
    } else {
        elog->can_message.data[2] = m->status;
        elog->can_message.data[3] = m->error;
        can_send_message_any_ch(&elog->can_message);
    }
}

void sensor_send_data(sensor_log_t* dlog, uint8_t* data, uint8_t length){
    uint16_t i, j;
    
    if(controller_address == 0){
        uart_message_t* m = &dlog->uart_message;
        
        length = MIN(length, m->length);
        for(i = 0; i < length; i++){
            m->data[i] = data[i];
        }
        m->length = length;
        
        uart_reset_message(m);
        uart_queue_message(m);
    } else {
        can_message_t* m = &dlog->can_message;
        
        for(j = 0; j < length; j+= CAN_MAX_N_BYTES){
            m->data_length = MAX(8, length - j);
            
            for(i = j; i <  j+m->data_length; i++){
                m->data[i] = data[i];
            }

            can_send_message_any_ch(m);
        }
    }   
}

void sensor_send_data_no_copy(sensor_log_t* dlog, uint8_t* data, uint8_t length){
    uint16_t j;
    
    if(controller_address == 0){
        uart_message_t* m = &dlog->uart_message;
        m->length = length;
        m->data = data;
        
        uart_reset_message(m);
        uart_queue_message(m);
    } else {
        can_message_t* m = &dlog->can_message;
        
        for(j = 0; j < length; j+= CAN_MAX_N_BYTES){
            m->data_length = MAX(CAN_MAX_N_BYTES, length - j);
            m->data = &data[j];

            can_send_message_any_ch(m);
        }
    }   
}

uint8_t sensor_get_local_id(void){
    return local_id++;
}

void sensor_reset_local_id(void){
    local_id = 0;
}