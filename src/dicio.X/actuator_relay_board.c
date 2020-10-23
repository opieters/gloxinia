#include "actuator_relay_board.h"
#include <utilities.h>
#include <address.h>

void actuator_relay_board_init(actuator_relay_board_t* config){
    i2c_message_t m;
    uint8_t data[2], i;
    
    void (*controller)(i2c_message_t* m);
    
    // configure general interface
    actuator_init_common_config(&config->general, ACTUATOR_RELAY_BOARD_DATA_LENGTH);
    
    config->general.status = ACTUATOR_STATE_ACTIVE;
    
    // configure board over I2C
    
    switch(config->i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("RELAY BOARD: I2C module not supported.");
            break;
    }
    
    // set output to zero
    i2c_init_message(&m, I2C_WRITE_ADDRESS(config->i2c_address), data, ARRAY_LENGTH(data),
            controller, 3, i2c_dummy_callback, NULL, NULL, 0, config->i2c_bus, NULL);
    
    data[0] = 3; // output register port 1
    data[1] = 0;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    if(m.error != I2C_NO_ERROR){
        uart_simple_print("actuator: A.");
    }
    
    delay_ms(100);
    
    // configure all IO pins as output
    i2c_init_message(&config->m_config, I2C_WRITE_ADDRESS(config->i2c_address), 
            config->config_data, ARRAY_LENGTH(config->config_data),
            controller, 3, actuator_relay_board_config_i2c_cb, NULL, (uint8_t*) config, 0, config->i2c_bus, NULL);
    i2c_init_message(&config->m_set_data, I2C_WRITE_ADDRESS(config->i2c_address), 
            config->set_data, ARRAY_LENGTH(config->set_data), 
            controller, 1, actuator_relay_board_set_data_i2c_cb, NULL, (uint8_t*) config, 0, config->i2c_bus, NULL);
    
    config->m_config.data[0] = 7; // configuration register port 1
    config->m_config.data[1] = 0;
    
    i2c_queue_message(&config->m_config);
    i2c_empty_queue();
    
    if(m.error != I2C_NO_ERROR){
        uart_simple_print("actuator: B.");
    }
    
    delay_ms(100);
    
    // configure polarity
    i2c_init_message(&m, I2C_WRITE_ADDRESS(config->i2c_address), data, ARRAY_LENGTH(data),
            controller, 3, i2c_dummy_callback, NULL, NULL, 0, config->i2c_bus, NULL);
    
    data[0] = 4; // polarity inversion register port 0
    data[1] = 0xff;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    if(m.error != I2C_NO_ERROR){
        uart_simple_print("actuator: switch on I2C not responding.");
    }
    
    delay_ms(100);
    
    /*for(i = 0; i < 8; i++){
        actuator_relay_board_update(config, 1 << i);
        actuator_relay_board_callback(config);
        i2c_empty_queue();

        delay_ms(500);
    }*/
    
    actuator_relay_board_update(config, 0x00);
    actuator_relay_board_callback(config);
    i2c_empty_queue();
}

void actuator_relay_board_update(actuator_relay_board_t* board, uint8_t new_output){ 
    board->new_output = new_output;
}

void actuator_relay_board_config_i2c_cb(i2c_message_t* m){
    actuator_relay_board_t* config = (actuator_relay_board_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        config->output = config->new_output;

        config->set_data[0] = 0x3;
        config->set_data[1] = config->output;
        
        i2c_reset_message(&config->m_set_data, 1);
        i2c_queue_message(&config->m_set_data);
    } else {
        // do nothing
    }
}

void actuator_relay_board_set_data_i2c_cb(i2c_message_t* m){
    actuator_relay_board_t* config = (actuator_relay_board_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        uint8_t log_data[2];
        log_data[0] = config->general.local_id;
        log_data[1] = config->output;
    
        // copies data internally, so local variable OK
        send_actuator_data(&config->general, log_data, ARRAY_LENGTH(log_data));
    } else {
        // do nothing
    }
}

uint8_t actuator_relay_board_read(actuator_relay_board_t* board){
    i2c_message_t m;
    i2c_message_t connected_m;
    uint8_t data[1];
    uint8_t connected_data[1];
    
    void (*controller)(i2c_message_t* m);
    
    switch(board->i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_read_controller;
            break;
        default:
            report_error("RELAY BOARD: I2C module not supported.");
            break;
    }
    
    i2c_init_message(&m, I2C_WRITE_ADDRESS(board->i2c_address), data, ARRAY_LENGTH(data),
            controller, 3, i2c_dummy_callback, NULL, NULL, 0, board->i2c_bus, NULL);
    
    i2c_init_connected_message(&connected_m, &m, connected_data, ARRAY_LENGTH(connected_data));
    
    data[0] = 0;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    return connected_data[0];
}

void actuator_relay_board_callback(actuator_relay_board_t* board){    
    i2c_reset_message(&board->m_config, 1);    
    i2c_queue_message(&board->m_config);
}
