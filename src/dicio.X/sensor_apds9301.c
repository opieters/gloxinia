#include "sensor_apds9301.h"
#include <gpio_expander.h>
#include <stddef.h>
#include <can.h>
#include "address.h"
#include <i2c.h>
#include <uart.h>
#include <device_configuration.h>
#include <utilities.h>
#include <adc.h>
#include <spi.h>


void i2c_processor_config_m0_apds9301(i2c_message_t* m){
    sensor_apds9301_config_t* config = (sensor_apds9301_config_t*) m->processor_data;
    
    uint8_t state = m->connected_message->processor_data[0];

    // copy the first byte, since we are updating the register after the read
    // operation performed using m0.
    config->conf_m1.data[0] = config->conf_m0.data[0];
    
    // update configuration
    switch(state){
        case 0:
            config->conf_m1.data[1] = 0x03;       // power device
            config->conf_m1.data_length = 2;
            break;
        case 1:
            config->conf_m1.data[1] = config->conf_m0.connected_message->data[0] & 0b11101100;
            config->conf_m1.data[1] = config->conf_m1.data[1] | 0b00000000 | 0b00000010; // 1x gain and 402ms integration time

            config->conf_m1.data_length = 2;
            break;
        case 2:
            config->conf_m1.data[1] = 0; // LSB of lower threshold
            config->conf_m1.data[2] = 0; // MSB of lower threshold
            config->conf_m1.data_length = 3;
            break;
        case 3:
            config->conf_m1.data[1] = 0; // LSB of lower threshold
            config->conf_m1.data[2] = 0; // MSB of lower threshold
            config->conf_m1.data_length = 3;
            break;
        case 4:
            config->conf_m1.data[1] = config->conf_m0.connected_message->data[0] & 0b11000000;
            config->conf_m1.data[1] = config->conf_m1.data[1] | 0b00000000 | 0b00000000; // disable interrupts, every ADC cycle generates interrupt
            config->conf_m1.data_length = 2;
            break;

        default:
            break;
    }

    // write the updated configuration value
    config->conf_m1.n_attempts = 3;
    i2c_queue_message(&config->conf_m1);
}

void i2c_processor_config_m1_apds9301(i2c_message_t* m){
    sensor_apds9301_config_t* config = (sensor_apds9301_config_t*) m->processor_data;
    
    uint8_t state = ++(m->connected_message->processor_data[0]);
    switch(state){
        case 1:
            config->conf_m0.data[0] = 0b10000001; // timing register
            config->conf_m0.connected_message->data_length = 1;
            break;
        case 2:
            config->conf_m0.data[0] = 0b10100010; // address of LSB of lower threshold
            config->conf_m0.connected_message->data_length = 2;
            break;
        case 3:
            config->conf_m0.data[0] = 0b10100100; // address of LSB of upper threshold
            config->conf_m0.connected_message->data_length = 2;
            break;
        case 4:
            config->conf_m0.data[0] = 0b10000110; // interrupt control register
            config->conf_m0.connected_message->data_length = 1;
            break;
        default:
            break;
    }
    
    if(state < 5){
        config->conf_m0.n_attempts = 3;
        i2c_queue_message(&config->conf_m0);
    }
    
}

void config_apds9301_sensor(sensor_apds9301_config_t* config){
    void (*controller)(i2c_message_t*);
    
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_read_controller;
            break;
        default:
            controller = i2c1_write_read_controller;
            break;
    }
    
    // init messages
    i2c_init_message(&config->conf_m0,
        I2C_WRITE_ADDRESS(config->general.address),
        config->conf_m0_data,
        APDS9301_CONFIG_M0_DATA_LENGTH,
        controller,
        3,
        i2c_processor_config_m0_apds9301,
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        &config->conf_m01_connected);
    
    i2c_init_connected_message(&config->conf_m01_connected,
            &config->conf_m0,
            config->conf_m0c_data,
            APDS9301_CONFIG_M0C_DATA_LENGTH);
    
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            return;
    }
    
    i2c_init_message(&config->conf_m1,
        I2C_WRITE_ADDRESS(config->general.address),
        config->conf_m1_data,
        APDS9301_CONFIG_M1_DATA_LENGTH,
        controller,
        3,
        i2c_processor_config_m1_apds9301,
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        &config->conf_m01_connected);
    
    // small hack to make this work
    // the connected message also contains valid processor data
    // this field is used to keep track of the transmission state
    config->conf_m01_connected.processor_data = config->conf_m01c_processor_data;
    config->conf_m01_connected.processor_data_length = APDS9301_CONFIG_PROCESSOR_DATA_LENGTH;
    config->conf_m01_connected.processor_data[0] = 0; // we start in state 0
    
    // first transfer: power on the device
    config->conf_m0.data[0] = 0b10000000; // control register
    config->conf_m0.connected_message->data_length = 1;
    config->conf_m0.n_attempts = 3;
    
    // start transfer
    i2c_queue_message(&config->conf_m0);
}

i2c_error_t apds9301_init_sensor(sensor_apds9301_config_t* config){
    void (*controller)(i2c_message_t* m);
    
    // initialise message to read PD0 value
    config->m0_data[0] = 0x0C | 0x80 | 0x20;
    /*                              ||     |     |
     *                              ||     |     |------------- read a word
     *                              ||     |------------------- command
     *                              ||------------------------- register address
     */
    
    // initialise measurement messages
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_read_controller;
            break;
        default:
            report_error("APDS9301: I2C module not supported.");
            break;
    }
    
    // initialise message to read PD1 value
    config->m1_data[0] = 0x0E | 0x80| 0x20;
    /*                              ||    |          |
     *                              ||    |          |-------- read a word
     *                              ||    |------------------- command
     *                              ||------------------------- register address
     */
    
    i2c_init_message(&config->m0,
            I2C_WRITE_ADDRESS(config->general.address),
            config->m0_data,
            SENSOR_APDS3901_DATA_CMD_LENGTH,
            controller,
            3,
            apds9301_i2c_cb_m0,
            (uint8_t*) config,
            0,
            config->general.i2c_bus,
            &config->m0_connected);
    i2c_init_message(&config->m1,
            I2C_WRITE_ADDRESS(config->general.address),
            config->m1_data,
            SENSOR_APDS3901_DATA_CMD_LENGTH,
            controller,
            3,
            apds9301_i2c_cb_m1,
            (uint8_t*) config,
            0,
            config->general.i2c_bus,
            &config->m1_connected);
    
    i2c_init_connected_message(&config->m0_connected, &config->m0, config->m0c_data, SENSOR_APDS3901_DATA_LENGTH);
    i2c_init_connected_message(&config->m1_connected, &config->m1, config->m1c_data, SENSOR_APDS3901_DATA_LENGTH);
    
    // init configuration messages
    config_apds9301_sensor(config);
    i2c_empty_queue();
    
    sensor_update_status(&config->general, config->conf_m0.error);
    sensor_update_status(&config->general, config->conf_m1.error);
    
    sensor_init_common_config(&config->general, SENSOR_APDS3901_CAN_DATA_LENGTH);
    
    return config->conf_m0.error;
}




void apds9301_i2c_cb_m0(i2c_message_t* m){   
    sensor_apds9301_config_t* config = (sensor_apds9301_config_t*) m->processor_data;
    if(m->error == I2C_NO_ERROR) {
        config->general.tx_data[0] = m->connected_message->data[1];
        config->general.tx_data[1] = m->connected_message->data[0];
        
        // queue the second message
        config->m1.n_attempts = 3;
        i2c_queue_message(&config->m1);
        
    } else {
        sensor_send_error(&config->general.elog, m);
        
        config->general.tx_data[0] = 0x00;
        config->general.tx_data[1] = 0x00;
    }
}

void apds9301_i2c_cb_m1(i2c_message_t* m){
    sensor_apds9301_config_t* config = (sensor_apds9301_config_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){  
        config->general.tx_data[2] = m->connected_message->data[1];
        config->general.tx_data[3] = m->connected_message->data[0];
        
        sensor_send_data(&config->general.dlog, config->general.tx_data, SENSOR_APDS3901_CAN_DATA_LENGTH);
    } else {
        sensor_send_error(&config->general.elog, m);
    }
}
