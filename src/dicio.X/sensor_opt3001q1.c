#include "sensor_opt3001q1.h"
/**
 * @brief Initialisation of OPT3001-Q1 sensor.
 * 
 * @details Resets the OPT3001-Q1 sensor and initialises it.
 * 
 * @return i2c_status_t indicating status of the I2C transfer.
 */
i2c_error_t opt3001q1_init_sensor(sensor_opt3001q1_config_t* config){ 
    void (*controller)(i2c_message_t*);
    
    sensor_init_common_config(&config->general, SENSOR_OPT3001Q1_CAN_DATA_LENGTH);
    
    // configure read data operation
    switch(config->m_read.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_read_controller;
            break;
        default:
            report_error("OPT3001-Q1: I2C module not supported.");
            break;
    }
    
    i2c_init_message(&config->m_read,
        I2C_READ_ADDRESS(config->general.address),
        config->m_read_data,
        SENSOR_OPT3001Q1_DATA_LENGTH,
        controller,
        3,
        opt3001q1_i2c_cb_read,
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        NULL);
    
    // configuration of the control register (issued first)
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("OPT3001-Q1: I2C module not supported.");
            break;
    }
    
    config->m_control_data[0] = 0x10; // configuration register
    
    config->m_control_data[1] = 0b11001010;
    /*                 |-- read only
     *               ||--- single shot mode
     *              |----- 800 ms measurement  
     *          ||||------ auto scale mode
     */
    config->m_control_data[2] = 0b00000000;
    /*                ||-- 1 fault count
     *               |---- do not mask exponent field
     *              |----- INT active low  
     *             |------ latch not cleared by user
     *          |||------- read only
     */
    
    i2c_init_message(&config->m_control,
        I2C_WRITE_ADDRESS(config->general.address),
        config->m_control_data,
        SENSOR_OPT3001Q1_CONTROL_LENGTH,
        controller,
        3,
        i2c_dummy_callback, // we do not yet set the correct controller here
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        NULL);
    
    // configuration of the read data register
    config->m_read_setup.data[0] = 0; // TODO
    
    //config->m_read.address = address;
    switch(config->m_read_setup.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("OPT3001-Q1: I2C module not supported.");
            break;
    }
    i2c_init_message(&config->m_read_setup,
        I2C_WRITE_ADDRESS(config->general.address),
        config->m_read_setup_data,
        SENSOR_OPT3001Q1_READ_SETUP_LENGTH,
        controller,
        3,
        i2c_dummy_callback,
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        NULL);
    
    // start the first measurement
    // this allows to update the sensor status and results in a successful first
    // sensor readout
    i2c_queue_message(&config->m_control);
    i2c_queue_message(&config->m_read_setup);
    
    i2c_empty_queue();
    config->m_control.callback = opt3001q1_i2c_cb_control;
    
    sensor_update_status(&config->general, config->m_control.error);
    
    return config->m_read_setup.error;
}

void opt3001q1_i2c_cb_control(i2c_message_t* m){
    sensor_opt3001q1_config_t* config = (sensor_opt3001q1_config_t*) m->processor_data;
    
    if(m->error != I2C_NO_ERROR){
        sensor_send_error(&config->general.elog, m);
    } else {
        config->m_read_setup.n_attempts = 1;
        i2c_queue_message(&config->m_read_setup);
    }
}

void opt3001q1_i2c_cb_read(i2c_message_t* m){
    sensor_opt3001q1_config_t* config = (sensor_opt3001q1_config_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        sensor_send_data(&config->general.dlog, config->general.tx_data, SENSOR_OPT3001Q1_CAN_DATA_LENGTH);
    } else {
        sensor_send_error(&config->general.elog, m);
    }
    
    config->m_control.n_attempts = 1;
    // start next measurement of this sensor
    i2c_queue_message(&config->m_control);
}
