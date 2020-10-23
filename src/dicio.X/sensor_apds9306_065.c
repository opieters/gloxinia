#include "sensor_apds9306_065.h"

i2c_error_t apds9306_init_sensor(sensor_apds9306_config_t* config){    
    i2c_message_t m;
    uint8_t m_data[4];
    
    void (*controller)(i2c_message_t* m);
    
    sensor_init_common_config(&config->general, SENSOR_APDS3906_CAN_DATA_LENGTH);
    
    // initialise measurement messages
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("APDS9306_065: I2C module not supported.");
            break;
    }
    
    // configure measurement rate
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_ALS_MEAS_RATE;
    m_data[1] = (config->meas_resolution << 4) | config->meas_rate;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    // configure measurement gain
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_ALS_GAIN;
    m_data[1] = config->gain;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    // configure interrupts
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_INT_CFG;
    m_data[1] = (0b01 << 4) | (0 << 3) | (0 << 2);
    /**             |          |          |- interrupts disabled   
     *              |          ------------- threshold interrupt mode
     *              ------------------------ ALS channel as interrupt source
     */
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    // configure interrupt persistence
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_INT_PERSISTENCE;
    m_data[1] = (0b00 << 4); // every ALS value out of the range triggers an interrupt
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    
    // configure high threshold
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_ALS_THRES_UP_0;
    m_data[1] = config->apds9306_als_threshold_high & 0xff;
    m_data[2] = (config->apds9306_als_threshold_high >> 8) & 0xff;
    m_data[3] = (config->apds9306_als_threshold_high >> 16) & 0xf;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    // configure low threshold
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_ALS_THRES_LOW_0;
    m_data[1] = config->apds9306_als_threshold_low & 0xff;
    m_data[2] = (config->apds9306_als_threshold_low >> 8) & 0xff;
    m_data[3] = (config->apds9306_als_threshold_low >> 16) & 0xf;
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    // activate sensor
    i2c_init_message(&m,
            I2C_WRITE_ADDRESS(config->general.address),
            m_data,
            2,
            controller,
            3,
            i2c_dummy_callback,
            NULL,
            NULL,
            0,
            config->general.i2c_bus,
            NULL);
    m_data[0] = S_APDS9306_R_MAIN_CTRL;
    m_data[1] = (1 << 1); // turn sensor on
    
    i2c_queue_message(&m);
    i2c_empty_queue();
    
    // initialise message to read PD value
    // initialise measurement messages
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_read_controller;
            break;
        default:
            report_error("APDS9306_065: I2C module not supported.");
            break;
    }
    
    config->m_read_address[0] = S_APDS9306_R_ALS_DATA_0; // select DATA register
    
    i2c_init_message(&config->m_read_setup,
            I2C_WRITE_ADDRESS(config->general.address),
            config->m_read_address,
            ARRAY_LENGTH(config->m_read_address),
            controller,
            3,
            apds9306_065_i2c_cb,
            NULL,
            (uint8_t*) config,
            0,
            config->general.i2c_bus,
            &config->m_read);
    
    i2c_init_connected_message(&config->m_read, &config->m_read_setup, 
            config->m_read_data, ARRAY_LENGTH(config->m_read_data));
    
    sensor_update_status(&config->general, m.error);
    
    return m.error;
}


void apds9306_065_i2c_cb(i2c_message_t* m){
    sensor_apds9306_config_t* config = (sensor_apds9306_config_t*) m->processor_data;
    
    if(m->error != I2C_NO_ERROR){
        sensor_send_error(&config->general.elog, m);
    } else {
        sensor_send_data(&config->general.dlog, m->connected_message->data, SENSOR_APDS3906_CAN_DATA_LENGTH);
    }
}
