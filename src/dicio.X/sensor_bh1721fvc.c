#include "sensor_bh1721fvc.h"


void bh1721fvc_init_sensor(sensor_bh1721fvc_config_t* config) {
    void (*controller)(i2c_message_t*);
    
    sensor_init_common_config(&config->general, SENSOR_BH1721FVC_CAN_DATA_LENGTH);
    
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_read_controller;
            break;
        default:
            controller = i2c1_read_controller;
            break;
    }
    
    i2c_init_message(&config->m,
            I2C_READ_ADDRESS(config->general.address),
            config->m_data,
            SENSOR_BH1721FVC_DATA_LENGTH,
            controller,
            3,
            bh1721fvc_i2c_cb,
            (uint8_t*) config,
            0,
            config->general.i2c_bus,
            NULL);
   
    config->mc0_data[0] = 0b00000001 ; // power on device
    
    switch(config->general.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            controller = i2c2_write_controller;
            break;
    }
    
    i2c_init_message(
        &config->mc0,
        I2C_WRITE_ADDRESS(config->general.address),
        config->mc0_data,
        BH1721FVC_CONFIG_M01_DATA_LENGTH,
        controller,
        3,
        bh1721fvc_i2c_cb_mc0,
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        NULL);
    
    i2c_init_message(
        &config->mc1,
        I2C_WRITE_ADDRESS(config->general.address),
        config->mc1_data,
        BH1721FVC_CONFIG_M01_DATA_LENGTH,
        controller,
        3,
        i2c_dummy_callback,
        (uint8_t*) config,
        0,
        config->general.i2c_bus,
        NULL);
    
    config->mc1_data[0] = 0b00010010; // high resolution mode
    
    i2c_queue_message(&config->mc0);
    
    i2c_empty_queue();
    
    sensor_update_status(&config->general, config->mc0.error);
    sensor_update_status(&config->general, config->mc1.error);
}


void bh1721fvc_i2c_cb_mc0(i2c_message_t* m){
    sensor_bh1721fvc_config_t* config = (sensor_bh1721fvc_config_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        config->mc1.n_attempts = 3;
        i2c_queue_message(&config->mc1);
    } else {
        sensor_send_error(&config->general.elog, m);
    }
}


/*******************************************************************************
 * 
 * Definitions of functions that process the received I2C data and send the 
 * necessary CAN responses.
 * 
 ******************************************************************************/

void bh1721fvc_i2c_cb(i2c_message_t* m){
    sensor_bh1721fvc_config_t* config = (sensor_bh1721fvc_config_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        sensor_send_data(&config->general.dlog, m->data, SENSOR_BH1721FVC_CAN_DATA_LENGTH);
       
    } else {
        sensor_send_error(&config->general.elog, m);
    }
}
