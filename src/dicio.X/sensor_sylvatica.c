#include "sensor_sylvatica.h"
#include "dicio.h"
#include <address.h>

extern volatile uint8_t i2c_transfer_status;
extern volatile uint8_t transfer_done;

void sylvatica_measurement_read_cb(i2c_message_t* m){
    sensor_sylvatica_channel_config_t* config = 
            (sensor_sylvatica_channel_config_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        // transmit received data
        sensor_send_data_no_copy(&config->dlog, &m->connected_message->data[1], 
                m->connected_message->data_length-1);
    } else {
        sensor_send_error(&config->sensor_config->general.elog, m);
    }
}

void sylvatica_measurement_raw_read_cb(i2c_message_t* m){
    sensor_sylvatica_raw_channel_config_t* config = 
            (sensor_sylvatica_raw_channel_config_t*) m->processor_data;
    
    if(m->error == I2C_NO_ERROR){
        // transmit received data
        sensor_send_data_no_copy(&config->dlog, &m->connected_message->data[1], 
                m->connected_message->data_length-1);
    } else {
        sensor_send_error(&config->sensor_config->general.elog, m);
    }
}


i2c_error_t sylvatica_init_sensor(sensor_sylvatica_config_t* config){
    void (*controller)(i2c_message_t* m);
    uint16_t i;
    
    sensor_init_common_config(&config->general, 0);

    // start sylvatica
    switch(config->general.i2c_bus) {
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("sylvatica: I2C module not supported.");
            break;
    }
    
    config->m_status_config_data[0] = SYLVATICA_REG_STATUS;
    config->m_status_config_data[1] = SYLVATICA_STATUS_ON | SYLVATICA_STATUS_RESET_BUFFER;
    
    i2c_init_message(&config->m_status_config, 
            I2C_WRITE_ADDRESS(config->general.address), 
            config->m_status_config_data, 2, controller, 3, i2c_dummy_callback, 
            NULL, 0, config->general.i2c_bus, NULL);
    
    i2c_queue_message(&config->m_status_config);
    i2c_empty_queue();
    
    // TODO: use gain configuration in struct
    // TODO: call calibration function and send result to logger
    
    // I2C messages
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){    
        sylvatica_sensor_init_channel(&config->channels[i], config, i); 
        sylvatica_sensor_init_raw_channel(&config->raw_channels[i], config, i);
    }
    
    // turn on ADC
    config->m_adc_config_data[0] = SYLVATICA_REG_ADC;
    config->m_adc_config_data[1] = SYLVATICA_ADC_ON;

    i2c_init_message(&config->m_adc_config, 
            I2C_WRITE_ADDRESS(config->general.address), 
            config->m_adc_config_data, 2, controller, 3, i2c_dummy_callback, 
            NULL, 0, config->general.i2c_bus, NULL);
    
    if(config->m_status_config.error != I2C_NO_ERROR){            
        config->general.status = SENSOR_STATUS_ERROR;
    } else {
        config->general.status = SENSOR_STATUS_ACTIVE;
    }
    
    if(config->m_status_config.error != I2C_NO_ERROR){
        return config->m_status_config.error;
    }
    
    // read the configuration from sensor and report
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){   
        sylvatica_read_channel_config(&config->channels[i], config, i);
    }
    
    return config->m_status_config.error;
}

void sylvatica_sensor_start(sensor_sylvatica_config_t* config){
    i2c_queue_message(&config->m_adc_config);
}

void sylvatica_read_channel_config(sensor_sylvatica_channel_config_t* config, 
        sensor_sylvatica_config_t* sensor_config,
        const uint8_t channel_n){
    
    uint8_t data_write[1], read_data[1];
    i2c_message_t m_write, m_read;
    void (*controller)(i2c_message_t* m);
    
    switch(sensor_config->general.i2c_bus) {
        case I2C1_BUS:
            controller = i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_read_controller;
            break;
        default:
            report_error("sylvatica: I2C module not supported.");
            break;
    }
    
    // init I2C message
    i2c_init_message(&m_write, sensor_config->general.address, data_write, 
            ARRAY_LENGTH(data_write), controller,
            3, i2c_dummy_callback, NULL, 0,  sensor_config->general.i2c_bus,
            &m_read);
    i2c_init_connected_message(&m_read,
        &m_write,
        read_data,
        ARRAY_LENGTH(read_data));
    
    // fill data
    data_write[0] = SYLVATICA_REG_CONFIG_CH0 + channel_n;
    
    // read I2C status
    i2c_queue_message(&m_write);
    i2c_empty_queue();
    
    // tx status message over CAN/UART
    dicio_send_message(SERIAL_SENSOR_CONFIG_CMD, 
            CAN_HEADER(CAN_CONFIG_CMD_SYLVATICA, config->local_id), 
            read_data, ARRAY_LENGTH(read_data));
    
}

void sylvatica_sensor_init_channel(sensor_sylvatica_channel_config_t* config, 
        sensor_sylvatica_config_t* sensor_config,
        const uint8_t channel_n){
    void (*controller)(i2c_message_t* m);
    
    config->sensor_config = sensor_config;
    
    // configure all channels
    // I2C messages to read the data
    switch(sensor_config->general.i2c_bus) {
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("sylvatica: I2C module not supported.");
            break;
    }
    
    config->m_read_data = config->readout_data;
    
    config->m_ch_config_data[0] = SYLVATICA_REG_CONFIG_CH0 + channel_n;
    config->m_ch_config_data[1] = SYLVATICA_CH_CONFIG_ON | SYLVATICA_CH_CONFIG_SET_GAIN(PGA_GAIN_1);

    i2c_init_message(&config->m_ch_config, I2C_WRITE_ADDRESS(sensor_config->general.address), config->m_ch_config_data, 2,
        controller, 3, i2c_dummy_callback, (uint8_t*) config, 0, sensor_config->general.i2c_bus,
        NULL);

    i2c_queue_message(&config->m_ch_config);
    i2c_empty_queue();
    
    // I2C messages to read the data
    switch(sensor_config->general.i2c_bus) {
        case I2C1_BUS:
            controller = sylvatica_i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = sylvatica_i2c2_write_read_controller;
            break;
        default:
            report_error("sylvatica: I2C module not supported.");
            break;
    }
    
    config->m_write_data[0] = SYLVATICA_REG_DATA_CH0 + channel_n;

    i2c_init_message(&config->m_write, 
        I2C_WRITE_ADDRESS(sensor_config->general.address),
        config->m_write_data, 
        1, 
        controller, 
        1, 
        sylvatica_measurement_read_cb, 
        (uint8_t*) config, 
        0,  
        sensor_config->general.i2c_bus, 
        &config->m_read);

    i2c_init_connected_message(&config->m_read,
            &config->m_write, 
            config->m_read_data, 
            SYLVATICA_I2C_READ_BUFFER_LENGTH);

    can_init_message(&config->dlog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_DATA_CMD_SYLVATICA, config->local_id),
            &config->readout_data[1],
            SENSOR_SYLVATICA_CAN_DATA_LENGTH);
    
    // attention: does not use the data array from the general struct like
    // the other sensors!
    uart_init_message(&config->dlog.uart_message, 
            SERIAL_SENSOR_DATA_CMD,
            controller_address,
            CAN_HEADER(CAN_DATA_CMD_SYLVATICA, config->local_id),            
            &config->readout_data[1],
            SYLVATICA_I2C_READ_BUFFER_LENGTH);
    
    if(config->m_ch_config.error == I2C_NO_ERROR){
        config->status = SENSOR_STATUS_ACTIVE;
    } else {
        config->status = SENSOR_STATUS_INACTIVE;
    }
}

void sylvatica_sensor_init_raw_channel(sensor_sylvatica_raw_channel_config_t* config, 
        sensor_sylvatica_config_t* sensor_config,
        const uint8_t channel_n){
    void (*controller)(i2c_message_t* m);
    
    config->sensor_config = sensor_config;
    
    // configure all channels
    // I2C messages to read the data
    switch(sensor_config->general.i2c_bus) {
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("sylvatica: I2C module not supported.");
            break;
    }
    
    config->m_read_data = config->readout_data;
    
    // I2C messages to read the data
    switch(sensor_config->general.i2c_bus) {
        case I2C1_BUS:
            controller = sylvatica_i2c1_write_read_controller;
            break;
        case I2C2_BUS:
            controller = sylvatica_i2c2_write_read_controller;
            break;
        default:
            report_error("sylvatica: I2C module not supported.");
            break;
    }
    
    config->m_write_data[0] = SYLVATICA_REG_RAW_CH0 + channel_n;

    i2c_init_message(&config->m_write, 
        I2C_WRITE_ADDRESS(sensor_config->general.address),
        config->m_write_data, 
        1, 
        controller, 
        1, 
        sylvatica_measurement_raw_read_cb, 
        (uint8_t*) config, 
        0,  
        sensor_config->general.i2c_bus, 
        &config->m_read);

    i2c_init_connected_message(&config->m_read,
            &config->m_write, 
            config->m_read_data, 
            SYLVATICA_I2C_READ_BUFFER_LENGTH);

    can_init_message(&config->dlog.can_message,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_DATA_CMD_SYLVATICA, config->local_id),
            &config->readout_data[1],
            SENSOR_SYLVATICA_CAN_DATA_LENGTH);
    
    // attention: does not use the data array from the general struct like
    // the other sensors!
    uart_init_message(&config->dlog.uart_message, 
            SERIAL_SENSOR_DATA_CMD,
            controller_address,
            CAN_HEADER(CAN_DATA_CMD_SYLVATICA, config->local_id),            
            &config->readout_data[1],
            SYLVATICA_I2C_READ_BUFFER_LENGTH);
    
    if(sensor_config->channels[channel_n].status == SENSOR_STATUS_ACTIVE){
        config->status = SENSOR_STATUS_ACTIVE;
    }
}

void sylvatica_sensor_read(sensor_sylvatica_config_t* config){
    uint16_t i;
    
    for(i = 0; i < SYLVATICA_N_CHANNELS; i++){
        if(config->channels[i].status == SENSOR_STATUS_ACTIVE){
            i2c_reset_message(&config->channels[i].m_read, 1);
            config->channels[i].m_read.data_length = SYLVATICA_I2C_READ_BUFFER_LENGTH;
            i2c_reset_message(&config->channels[i].m_write, 1);
            i2c_queue_message(&config->channels[i].m_write);
        }
        
        if(config->raw_channels[i].status == SENSOR_STATUS_ACTIVE){
            i2c_reset_message(&config->raw_channels[i].m_read, 1);
            config->raw_channels[i].m_read.data_length = SYLVATICA_I2C_READ_BUFFER_LENGTH;
            i2c_reset_message(&config->raw_channels[i].m_write, 1);
            i2c_queue_message(&config->raw_channels[i].m_write);
        }
    }
}

void sylvatica_i2c1_write_read_controller(i2c_message_t* m){
    static uint16_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            i2c_transfer_status = 1;
            n_transfers = 0;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C1CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for reading
            transfer_done = 0;
            I2C1TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // transfer data byte
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C1CONbits.PEN = 1;
            } else {
                I2C1TRN = m->data[n_transfers];
                n_transfers++;
                
                if(n_transfers == m->data_length){
                    i2c_transfer_status = 4;
                } else {
                    i2c_transfer_status = 3;
                }
            }
            break;
        case 4:
            // check final ACK and send repeated start event
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C1CONbits.PEN = 1;           
            } else {
                I2C1CONbits.RSEN = 1;
                i2c_transfer_status = 5;
            }
            break;
        case 5:
            // send start byte with address for reading
            transfer_done = 0;
            I2C1TRN = m->connected_message->address;
            i2c_transfer_status = 6;
            n_transfers = 0;
            break;
        case 6:
            // enable receive mode
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C1CONbits.PEN = 1;  
            } else {
                I2C1CONbits.RCEN = 1;
                i2c_transfer_status = 7;      
            }
            break;
        case 7:
            // receive data byte
            transfer_done = 0;
            
            m->connected_message->data[n_transfers] = I2C1RCV;
            n_transfers++;
            
            // actual data length
            if(n_transfers == 1){
                m->connected_message->data_length = MIN(m->connected_message->data_length, m->connected_message->data[0]+1);
            }
            
            // normal message handling
            if(n_transfers == m->connected_message->data_length){
                I2C1CONbits.ACKDT = 1; // send NACK
                i2c_transfer_status = 8;  
            } else {
                I2C1CONbits.ACKDT = 0; // send ACK
                i2c_transfer_status = 6;
            }
            I2C1CONbits.ACKEN = 1; // start ACK event
            
            break;
        case 8:
            // send stop event
            transfer_done = 0;
            i2c_transfer_status = 9;
            I2C1CONbits.PEN = 1;
            break;
        case 9:
            m->status = I2C_MESSAGE_PROCESSING;
            i2c_transfer_status = 0;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            i2c_transfer_status = 0;
            m->status = m->status = I2C_MESSAGE_PROCESSING;
            I2C1CONbits.PEN = 1;
            break;
    }
}

void sylvatica_i2c2_write_read_controller(i2c_message_t* m){
    
}
