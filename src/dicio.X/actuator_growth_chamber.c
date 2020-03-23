#include "actuator_growth_chamber.h"
#include <address.h>
#include <actuator_common.h>

void init_growth_chamber(growth_chamber_config_t* config){
    void (*controller)(i2c_message_t*);
    
    switch(config->i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("growth chamber: I2C module not supported.");;
            break;
    }
    
    config->timer_value = config->period;
    
    reset_actuator_local_id();
    
    config->general.local_id = get_actuator_local_id();
    config->general.global_id = CAN_DATA_CMD_GROWTH_CHAMBER;
    
    actuator_init_common_config(&config->general, ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH);
    
    i2c_init_message(&config->m_temp, 
            I2C_WRITE_ADDRESS(config->address), 
            config->i2c_data,
            ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH,
            controller,
            3,
            i2c_cb_growth_chamber,
            (uint8_t*) config,
            0,
            config->i2c_bus,
            NULL);
    
    i2c_init_message(&config->m_rh, 
            I2C_WRITE_ADDRESS(config->address), 
            config->i2c_data,
            ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH,
            controller,
            3,
            i2c_cb_growth_chamber,
            (uint8_t*) config,
            0,
            config->i2c_bus,
            NULL);
    
    i2c_queue_message(&config->m_rh);
    i2c_queue_message(&config->m_temp);
    i2c_empty_queue();
    
    if(config->m_temp.error != I2C_NO_ERROR){
        config->general.status = ACTUATOR_STATE_ERROR;
    } else {
        config->general.status = ACTUATOR_STATE_ACTIVE;
    }
}

void i2c_cb_growth_chamber(i2c_message_t* m){
    growth_chamber_config_t* config = (growth_chamber_config_t*) m->processor_data;
    
    if(m->error != I2C_NO_ERROR){
        send_actuator_error(&config->general, m);
    }
}


void parse_growth_chamber_data(growth_chamber_config_t* config, uint8_t channel_n){
    // parse I2C message data
    switch(channel_n){
        case ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL:
          config->m_temp.data[0] = ACTUATOR_GROWTH_CHAMBER_DAC_CH_0;
          config->m_temp.data[1] = (config->temperature & 0xff0) >> 4;
          config->m_temp.data[2] = (config->temperature & 0xf) << 4; 
          break;
        case ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL:
          config->m_rh.data[0] = ACTUATOR_GROWTH_CHAMBER_DAC_CH_1;
          config->m_temp.data[1] = (config->relative_humidity & 0xff0) >> 4;
          config->m_temp.data[2] = (config->relative_humidity & 0xf) << 4; 
          break;
        default:
          report_error("growth chamber: channel not configured.");
          break;
    }
    
    // parse CAN/UART message data
    config->general.dlog.can_message.data[0] = (config->temperature >> 8) & 0xff;
    config->general.dlog.can_message.data[1] = config->temperature & 0xff;
    config->general.dlog.can_message.data[2] = (config->relative_humidity >> 8) & 0xff;
    config->general.dlog.can_message.data[3] = config->relative_humidity & 0xff;
    
}

void growth_chamber_set_temperature(growth_chamber_config_t* config){
    i2c_reset_message(&config->m_temp, 3);
    
    parse_growth_chamber_data(config, ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL);
    
    i2c_queue_message(&config->m_temp);
    send_actuator_data_no_copy(&config->general);
}

void growth_chamber_set_relative_humidity(growth_chamber_config_t* config){
    i2c_reset_message(&config->m_rh, 3);
    
    parse_growth_chamber_data(config, ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL);
    
    i2c_queue_message(&config->m_rh);
    send_actuator_data_no_copy(&config->general);
}
