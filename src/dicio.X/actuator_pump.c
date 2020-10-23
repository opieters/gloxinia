#include "actuator_pump.h"
#include <address.h>
#include "actuator_growth_chamber.h"

void init_actuator_pump(pump_config_t* config){
    // configure pump pin
    CLEAR_BIT(config->pin.tris_r, config->pin.n); // configure as output
    CLEAR_BIT(config->pin.port_r, config->pin.n); // set pin low
    
    reset_actuator_local_id();
    config->general.local_id = get_actuator_local_id();
    config->general.global_id = CAN_DATA_CMD_PUMP;
    config->general.status = ACTUATOR_STATE_ACTIVE;
    
    actuator_init_common_config(&config->general, ACTUATOR_PUMP_DATA_LENGTH);
    
    config->general.status = ACTUATOR_STATE_ACTIVE;
    
    // force pump to start for several seconds during init.
    config->status = PUMP_STATUS_OFF;
    config->timer_value = 1;
    actuator_pump_cb(config);
    
    config->timer_value = 5;
}

void start_pump(pump_config_t* config){
    uint8_t log_data[1];
    
    SET_BIT(config->pin.port_r, config->pin.n); // set pin high
    
    config->status = PUMP_STATUS_ON;
    
    log_data[0] = config->status;
    send_actuator_data(&config->general, log_data, ARRAY_LENGTH(log_data));
}

void stop_pump(pump_config_t* config){
    uint8_t log_data[1];
    
    CLEAR_BIT(config->pin.port_r, config->pin.n); // set pin low
    
    config->status = PUMP_STATUS_OFF;
    
    log_data[0] = config->status;
    send_actuator_data(&config->general, log_data, ARRAY_LENGTH(log_data));
}

void actuator_pump_cb(pump_config_t* config){
    if(config->general.status == ACTUATOR_STATE_ACTIVE){
        config->timer_value--;

        if(config->timer_value == 0){
            switch(config->status){
                case PUMP_STATUS_OFF:
                    start_pump(config);

                    config->timer_value = config->on_time;
                    break;
                case PUMP_STATUS_ON:
                    stop_pump(config);
                    
                    config->timer_value = config->period - config->on_time;
                    break;
                default:
                    report_error("PUMP: unknown status.");
                    break;
            }
        }
    }
}
