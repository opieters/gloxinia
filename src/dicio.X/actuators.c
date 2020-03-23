#include "actuators.h"
#include "dicio.h"
#include "actuator_pump.h"
#include "actuator_growth_chamber.h"

pump_config_t pump_config[N_ACTUATOR_PUMP];

const pump_config_t pump_default_config = {
    .general = {
        .status = ACTUATOR_STATE_INACTIVE,
    },
    .pin = PIN_INIT(F, 1),
    .period = 360,
    .on_time = 60,
};

#ifdef ENABLE_GROWTH_CHAMBER
growth_chamber_config_t growth_chamber_config = {
    .address = ACTUATOR_GROWTH_CHAMBER_DAC_ADDRESS,
    .i2c_bus = I2C2_BUS,
    .temperature = ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL,
    .relative_humidity = ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL,
    .period = 60
};
#endif

void actuators_init(void){
    uint16_t i;
    
    for(i = 0; i < N_ACTUATOR_PUMP; i++){
        init_actuator_pump(&pump_config[i]);
    }
    
    #ifdef ENABLE_GROWTH_CHAMBER
    init_growth_chamber(&growth_chamber_config);
    #endif
    
    dicio_set_actuator_callback(actuator_callback);
}

void actuator_callback(void) {
    static uint16_t interrupt_counter = 0;
    uint16_t i;
    
    interrupt_counter++;
    
    if(interrupt_counter == 10){
        for(i = 0; i < N_ACTUATOR_PUMP; i++){
            if(pump_config[i].general.status == ACTUATOR_STATE_ACTIVE){
                pump_config[i].timer_value--;

                if(pump_config[i].timer_value == 0){
                    switch(pump_config[i].status){
                        case PUMP_STATUS_OFF:
                            start_pump(&pump_config[i]);
                            pump_config[i].status = PUMP_STATUS_ON;

                            pump_config[i].timer_value = pump_config[i].on_time;
                            break;
                        case PUMP_STATUS_ON:
                            stop_pump(&pump_config[i]);
                            pump_config[i].status = PUMP_STATUS_OFF;

                            pump_config[i].timer_value = pump_config[i].period - pump_config[i].on_time;
                            break;
                        default:
                            report_error("PUMP: unknown status.");
                            break;
                    }
                }
            }
        }
        
        #ifdef ENABLE_GROWTH_CHAMBER
        if(growth_chamber_config.general.status == ACTUATOR_STATE_ACTIVE){
            growth_chamber_config.timer_value--;

            if(growth_chamber_config.timer_value == 0){
                growth_chamber_config.timer_value = growth_chamber_config.period;

                growth_chamber_set_temperature(&growth_chamber_config);
                growth_chamber_set_relative_humidity(&growth_chamber_config);
            }
        }
        #endif
        
        interrupt_counter = 0;
    }
}

void actuators_start(void){
    
}

void actuators_data_init(void){
    size_t i;
    
#if N_ACTUATOR_PUMP > 0
    for(i = 0; i < N_ACTUATOR_PUMP; i++){
         pump_config[i] = pump_default_config;
    }
#endif
}
