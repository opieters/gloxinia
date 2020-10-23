#include "actuators.h"
#include "dicio.h"
#include "actuator_pump.h"
#include "actuator_growth_chamber.h"
#include "address.h"
#include "actuator_relay_board.h"

pump_config_t pump_config[N_ACTUATOR_PUMP];

bool actuator_error_detected = false;


const pump_config_t pump_default_config = {
    .general = {
        .status = ACTUATOR_STATE_INACTIVE,
    },
    .pin = PIN_INIT(D, 9),
    .period = 1UL*24UL*60UL*60UL, // turn on every two days
    .on_time = 90UL, // for 90 seconds
};

growth_chamber_config_t growth_chamber_config = {
    .address = ACTUATOR_GROWTH_CHAMBER_DAC_ADDRESS,
    .i2c_bus = I2C2_BUS,
    .temperature = ACTUATOR_GROWTH_CHAMBER_TEMP_DEFAULT,
    .relative_humidity = ACTUATOR_GROWTH_CHAMBER_RH_DEFAULT,
    .period = 1,
    .uart1_interface_up = false,
    .uart3_interface_up = false,
};

uart_message_t serial_actuator_trigger;
can_message_t can_actuator_trigger;

actuator_relay_board_t relay_boards[N_ACTUATOR_RELAY_BOARD];


void actuators_init(void){
    uint16_t i;
    
    if(controller_can_address == 0){
        uart_init_message(&serial_actuator_trigger,
            SERIAL_ACTUATOR_TRIGGER_CMD,
            controller_address,
            CAN_HEADER(CAN_INFO_CMD_ACTUATOR_START, 0),            
            NULL,
            0);
        can_init_message(&can_actuator_trigger,
            controller_can_address,
            CAN_NO_REMOTE_FRAME,
            CAN_EXTENDED_FRAME,
            CAN_HEADER(CAN_INFO_CMD_ACTUATOR_START, 0),
            NULL,
            0);
        
        dicio_set_actuator_callback(actuator_callback);
    }
    
    for(i = 0; i < N_ACTUATOR_RELAY_BOARD; i++){
        relay_boards[i].general.global_id = CAN_DATA_CMD_RELAY_BOARD;
        relay_boards[i].general.local_id = get_actuator_local_id();
        
        switch(i){
            case 0:
                relay_boards[i].i2c_address = ACTUATOR_RELAY_BOARD_I2C_ADDRESS_0;
                break;
            case 1:
                relay_boards[i].i2c_address = ACTUATOR_RELAY_BOARD_I2C_ADDRESS_1;
                break;
            case 2:
                relay_boards[i].i2c_address = ACTUATOR_RELAY_BOARD_I2C_ADDRESS_2;
                break;
            case 3:
                relay_boards[i].i2c_address = ACTUATOR_RELAY_BOARD_I2C_ADDRESS_3;
                break;
            default:
                relay_boards[i].i2c_address = ACTUATOR_RELAY_BOARD_I2C_ADDRESS_0;
                break;
        }
        relay_boards[i].i2c_bus = I2C2_BUS;
        relay_boards[i].new_output = 0;
        relay_boards[i].output = 0;
        
        actuator_relay_board_init(&relay_boards[i]);
    }
    
    reset_actuator_local_id();
    growth_chamber_config.general.global_id = CAN_DATA_CMD_GROWTH_CHAMBER;
    growth_chamber_config.general.local_id = get_actuator_local_id();
    init_growth_chamber(&growth_chamber_config);
    
    for(i = 0; i < N_ACTUATOR_PUMP; i++){
        init_actuator_pump(&pump_config[i]);
    }
    
    i2c_empty_queue();
}

void actuator_callback(void) {
    static uint16_t interrupt_counter = ACTUATOR_PERIOD-1;
    uint16_t i;
    
    interrupt_counter++;
    
    if(interrupt_counter == ACTUATOR_PERIOD){
        interrupt_counter = 0;
        
        if(controller_address == 0){
            uart_reset_message(&serial_actuator_trigger);
            uart_queue_message(&serial_actuator_trigger);
        }
    }
    
    if(interrupt_counter == N_ACTUATOR_RELAY_BOARD){   
        actuator_gc_callback();
    }
    
    for(i = 0; i < N_ACTUATOR_RELAY_BOARD; i++){
        if(interrupt_counter == i){  
            actuator_relay_board_callback(&relay_boards[i]);
        }
    }
    
    if((interrupt_counter % 10) == 0){
        for(i = 0; i < N_ACTUATOR_PUMP; i++){
            actuator_pump_cb(&pump_config[i]);
        }
    }
    
    actuators_error_check();
}

void actuators_start(void){
    
}

void actuators_data_init(void){
#if N_ACTUATOR_PUMP > 0
    size_t i;
    
    for(i = 0; i < N_ACTUATOR_PUMP; i++){
         pump_config[i] = pump_default_config;
    }
#endif
}

bool process_actuator_serial_command(uart_message_t* m){
    uint16_t i;
    can_message_t can_m;
    uint8_t can_data[CAN_MAX_N_BYTES];
    
    switch(m->command){
        case SERIAL_ACTUATOR_DATA_CMD:
        case SERIAL_ACTUATOR_ERROR_CMD:
        case SERIAL_ACTUATOR_TRIGGER_CMD:
            break;
        case SERIAL_ACTUATOR_GC_TEMP:
            gc_update_temperature((m->data[0] << 8) | m->data[1]);
            break;
        case SERIAL_ACTUATOR_GC_RH:
            gc_update_rh((m->data[0] << 8) | m->data[1]);
            break;
        case SERIAL_ACTUATOR_GC_LEDS:
            for(i = 0; i < N_ACTUATOR_RELAY_BOARD; i++){
                actuator_relay_board_update(&relay_boards[i], m->data[i]);
            }
            
            break;
        case SERIAL_ACTUATOR_PUMP:
            if(m->id == controller_address){
                for(i = 0; i < N_ACTUATOR_PUMP; i++){
                    if(GET_BIT(m->data, i)){
                        start_pump(&pump_config[i]);
                    } else {
                        stop_pump(&pump_config[i]);
                    }
                }
            } else {
                can_init_message(&can_m, m->id, 
                    CAN_NO_REMOTE_FRAME, CAN_EXTENDED_FRAME, m->extended_id, can_data, m->length);
                
                for(i = 0; i < m->length; i++){
                    can_data[i] = m->data[i];
                }
                can_send_message_any_ch(&can_m);
            }
            break;
        default:
            return false;
            break;
    }
    
    return true;
}

void actuators_error_check(void){
    size_t i;
    
    for(i = 0; i < N_ACTUATOR_RELAY_BOARD; i++){
        if(relay_boards[i].general.elog.n_errors >= ACTUATOR_ERROR_TH){
            actuator_error_detected = true;
            return;
        }
    }

    if(growth_chamber_config.general.elog.n_errors >= ACTUATOR_ERROR_TH){
        actuator_error_detected = true;
        return;
    }
}

void actuators_error_recover(void){
    size_t i;
    
    if(actuator_error_detected){
        
        sprintf(print_buffer, "Actuator readout error occurred. Resetting...");
        uart_print(print_buffer, strlen(print_buffer));
        
        delay_ms(100);
        
        asm("RESET");
        
        actuator_error_detected = false;
        for(i = 0; i < N_ACTUATOR_RELAY_BOARD; i++){
            if(relay_boards[i].general.elog.n_errors >= ACTUATOR_ERROR_TH){
                actuator_relay_board_init(&relay_boards[i]);
                relay_boards[i].general.elog.n_errors = 0;
            }
        }

        if(growth_chamber_config.general.elog.n_errors >= ACTUATOR_ERROR_TH){
            init_growth_chamber(&growth_chamber_config);
            growth_chamber_config.general.elog.n_errors = 0;
        }
    }
}