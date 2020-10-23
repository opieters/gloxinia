#include "sensors.h"
#include <address.h>
#include "sensor_licor.h"

const sensor_planalta_config_t sensor_planalta_config_default = {
    .general = {
        .address = I2C_ADDRESS_0_SENSOR_PLANALTA,
        .i2c_bus = I2C1_BUS,
    },
};

const sensor_licor_config_t sensor_licor_config_default = {
    .pin = PIN_INIT(F, 3)
};

const sensor_sylvatica_config_t sensor_sylvatica_config_default = {
    .general = {                                    
        .address = I2C_ADDRESS_0_SENSOR_SYLVATICA,  
        .i2c_bus = I2C1_BUS,                        
    },                                              
};

#if N_SYLVATICA_SENSORS > 0
sensor_sylvatica_config_t sylvatica_config[N_SYLVATICA_SENSORS];
#endif

#if N_PLANALTA_SENSORS > 0
sensor_planalta_config_t planalta_config[N_PLANALTA_SENSORS];
#endif

const sensor_apds9301_config_t sensor_apds9301_config_default = { 
    .general = { 
        .i2c_bus = I2C2_BUS,
        .address = I2C_ADDRESS_1_SENSOR_APDS_9301,
        .status = SENSOR_STATUS_INACTIVE, 
    } 
};

const sensor_sht35_config_t sensor_sht35_config_default = { 
    .general = { 
        .i2c_bus = I2C2_BUS,
        .address = I2C_ADDRESS_0_SENSOR_SHT35,
        .status = SENSOR_STATUS_INACTIVE, 
    } 
};

const sensor_bh1721fvc_config_t bh1721fvc_config_default = { 
    .general = { 
        .i2c_bus = I2C2_BUS,\
        .address = I2C_ADDRESS_SENSOR_BH1721FVC,
        .status = SENSOR_STATUS_INACTIVE, 
    } 
};

const sensor_opt3001q1_config_t sensor_opt3001q1_config_default = { 
    .general = { 
        .i2c_bus = I2C2_BUS,
        .address = I2C_ADDRESS_1_SENSOR_OPT3001Q1,
        .status = SENSOR_STATUS_INACTIVE, 
    } 
};

const sensor_apds9306_config_t sensor_apds9306_config_default = { 
    .general = { 
        .i2c_bus = I2C2_BUS,
        .address = I2C_ADDRESS_SENSOR_APDS9306,
        .status = SENSOR_STATUS_INACTIVE, 
    } 
};

#if N_SENSOR_APDS9301 > 0
sensor_apds9301_config_t apds9301_config[N_SENSOR_APDS9301];
#endif

#if N_SENSOR_SHT35 > 0
sensor_sht35_config_t sht35_config[N_SENSOR_SHT35];
#endif

#if N_SENSOR_BH1721FVC > 0
sensor_bh1721fvc_config_t bh1721fvc_config[N_SENSOR_BH1721FVC];
#endif

#if N_SENSOR_OPT3001Q1 > 0
sensor_opt3001q1_config_t opt3001q1_config[N_SENSOR_OPT3001Q1];
#endif

#if N_SENSOR_APDS9306 > 0
sensor_apds9306_config_t apds9306_config[N_SENSOR_APDS9306];
#endif

#if N_SENSOR_LICOR > 0
sensor_licor_config_t licor_config[N_SENSOR_LICOR];
#endif

uart_message_t serial_meas_trigger;
can_message_t can_meas_trigger;

volatile bool sensor_error_detected = false;
volatile bool sensor_reset_env_sensors = false;
volatile bool sensor_reset_plant_sensors = false;

i2c_error_t sensor_init_opt3001q1(void){
    i2c_error_t error = I2C_NO_ERROR;
    
#if N_SENSOR_OPT3001Q1 > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint16_t i;
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Initialising OPT3001-Q1.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif
    
    sensor_reset_local_id();
    for(i = 0; i < N_SENSOR_OPT3001Q1; i++){
        if((opt3001q1_config[i].general.status == SENSOR_STATUS_ERROR) ||
            (opt3001q1_config[i].general.status == SENSOR_STATUS_ACTIVE)){
            continue;
        }
        
        opt3001q1_config[i].general.local_id = sensor_get_local_id();
        opt3001q1_config[i].general.global_id = CAN_DATA_CMD_OPT3001Q1;
        
        tmp = opt3001q1_init_sensor(&opt3001q1_config[i]);
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
#endif
    
    return error;
}

i2c_error_t sensor_init_apds9301(void){
    i2c_error_t error = I2C_NO_ERROR;
    
#if N_SENSOR_APDS9301 > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint16_t i;
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Initialising APDS9301.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif
    
    sensor_reset_local_id();
    for(i = 0; i < N_SENSOR_APDS9301; i++){
        if((apds9301_config[i].general.status == SENSOR_STATUS_ERROR) ||
            (apds9301_config[i].general.status == SENSOR_STATUS_ACTIVE)){
            continue;
        }
        
        apds9301_config[i].general.local_id = sensor_get_local_id();
        apds9301_config[i].general.global_id = CAN_DATA_CMD_APDS9301;
        
        tmp = apds9301_init_sensor(&apds9301_config[i]);
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
#endif
    
    return error;
}

i2c_error_t sensor_init_sht35(void){
    i2c_error_t error = I2C_NO_ERROR;
    
#if N_SENSOR_SHT35 > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint16_t i;
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Initialising SHT35.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif
    
    sensor_reset_local_id();
    for(i = 0; i < N_SENSOR_SHT35; i++){
        if((sht35_config[i].general.status == SENSOR_STATUS_ERROR) ||
            (sht35_config[i].general.status == SENSOR_STATUS_ACTIVE)){
            continue;
        }
        
        sht35_config[i].general.local_id = sensor_get_local_id();
        sht35_config[i].general.global_id = CAN_DATA_CMD_SHT35;
        
        sht35_config[i].repeatability = S_SHT35_HIGH_REPEATABILIBTY;
        sht35_config[i].clock = S_SHT35_ENABLE_CLOCK_STRETCHING;
        sht35_config[i].rate = S_SHT35_MPS_NONE;
        sht35_config[i].periodicity = S_SHT35_SINGLE_SHOT;
        
        tmp = sht35_init_sensor(&sht35_config[i]);
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
#endif
    
    return error;
}

i2c_error_t sensor_init_bh1721fvc(void){
    i2c_error_t error = I2C_NO_ERROR;
    
#if N_SENSOR_BH1721FVC > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint16_t i;
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Initialising BH1721FVC.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif
    
    sensor_reset_local_id();
    for(i = 0; i < N_SENSOR_BH1721FVC; i++){
        if((bh1721fvc_config[i].general.status == SENSOR_STATUS_ERROR) ||
            (bh1721fvc_config[i].general.status == SENSOR_STATUS_ACTIVE)){
            continue;
        }
        
        bh1721fvc_config[i].general.local_id = sensor_get_local_id();
        bh1721fvc_config[i].general.global_id = CAN_DATA_CMD_BH1721FVC;
        
        tmp = bh1721fvc_init_sensor(&bh1721fvc_config[i]);
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
#endif
    
    return error;
}

i2c_error_t sensor_init_apds9306(void){
    i2c_error_t error = I2C_NO_ERROR;
    
#if N_SENSOR_APDS9306 > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint16_t i;
    
    sensor_reset_local_id();
    for(i = 0; i < N_SENSOR_APDS9306; i++){
        apds9306_config[i].general.local_id = sensor_get_local_id();
        apds9306_config[i].general.global_id = CAN_DATA_CMD_APDS9306;
        
        apds9306_config[i].gain = apds9306_als_gain_1;
        apds9306_config[i].meas_resolution = apds9306_als_resolution_20bit;
        apds9306_config[i].meas_rate =  apds9306_als_meas_rate_500ms;
        
        tmp = apds9306_init_sensor(&apds9306_config[i]);
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
#endif
    
    return error;
}

void sensor_init_licor(){
    size_t i;
    
    sensor_reset_local_id();
    for(i = 0; i < N_SENSOR_LICOR; i++){
        licor_config[i].general.local_id = sensor_get_local_id();
        licor_config[i].general.global_id = CAN_DATA_CMD_LICOR;
        
        licor_init(&licor_config[i]);
    }
}

i2c_error_t sensor_init_sylvatica(void){
    i2c_error_t error = I2C_NO_ERROR;
    
#if N_SYLVATICA_SENSORS > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint8_t i = 0, j;
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Initialising sylvatica.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif

    sensor_reset_local_id();
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){
        for(j = 0; j < SYLVATICA_N_CHANNELS; j++){
            sylvatica_config[i].channels[j].local_id = sensor_get_local_id();
        }
        for(j = 0; j < SYLVATICA_N_CHANNELS; j++){
            sylvatica_config[i].raw_channels[j].local_id = sensor_get_local_id();
        }
        sylvatica_config[i].general.global_id = CAN_DATA_CMD_SYLVATICA;
        sylvatica_config[i].general.local_id = sylvatica_config[i].channels[0].local_id;
    }
    
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){        
        tmp = sylvatica_sensor_init(&sylvatica_config[i]);
        
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
    
    
    
#endif
    
    return error;
}

i2c_error_t sensor_init_planalta(void){
    i2c_error_t error = I2C_NO_ERROR;
#if N_PLANALTA_SENSORS > 0
    i2c_error_t tmp = I2C_NO_ERROR;
    uint8_t i = 0, j;
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Initialising planalta.");
        uart_print(print_buffer, strlen(print_buffer));
    #endif

    sensor_reset_local_id();
    
    // I2C messages
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        for(j = 0; j < PLANALTA_N_ADC_CHANNELS; j++){  
            planalta_config[i].channels[j].local_id = sensor_get_local_id();
        }
        planalta_config[i].general.global_id = CAN_DATA_CMD_PLANALTA;
        planalta_config[i].general.local_id = planalta_config[i].channels[0].local_id;
    }
    
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        tmp = planalta_sensor_init(&planalta_config[i]);
        
        if(tmp != I2C_NO_ERROR){
            error = tmp;
        }
    }
#endif
    
    return error;
}

void sensors_reset_environmental(void){
    // TODO: this should use the macro defined in dicio.c
    _RD8 = 0;
    delay_us(100);
    _RD8 = 1;
}

bool sensors_init_environmental(void) {
    bool init_status = true;
    
    if(sensor_init_opt3001q1() != I2C_NO_ERROR){
        init_status = false;
    }
    if(sensor_init_apds9301() != I2C_NO_ERROR){
        init_status = false;
    }
    if(sensor_init_sht35() != I2C_NO_ERROR){
        init_status = false;
    }
    if(sensor_init_apds9306() != I2C_NO_ERROR){
        init_status = false;
    }
    
    // there is no interaction between the board and this sensor
    sensor_init_licor();
    
    return init_status;
}

void sensors_reset_plant(void){
    // TODO: this should use the macro defined in dicio.c
    _RC14 = 0;
    delay_us(100);
    _RC14 = 1;
}

bool sensors_init_plant(void) {
    bool init_status = true;
    if(sensor_init_sylvatica() != I2C_NO_ERROR){
        init_status = false;
    }
    if(sensor_init_planalta() != I2C_NO_ERROR){
        init_status = false;
    }
    
    return init_status;
}

void sensors_init(void){
    sensors_reset_environmental();
    sensors_init_environmental();
    
    i2c_add_reset_callback(I2C2_BUS, sensors_reset_environmental, 
            sensors_init_environmental);
    
    sensors_init_plant();
    
    i2c_add_reset_callback(I2C1_BUS, sensors_reset_plant, sensors_init_plant);
    
    if(controller_can_address == 0){
        uart_init_message(&serial_meas_trigger,
            SERIAL_MEAS_TRIGGER_CMD,
            controller_address,
            CAN_HEADER(CAN_INFO_CMD_MEASUREMENT_START, 0),            
            NULL,
            0);
    }
    
    dicio_set_sensor_callback(sensor_callback);
    
    // report about all sensor status to host
    sensor_status_report();
    
}

void sensor_apds9301_cb(void){
#if N_SENSOR_APDS9301 > 0
    uint16_t i;
    
    for(i = 0; i < N_SENSOR_APDS9301; i++){
        if(apds9301_config[i].general.status == SENSOR_STATUS_ACTIVE){
            if(i2c_check_message_sent(&apds9301_config[i].m0)) {
                i2c_reset_message(&apds9301_config[i].m0, 1);
            i2c_queue_message(&apds9301_config[i].m0);
            } else {
                sprintf(print_buffer, "APDS9301 %x not fully processed.", apds9301_config[i].general.global_id);
                uart_print(print_buffer, strlen(print_buffer));
                apds9301_config[i].general.elog.n_errors++;
            }
            
            
        }
    }
#endif
}

void sensor_apds9306_cb(void){
    uint16_t i;
    
    for(i = 0; i < N_SENSOR_APDS9306; i++){
        if(apds9306_config[i].general.status == SENSOR_STATUS_ACTIVE){
            if(i2c_check_message_sent(&apds9306_config[i].m_read_setup)) {
                i2c_reset_message(&apds9306_config[i].m_read_setup, 1);
                i2c_queue_message(&apds9306_config[i].m_read_setup);
            } else {
                sprintf(print_buffer, "APDS9306 %x not fully processed.", apds9306_config[i].general.global_id);
                uart_print(print_buffer, strlen(print_buffer));
                apds9306_config[i].general.elog.n_errors++;
            }
        }
    }
}

void sensor_bh1721fvc_cb(void){
#if N_SENSOR_BH1721FVC > 0
    uint16_t i;
    
    for(i = 0; i < N_SENSOR_BH1721FVC; i++){
        if(bh1721fvc_config[i].general.status == SENSOR_STATUS_ACTIVE){
            if(i2c_check_message_sent(&bh1721fvc_config[i].m_read_setup)) {
                i2c_reset_message(&bh1721fvc_config[i].m, 1);
                i2c_queue_message(&bh1721fvc_config[i].m);
            } else {
                sprintf(print_buffer, "BH1721FVC %x not fully processed.", bh1721fvc_config[i].general.global_id);
                uart_print(print_buffer, strlen(print_buffer));
                bh1721fvc_config[i].general.elog.n_errors++;
            }
        }
    }
#endif
}

void sensor_opt3001q1_cb(void){
#if N_SENSOR_OPT3001Q1 > 0
    uint16_t i;
    
    for(i = 0; i < N_SENSOR_OPT3001Q1; i++){
        if(opt3001q1_config[i].general.status == SENSOR_STATUS_ACTIVE){
            if(i2c_check_message_sent(&opt3001q1_config[i].m_control)) {
                i2c_reset_message(&opt3001q1_config[i].m_control, 1);
                i2c_queue_message(&opt3001q1_config[i].m_control);
            } else {
                sprintf(print_buffer, "OPT3001Q1 %x not fully processed.", opt3001q1_config[i].general.global_id);
                uart_print(print_buffer, strlen(print_buffer));
                opt3001q1_config[i].general.elog.n_errors++;
            }
        }
    }
#endif
}

void sensor_sht35_cb(void){
    uint16_t i;
    
    for(i = 0; i < N_SENSOR_SHT35; i++){
        if(sht35_config[i].general.status == SENSOR_STATUS_ACTIVE){
            if(i2c_check_message_sent(&sht35_config[i].m_read) && 
                    i2c_check_message_sent(&sht35_config[i].m_config)) {
                i2c_reset_message(&sht35_config[i].m_read, 1);
                i2c_queue_message(&sht35_config[i].m_read);
                i2c_reset_message(&sht35_config[i].m_config, 1);
                i2c_queue_message(&sht35_config[i].m_config);
            } else {
                sprintf(print_buffer, "SHT35 %x not fully processed.", sht35_config[i].general.global_id);
                uart_print(print_buffer, strlen(print_buffer));
                sht35_config[i].general.elog.n_errors++;
            }

            
        }
    }
}

void sensor_environment_callback(void){

#if N_SENSOR_APDS9301 > 0  
    sensor_apds9301_cb();
#endif
    
#if N_SENSOR_APDS9306 > 0      
    sensor_apds9306_cb();
#endif

#if N_SENSOR_BH1721FVC > 0
    sensor_bh1721fvc_cb();
#endif

#if N_SENSOR_OPT3001Q1 > 0
    sensor_opt3001q1_cb();
#endif

#if N_SENSOR_SHT35 > 0
    sensor_sht35_cb();
#endif
}

bool sensors_environment_reset_check(void){
    size_t i;
    
    #if N_SENSOR_OPT3001Q1 > 0
    for(i = 0; i < N_SENSOR_OPT3001Q1; i++){
        if(opt3001q1_config[i].general.elog.n_errors >= SENSOR_ERROR_TH){
            return true;
        }
    }
    #endif
    #if N_SENSOR_APDS9301 > 0
    for(i = 0; i < N_SENSOR_APDS9301; i++){
        if(apds9301_config[i].general.elog.n_errors >= SENSOR_ERROR_TH){
            return true;
        }
    }
    #endif
    #if N_SENSOR_SHT35 > 0
    for(i = 0; i < N_SENSOR_SHT35; i++){
        if(sht35_config[i].general.elog.n_errors >= SENSOR_ERROR_TH){
            return true;
        }
    }
    #endif
    #if N_SENSOR_APDS9306 > 0
    for(i = 0; i < N_SENSOR_APDS9306; i++){
        if(apds9306_config[i].general.elog.n_errors >= SENSOR_ERROR_TH){
            return true;
        }
    }
    #endif

    return false;
}

bool sensors_plant_reset_check(void) {
    size_t i;
    
    #if N_SYLVATICA_SENSORS > 0
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){
        if(sylvatica_config[i].general.elog.n_errors >= SENSOR_ERROR_TH){
            return true;
        }
    }
    #endif
    #if N_PLANALTA_SENSORS > 0
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        if(planalta_config[i].general.elog.n_errors >= SENSOR_ERROR_TH){
            return true;
        }
    }
    #endif

    return false;
}

void sensors_error_check(void){
    sensor_reset_env_sensors = sensors_environment_reset_check();
    sensor_reset_plant_sensors = sensors_plant_reset_check();
    sensor_error_detected = sensor_error_detected || sensor_reset_env_sensors;
} 

void sensors_error_recover(void){
    size_t i;
    i2c_error_t error;
    
    if(!sensor_error_detected){
        return;
    }
    
    #ifdef ENABLE_DEBUG        
        sprintf(print_buffer, "Sensor readout error occurred. Resetting...");
        uart_print(print_buffer, strlen(print_buffer));
    #endif 

    delay_ms(100);

    asm("RESET");
        
    // make sure I2C queue is empty
    i2c_empty_queue();

    if(sensor_reset_env_sensors){
        sensor_reset_env_sensors = false;
        
        #ifdef ENABLE_DEBUG        
            sprintf(print_buffer, "Resetting environmental sensors...");
            uart_print(print_buffer, strlen(print_buffer));
        #endif

        sensors_reset_environmental();
        
        #if N_SENSOR_OPT3001Q1 > 0
        for(i = 0; i < N_SENSOR_OPT3001Q1; i++){
            error = opt3001q1_init_sensor(&opt3001q1_config[i]);
            if(error != I2C_NO_ERROR){
                opt3001q1_config[i].general.status = SENSOR_STATUS_ERROR;

            }
            opt3001q1_config[i].general.elog.n_errors = 0;
        }
        #endif

        #if N_SENSOR_APDS9301 > 0
        for(i = 0; i < N_SENSOR_APDS9301; i++){
            error = apds9301_init_sensor(&apds9301_config[i]);
            if(error != I2C_NO_ERROR){
                apds9301_config[i].general.status = SENSOR_STATUS_ERROR;
            }
            apds9301_config[i].general.elog.n_errors = 0;
        }
        #endif

        #if N_SENSOR_SHT35 > 0
        for(i = 0; i < N_SENSOR_SHT35; i++){
            error = sht35_init_sensor(&sht35_config[i]);
            if(error != I2C_NO_ERROR){
                sht35_config[i].general.status = SENSOR_STATUS_ERROR;
            }
            sht35_config[i].general.elog.n_errors = 0;
        }
        #endif


        #if N_SENSOR_APDS9306 > 0
        for(i = 0; i < N_SENSOR_APDS9306; i++){
            error = apds9306_init_sensor(&apds9306_config[i]);
            if(error != I2C_NO_ERROR){
                apds9306_config[i].general.status = SENSOR_STATUS_ERROR;
            }
            apds9306_config[i].general.elog.n_errors = 0;
        }
        #endif
    }
    
    if(sensor_reset_plant_sensors){
        sensor_reset_plant_sensors = false;
        
        #ifdef ENABLE_DEBUG        
            sprintf(print_buffer, "Resetting plant sensors...");
            uart_print(print_buffer, strlen(print_buffer));
        #endif

        dicio_stop_clock_sync();
        delay_ms(100);
        dicio_start_clock_sync();
        delay_ms(5000);
        
        // check if there are sensors with errors
        #if N_SYLVATICA_SENSORS > 0
        for(i = 0; i < N_SYLVATICA_SENSORS; i++){
            // initialise sensor anew
            error = sylvatica_sensor_init(&sylvatica_config[i]);
            if(error != I2C_NO_ERROR){
                sylvatica_config[i].general.status = SENSOR_STATUS_ERROR;
            } else {
                sylvatica_sensor_start(&sylvatica_config[i]);
            }
            
            sylvatica_config[i].general.elog.n_errors = 0;   
        }
        #endif
        #if N_PLANALTA_SENSORS > 0
        for(i = 0; i < N_PLANALTA_SENSORS; i++){
            // initialise sensor anew
            error = planalta_sensor_init(&planalta_config[i]);
            if(error != I2C_NO_ERROR){
                planalta_config[i].general.status = SENSOR_STATUS_ERROR;
            } else {
                planalta_sensor_start(&planalta_config[i]);
            }

            planalta_config[i].general.elog.n_errors = 0;    
        }
        #endif
    }
    
    sensor_status_report();
    
    sensor_error_detected = false;
}


void sensor_callback(void){
    static uint8_t interrupt_counter = 0;
    size_t i, j;
    
    interrupt_counter++;
    
    if(((interrupt_counter % 10) == 0) && (controller_address == 0)){
        uart_reset_message(&serial_meas_trigger);
        uart_queue_message(&serial_meas_trigger);
    }
    
    if(sensor_error_detected){
        return;
    }
    
    switch(interrupt_counter){
        case 10:
            sensor_licor_all_low();
            sensor_environment_callback();
            sensors_error_check();
            
            break;
        case 20:
            sensor_environment_callback();
            sensors_error_check();
            
            break;
        case 30:
            sensor_licor_all_high();
            sensor_environment_callback();
            sensors_error_check();
            
            interrupt_counter = 0;
            
            break;
        default:
            break;
    }
    
    #if N_SYLVATICA_SENSORS > 0
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){
        if((interrupt_counter % 10) == (sylvatica_config[i].sample_time % 10)){
            if(sylvatica_config[i].sample_time >= 10){
                sylvatica_config[i].sample_time -= 10;
                
#ifdef ENABLE_DEBUG
                        sprintf(print_buffer, "Corrected Sylvatica %x sample time to %x.", i, sylvatica_config[i].sample_time);
                        uart_print(print_buffer, strlen(print_buffer));
#endif
                
                        
                sylvatica_sensor_read(&sylvatica_config[i]);
                continue;
            }
            

            if(sylvatica_config[i].general.status == SENSOR_STATUS_ACTIVE){
                sylvatica_sensor_read(&sylvatica_config[i]);
                
                // check if all channels produced values, otherwise change sample moment
                for(j = 0; j < SYLVATICA_N_CHANNELS; j++){
                    if(sylvatica_config[i].channels[j].m_read.data[0] == 0){
                        sylvatica_config[i].sample_time += 21;  
                        break;
                    }
                }
            }
        }
    }
    #endif
      
#if N_PLANALTA_SENSORS > 0
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        if((interrupt_counter % 10) == (planalta_config[i].sample_time % 10)){
            // check if value updated in the previous cycle, if so, skip this readout
            if(planalta_config[i].sample_time >= 10){
                planalta_config[i].sample_time -= 10;
                
#ifdef ENABLE_DEBUG
                        sprintf(print_buffer, "Corrected Planalta %x sample time to %x.", i, planalta_config[i].sample_time);
                        uart_print(print_buffer, strlen(print_buffer));
#endif
                
                planalta_sensor_read(&planalta_config[i]);
                
                continue;
            }
        
            if(planalta_config[i].general.status == SENSOR_STATUS_ACTIVE){
                planalta_sensor_read(&planalta_config[i]);
                
                // check if all channels produced values
                for(j = 0; j < PLANALTA_N_ADC_CHANNELS; j++){
                    if(planalta_config[i].channels[j].m_read.data[0] == 0){
                        planalta_config[i].sample_time += 21;
                        break;
                    }
                }
            }
        }
#endif
    }
}

void sensors_start(void){
    uint16_t i;
    
#if N_SYLVATICA_SENSORS > 0
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){
        sylvatica_sensor_start(&sylvatica_config[i]);
    }
#endif
    
#if N_PLANALTA_SENSORS > 0
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        planalta_sensor_start(&planalta_config[i]);
    }
#endif
}

void sensor_licor_all_low(){
    size_t i;
    
    for(i = 0; i < N_SENSOR_LICOR; i++){
        licor_output_low(&licor_config[i]);
    }
}

void sensor_licor_all_high(){
    size_t i;
    
    for(i = 0; i < N_SENSOR_LICOR; i++){
        licor_output_high(&licor_config[i]);
    }
}

#define DICIO_SENSOR_STATUS_LOG_MESSAGE 3
void send_sensor_status(sensor_general_config_t* config){
    uint8_t data[DICIO_SENSOR_STATUS_LOG_MESSAGE];
    
    data[0] = config->global_id;
    data[1] = config->local_id;
    data[2] = config->status;
    
    dicio_send_message(SERIAL_SENSOR_STATUS_CMD, 
            CAN_HEADER(CAN_MSG_SENSOR_STATUS, 0), data, 
            DICIO_SENSOR_STATUS_LOG_MESSAGE);
}

void sensor_status_report(void){
    
    size_t i;
    
    #if N_SYLVATICA_SENSORS > 0
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){
        send_sensor_status(&sylvatica_config[i].general);
    }

#endif

#if N_PLANALTA_SENSORS > 0
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        send_sensor_status(&planalta_config[i].general);
    }
#endif

#if N_SENSOR_APDS9301 > 0
    for(i = 0; i < N_SENSOR_APDS9301; i++){
        send_sensor_status(&apds9301_config[i].general);
    }
#endif

#if N_SENSOR_SHT35 > 0
    for(i = 0; i < N_SENSOR_SHT35; i++){
        send_sensor_status(&sht35_config[i].general);
    }
#endif

#if N_SENSOR_BH1721FVC > 0
    for(i = 0; i < N_SENSOR_BH1721FVC; i++){
        send_sensor_status(&bh1721fvc_config[i].general);
    }
#endif

#if N_SENSOR_OPT3001Q1 > 0
    for(i = 0; i < N_SENSOR_OPT3001Q1; i++){
        send_sensor_status(&opt3001q1_config[i].general);
    }
#endif

#if N_SENSOR_APDS9306 > 0
    for(i = 0; i < N_SENSOR_APDS9306; i++){
        send_sensor_status(&apds9306_config[i].general);
    }
#endif

#if N_SENSOR_LICOR > 0
    for(i = 0; i < N_SENSOR_LICOR; i++){
        send_sensor_status(&licor_config[i].general);
    }
#endif
}

void sensors_data_init(void){
    size_t i;
    
    #if N_SYLVATICA_SENSORS > 0
    for(i = 0; i < N_SYLVATICA_SENSORS; i++){
        sylvatica_config[i] = sensor_sylvatica_config_default;
        
        if(i == 0){
            sylvatica_config[i].general.address = I2C_ADDRESS_0_SENSOR_SYLVATICA;
        } else if(i == 1){
            sylvatica_config[i].general.address = I2C_ADDRESS_1_SENSOR_SYLVATICA;
        } else {
            report_error("No config found.");
        }
    }
    #endif

    #if N_PLANALTA_SENSORS > 0
    for(i = 0; i < N_PLANALTA_SENSORS; i++){
        planalta_config[i] = sensor_planalta_config_default;
        
        if(i > 0){
            report_error("No config found.");
        }
    }
    #endif

    #if N_SENSOR_APDS9301 > 0
    for(i = 0; i < N_SENSOR_APDS9301; i++){
        apds9301_config[i] = sensor_apds9301_config_default;
        
        if(i > 0){
            report_error("No config found.");
        }
    }
    #endif

    #if N_SENSOR_SHT35 > 0
    for(i = 0; i < N_SENSOR_SHT35; i++){
        sht35_config[i] = sensor_sht35_config_default;
                
        if(i > 0){
            report_error("No config found.");
        }
    }
    #endif

    #if N_SENSOR_BH1721FVC > 0
    #error("There was no configuration set for BH1721FVC.")
    #endif

    #if N_SENSOR_OPT3001Q1 > 0
    #error("There was no configuration set for OPT3001Q1.")
    #endif

    #if N_SENSOR_APDS9306 > 0
    for(i = 0; i < N_SENSOR_APDS9306; i++){
        apds9306_config[i] = sensor_apds9306_config_default;
                
        if(i > 0){
            report_error("No config found.");
        }
    }
    #endif

#if N_SENSOR_LICOR > 0
    for(i = 0; i < N_SENSOR_LICOR; i++){
        licor_config[i] = sensor_licor_config_default;
                
        if(i > 0){
            report_error("No config found.");
        }
    }
#endif

}
