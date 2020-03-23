#include "sensor_apds9306_065.h"

void apds9306_init_sensor(sensor_apds9306_config_t* config){
    switch(config->meas_resolution){
        case apds9306_als_resolution_13bit:
        case apds9306_als_resolution_16bit:
        case apds9306_als_resolution_17bit:
        case apds9306_als_resolution_18bit:
        case apds9306_als_resolution_19bit:
        case apds9306_als_resolution_20bit:
        default:
            break;
    }
    
    sensor_init_common_config(&config->general, SENSOR_APDS3906_CAN_DATA_LENGTH);
}
