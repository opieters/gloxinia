#include "sensor_licor.h"
#include <gpio_expander.h>
#include <stddef.h>
#include <can.h>
#include "address.h"
#include <i2c.h>
#include <uart.h>
#include <device_configuration.h>
#include <utilities.h>
#include <adc.h>
#include <dac.h>
#include <spi.h>

void licor_init(sensor_licor_config_t* config){  
    CLEAR_BIT(config->pin.tris_r, config->pin.n); // configure as output
    CLEAR_BIT(config->pin.port_r, config->pin.n); // set pin low
    
    sensor_init_common_config(&config->general, SENSOR_LICOR_DATA_LENGTH);
    
    config->general.status = SENSOR_STATUS_ACTIVE;
}

void licor_output_high(sensor_licor_config_t* config){
    SET_BIT(config->pin.port_r, config->pin.n);
}

void licor_output_low(sensor_licor_config_t* config){
    CLEAR_BIT(config->pin.port_r, config->pin.n);
}