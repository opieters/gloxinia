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
#include "actuator_growth_chamber.h"

//#define USE_GC_BOARD

void licor_init(sensor_licor_config_t* config){  
    CLEAR_BIT(config->pin.tris_r, config->pin.n); // configure as output
    CLEAR_BIT(config->pin.port_r, config->pin.n); // set pin low
    
    sensor_init_common_config(&config->general, SENSOR_LICOR_DATA_LENGTH);
    
    config->general.status = SENSOR_STATUS_ACTIVE;
}

void licor_output_high(sensor_licor_config_t* config){
#ifdef USE_GC_BOARD
    uint8_t m_data[2];
    gc_uart_message_t m;
    
    gc_uart_init_message(&m,
                         GC_UART_IF3,
                         GC_UART_CMD_SET_RELAY,          
                         m_data,
                         ARRAY_LENGTH(m_data));
    
    m_data[0] = SENSOR_LICOR_GC_PIN_BOARD2;
    m_data[1] = 1;
    
    gc_uart_send(&m);
#else
    SET_BIT(config->pin.port_r, config->pin.n);
#endif
    
    sensor_send_data(&config->general.dlog, NULL, SENSOR_LICOR_DATA_LENGTH);
}

void licor_output_low(sensor_licor_config_t* config){
#ifdef USE_GC_BOARD
    uint8_t m_data[2];
    gc_uart_message_t m;
    
    gc_uart_init_message(&m,
                         GC_UART_IF3,
                         GC_UART_CMD_SET_RELAY,          
                         m_data,
                         ARRAY_LENGTH(m_data));
    
    m_data[0] = SENSOR_LICOR_GC_PIN_BOARD2;
    m_data[1] = 0;
    
    gc_uart_send(&m);
#else
    CLEAR_BIT(config->pin.port_r, config->pin.n);
#endif
}