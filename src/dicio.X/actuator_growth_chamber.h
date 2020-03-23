#ifndef __ACTUATOR_GROWTH_CHAMBER_H__
#define	__ACTUATOR_GROWTH_CHAMBER_H__

#include <xc.h>  
#include <i2c.h>
#include <actuator_common.h>

#define ACTUATOR_GROWTH_CHAMBER_LENGTH 2
#define ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH 3
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_0 0b00010000
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_1 0b00010010
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_2 0b00010100
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_3 0b00010110
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_4 0b00110100
#define ACTUATOR_GROWTH_CHAMBER_DAC_ADDRESS 0b1001110
#define ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL 0
#define ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL 1

#define ACTUATOR_GROWTH_CHAMBER_TEMP_DEFAULT 0x69F
#define ACTUATOR_GROWTH_CHAMBER_RH_DEFAULT 0xA00

#ifdef	__cplusplus
extern "C" {
#endif
    
typedef struct {
    actuator_general_config_t general;
    
    uint8_t address;
    i2c_bus_t i2c_bus; 
    
    uint16_t temperature;
    uint16_t relative_humidity;
    
    i2c_message_t m_temp;
    i2c_message_t m_rh;
    uint8_t i2c_data[ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH];
    
    uint32_t period;
    uint32_t timer_value;
    
} growth_chamber_config_t;

void init_growth_chamber(growth_chamber_config_t* config);

void parse_growth_chamber_data(growth_chamber_config_t* config, uint8_t channel_n);

void growth_chamber_set_temperature(growth_chamber_config_t* config);

void growth_chamber_set_relative_humidity(growth_chamber_config_t* config);

void i2c_cb_growth_chamber(i2c_message_t* m);

#ifdef	__cplusplus
}
#endif

#endif

