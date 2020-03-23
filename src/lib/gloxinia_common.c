#include <gloxinia_common.h>

uint8_t gloxinia_i2c_get_address(gloxinia_config_t* config){
    uint16_t address = SYLVATICA_I2C_BASE_ADDRESS;
    address = address 
            | (GET_BIT(config->address_selection1.port_r, config->address_selection1.n) << 1)
            | GET_BIT(config->address_selection0.port_r, config->address_selection0.n);
    return address;
}