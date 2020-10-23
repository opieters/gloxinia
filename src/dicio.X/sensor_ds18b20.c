#include "sensor_ds18b20.h"
#include <utilities.h>

bool ds18b20_start_conversion(ds18b20_config_t* config){
    bool status = one_wire_reset(config->config); 
    one_wire_write_byte(config->config, 0xCC); // Issue skip ROM command (CCh)
    one_wire_write_byte(config->config, 0x44); // Convert T command (44h)
    
    return status;
}

bool ds18b20_read_result(ds18b20_config_t* config){
    uint8_t lsb, msb;
    int16_t n_tests = 500;
    
    while((!one_wire_read(config->config)) && (n_tests > 0)){
        n_tests--;
    }
    one_wire_reset(config->config);
    one_wire_write_byte(config->config, 0xCC); // Issue skip ROM command 
    one_wire_write_byte(config->config, 0xBE); // Read Scratchpad (BEh) - 15 bits
    
    lsb = one_wire_read_byte(config->config);
    msb = one_wire_read_byte(config->config);
    
    one_wire_reset(config->config); // Stop Reading 
    
    config->data = (msb << 8) | lsb;
    
    return true;
}

float ds18b20_result_to_float(ds18b20_config_t* config){    
    return ((int16_t) config->data) / 16;
}
