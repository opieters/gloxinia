#include <utilities.h>
#include <one_wire.h>

void one_wire_high(one_wire_config_t* config){
    CLEAR_BIT(config->pin.lat_r, config->pin.n);
    SET_BIT(config->pin.tris_r, config->pin.n);
}

void one_wire_low(one_wire_config_t* config){
    CLEAR_BIT(config->pin.lat_r, config->pin.n);
    CLEAR_BIT(config->pin.tris_r, config->pin.n);
}

uint8_t one_wire_read(one_wire_config_t* config){
    return GET_BIT(config->pin.port_r, config->pin.n);;
}

uint8_t one_wire_reset(one_wire_config_t* config){
    uint8_t status;
    
    one_wire_low(config);
    delay_us(240);
    delay_us(240);
    one_wire_high(config);
    delay_us(70);
    status = one_wire_read(config);
    delay_us(205);
    delay_us(205);
    one_wire_high(config);
    return status; 
}


void one_wire_write_bit(one_wire_config_t* config, uint8_t b){
    if(b){
        one_wire_low(config);
        delay_us(6);
        one_wire_high(config);
        delay_us(64);
    } else {
        one_wire_low(config);
        delay_us(60);
        one_wire_high(config);
        delay_us(10);
    }
}

uint8_t one_wire_read_bit(one_wire_config_t* config){
    uint8_t out;
    
    one_wire_low(config);
    delay_us(6);
    one_wire_high(config);
    delay_us(9);
    out = one_wire_read(config);
    delay_us(55);
    return out;
}

void one_wire_write_byte(one_wire_config_t* config, uint8_t b){
    int i;
    
    for(i = 0; i < 8; i++){
        one_wire_write_bit(config, b & 0x01); // send LS bit first 
        b = b >> 1;
    }
}

unsigned char one_wire_read_byte(one_wire_config_t* config){
    uint8_t out; 
    int i;
    
    for(i = 0; i < 8; i++){ // read in LS bit first
        out = out >> 1; // get out ready for next bit
        if(one_wire_read_bit(config) & 0x01) // if its a one 
            out = out | 0x80; // place a 1 
    }
    
    return out;
}
