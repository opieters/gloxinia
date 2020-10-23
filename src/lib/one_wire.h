#ifndef ONE_WIRE_H
#define	ONE_WIRE_H

#include <xc.h>
#include <utilities.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
    typedef struct {
        pin_t pin;
    } one_wire_config_t;

    uint8_t one_wire_reset(one_wire_config_t* config);
    void one_wire_write_bit(one_wire_config_t* config, uint8_t data); 
    uint8_t one_wire_read_bit(one_wire_config_t* config); 
    void one_wire_write_byte(one_wire_config_t* config, uint8_t data); 
    uint8_t one_wire_read_byte(one_wire_config_t* config); 
    uint8_t one_wire_read(one_wire_config_t* config); 
    void one_wire_high(one_wire_config_t* config); 
    void one_wire_low(one_wire_config_t* config);

#ifdef	__cplusplus
}
#endif

#endif	/* ONE_WIRE_H */

