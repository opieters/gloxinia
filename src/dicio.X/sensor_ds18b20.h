#ifndef SENSOR_DS18B20_H
#define	SENSOR_DS18B20_H

#include <xc.h>
#include <stdbool.h>
#include <one_wire.h>

#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct {
        one_wire_config_t* config;
        uint16_t data;
    } ds18b20_config_t;
    
    bool ds18b20_start_conversion(ds18b20_config_t* config);

    bool ds18b20_read_result(ds18b20_config_t* config);

    float ds18b20_result_to_float(ds18b20_config_t* config);


#ifdef	__cplusplus
}
#endif

#endif

