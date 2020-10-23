#ifndef __SENSOR_LICOR_H__
#define	__SENSOR_LICOR_H__

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
#include "sensor_common.h"

/**
 * Number of sensors of this sensor type. Other sensors are assumed to be 
 * present only once.
 */

#define SENSOR_LICOR_DATA_LENGTH 0
#define SENSOR_LICOR_GC_PIN_BOARD2 6 


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
   
typedef struct sensor_licor_config_s {
    sensor_general_config_t general;
    pin_t pin;
} sensor_licor_config_t;


void licor_init(sensor_licor_config_t* config);

void licor_output_high(sensor_licor_config_t* config);
void licor_output_low(sensor_licor_config_t* config);




#ifdef	__cplusplus
}
#endif


#endif

