#ifndef __ACTUATOR_PUMP_H__
#define	__ACTUATOR_PUMP_H__ 

#include <xc.h>
#include <utilities.h>
#include <sensor_common.h>
#include <actuator_common.h>

#define ACTUATOR_PUMP_DATA_LENGTH 1

typedef enum {
    PUMP_STATUS_OFF,
    PUMP_STATUS_ON,
} pump_status_t;

#ifdef	__cplusplus
extern "C" {
#endif
   
    
typedef struct {
    actuator_general_config_t general;
    
    pin_t pin;
    
    uint32_t period; // number of seconds between pump activations
    uint32_t on_time; // number of seconds the pump is on
    uint32_t timer_value;
    
    pump_status_t status;
} pump_config_t;

void init_actuator_pump(pump_config_t* config);


void start_pump(pump_config_t* config); 
void stop_pump(pump_config_t* config);

#ifdef	__cplusplus
}
#endif

#endif
