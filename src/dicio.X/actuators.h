#ifndef __ACTUATORS_H__
#define	__ACTUATORS_H__

#include <xc.h>  

#define N_ACTUATOR_PUMP 1
#define ENABLE_GROWTH_CHAMBER

#ifdef	__cplusplus
extern "C" {
#endif 
    
void actuators_data_init(void);
void actuators_init(void);
void actuators_start(void);

void actuator_callback(void);

#ifdef	__cplusplus
}
#endif

#endif

