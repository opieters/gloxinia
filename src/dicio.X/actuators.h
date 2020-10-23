#ifndef __ACTUATORS_H__
#define	__ACTUATORS_H__

#include <xc.h>  
#include <uart.h>
#include <stdbool.h>

#define N_ACTUATOR_PUMP 1
#define N_ACTUATOR_RELAY_BOARD 4
#define ACTUATOR_PERIOD (600) // expressed in in 100ms, 16-bit value!

#define ACTUATOR_ERROR_TH      1

#ifdef	__cplusplus
extern "C" {
#endif 
    
    /*
     * Initialises the configuration data vectors of all actuators that support
     * multiple instantiations. 
     */
    void actuators_data_init(void);

    /*
     * Initialise the actuators (calls each of the initialisers)
     */
    void actuators_init(void);

    /*
     * Can be used to activate dedicated actuator timers prior to the actual start
     * of the experiment. This is only exectuted just before the main loop. 
     */
    void actuators_start(void);

    /*
     * Callback runs every 100ms and executes the actuator code
     */
    void actuator_callback(void);
    
    bool process_actuator_serial_command(uart_message_t* m);
    
    void actuators_error_recover(void);
    
    void actuators_error_check(void);
    
#ifdef	__cplusplus
}
#endif

#endif

