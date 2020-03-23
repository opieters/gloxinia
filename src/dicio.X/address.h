/* 
 * File: address.h  
 * Author: opieters
 * Comments:
 * Revision history: 0.0.1
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __ADDRESS_H__
#define	__ADDRESS_H__ 

#include <xc.h>
#include <utilities.h>
#include <i2c.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
    //i2c_error_status_t get_device_address(void);
    
    extern uint8_t gateway;
    extern volatile uint16_t controller_can_address;
    extern volatile uint8_t controller_address;
    
    /**
     * @brief Controller function to obtain device address
     * 
     * @details Implements the finite state machine to read the GPIO expander
     * that is connected to a DIP switch to form the device address. Does NOT 
     * perform the actual transfer alone. This function needs to be used with
     * one of the I2C transfer routines and inserted there. Both blocking and 
     * non-blocking routines are supported.
     * The result is stored in the global variable `controller_address`
     * 
     * @return i2c_error_status_t indicating the status of the I2C transfer
     */
    i2c_status_t i2c_device_address_transciever(void);
    
    
    /**
     * @brief Reads the device address.
     * 
     * @details Function that retrieves the device address in a blocking 
     * fashion. Only if the device address is read or and error occurs does this
     * function return. 3 attempts are made to read the device address.
     * 
     * @return i2c_error_status_t indicating transfer success/failure.
     */
    i2c_status_t set_device_address(void);
    


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __ADDRESS_H__ */
