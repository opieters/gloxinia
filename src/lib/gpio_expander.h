/* 
 * File: gpio_expander.h  
 * Author: opieters
 * Comments:
 * Revision history: 0.0.1
 */

  
#ifndef __GPIO_EXPANDER_H__
#define	__GPIO_EXPANDER_H__ 

#include <xc.h>
#include <utilities.h>
#include <i2c.h>
    
/**
 * (Write) Address of the GPIO expander used to configure the device address.
 */
#define I2C_ADDRESS_GPIO_EXPANDER 0x27 
#define SENSOR_GPI 15

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
    
    /**
     * @brief Software reset of the GPIO expander used to obtain the device 
     * address
     * 
     * @details Full software reset that implements FSM needed to reset the
     * expander. Needs to be wrapped in an I2C transfer routine. If this fails,
     * the user can fall back to a full hardware reset using the reset pin.
     * 
     * @attention Can reset other sensors using the same bus too! This routine
     * uses the default method of resetting I2C devices. Re-initialisation of
     * all I2C devices on this bus is thus advised.
     * 
     * @return i2c_error_status_t indicating transfer success/failure.
     */
    i2c_error_t reset_gpio_expander(i2c_message_t* m);


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __GPIO_EXPANDER_H__ */


