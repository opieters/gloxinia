/*
 * File:   address.c
 * Author: opieters
 *
 * Created on June 28, 2018, 2:52 PM
 */

#include <xc.h>
#include <i2c.h>
#include <address.h>
#include <gpio_expander.h>
#include <uart.h>
#include <string.h>
#include <utilities.h>

/**
 * @brief Variable containing the device address.
 * 
 * @details The device address is read by means of the 
 * `i2c_device_address_transceiver` FSM and is NOT set at start-up. An 
 * explicit I2C transfer is required to initialise the address.
 */
volatile uint16_t controller_can_address;
volatile uint8_t controller_address;
   
void address_data_processor(i2c_message_t* m){
    controller_address = ~m->data[0];
    controller_can_address = controller_address << 3;
}

i2c_mstatus_t set_device_address(void){
    // create I2C message
    i2c_message_t message;
    uint8_t data[1];
    
    i2c_init_message(&message, 
            I2C_READ_ADDRESS(I2C_ADDRESS_GPIO_EXPANDER), 
            data,
            sizeof(data) / sizeof(data[0]), 
            i2c1_read_controller,
            3, 
            address_data_processor, 
            NULL,
            NULL, 
            0, 
            I2C1_BUS,
            NULL);
    
    delay_us(100);
    
    i2c_queue_message(&message);
    while((message.status != I2C_MESSAGE_HANDLED) 
            && (message.status != I2C_MESSAGE_CANCELED)){
        i2c_process_queue(); 
    }
    
    return message.error;
}
