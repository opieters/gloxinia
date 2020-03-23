/*
 * File:   address.c
 * Author: opieters
 *
 * Created on June 28, 2018, 2:52 PM
 */

#include <xc.h>
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
volatile uint8_t controller_address;
   
void address_data_processor(i2c_message_t* m){
    controller_address = (*m).data[0];
}

i2c_mstatus_t set_device_address(void){
    // create I2C message
    i2c_message_t message;
    uint8_t data[1];
    
    message.address = I2C_READ_ADDRESS(I2C_ADDRESS_GPIO_EXPANDER);
    message.controller = default_i2c_read_controller;
    message.n_attempts = 3;
    message.callback = address_data_processor;
    message.data = data;
    message.data_length = 1;
    message.i2c_bus = I2C1_BUS;
    
    // RESET the GPIO expander
    TRISBbits.TRISB8 = 0;
    PORTBbits.RB8 = 1;
    delay_ms(100);
    PORTBbits.RB8 = 0;
    delay_ms(100);
    PORTBbits.RB8 = 1;
    delay_ms(100);
    
    queue_i2c_message(&message);
    while(message.status != I2C_MESSAGE_HANDLED){
        process_i2c_queue(); 
    }
    
    if(message.error == I2C_NO_ERROR) {
#ifdef __LOG__
        sprintf(print_buffer, "Initialised device address to 0x%x.", controller_address);
        uart_print(print_buffer, strlen(print_buffer));
#endif
    } else {
#ifdef __LOG__        
        sprintf(print_buffer, "Failed to initialise device address.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
    }
    
    return message.error;
}
