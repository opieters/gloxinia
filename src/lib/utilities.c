/*
 * File:   utilities.c
 * Author: opieters
 *
 * Created on September 19, 2018, 3:30 PM
 */


#include <xc.h>
#include <utilities.h>
#include <uart.h>

char print_buffer[PRINT_BUFFER_LENGTH];

static pin_t blinky_pin;
static pin_t* error_pin = NULL;

static void (*error_loop_fn)(void) = NULL;


void delay_us(uint16_t delay){ //delay x us
    while(delay--) {
        __asm__ volatile ("repeat #63");
        __asm__ volatile ("nop");
    }
}

void delay_ms(uint16_t delay){
    while(delay--) {
        delay_us(1000);
    }
}

void blinky_init(pin_t* pin, short timer_on){
    
    blinky_pin = *pin;
    
    CLEAR_BIT(blinky_pin.tris_r, blinky_pin.n);
    CLEAR_BIT(blinky_pin.port_r, blinky_pin.n);
    
    if(timer_on == 1){
        T1CONbits.TON = 0; 

        T1CONbits.TCS = 0; // use internal instruction cycle as clock source
        T1CONbits.TGATE = 0; // disable gated timer
        T1CONbits.TCKPS = 0b11; // prescaler 1:256
        TMR1 = 0; // clear timer register

        PR1 = 0xFFFF;

        _T1IF = 0; // clear interrupt flag
        _T1IE = 1; // enable interrupt

        T1CONbits.TON = 1;
    }
}

void set_error_pin(pin_t* pin){
    error_pin = pin;
    
    CLEAR_BIT(error_pin->tris_r, error_pin->n);
    CLEAR_BIT(error_pin->port_r, error_pin->n);
}

void set_error_led(void){
    T1CONbits.TON = 0;
    _T1IE = 0; // enable interrupt
    _T1IF = 0; // clear interrupt flag
    
    if(error_pin != NULL) SET_BIT(error_pin->port_r, error_pin->n);
}

void toggle_led(void){
    T1CONbits.TON = 0;
    _T1IE = 0; // enable interrupt
    _T1IF = 0; // clear interrupt flag
    
    TOGGLE_BIT(blinky_pin.lat_r, blinky_pin.n);
}

void __attribute__ ( (__interrupt__, no_auto_psv) ) _T1Interrupt( void ){
    TOGGLE_BIT(blinky_pin.lat_r, blinky_pin.n);
    _T1IF = 0;
}

void set_error_loop_fn(void (*fn)(void)){
    error_loop_fn = fn;
}

void print_error(char* message, unsigned int length){
    set_error_led();
    
    uart_print(message, length);
    
    if(error_loop_fn != NULL){
        while(1){
            error_loop_fn();
        }
    } else {
        while(1);
    }
}
