#include <xc.h>
#include <i2c.h>
#include <stddef.h>
#include <string.h>
#include <uart.h>
#include <device_configuration.h>
#include <utilities.h>
#include <p33EP512MC806.h>

i2c_mstatus_t (*i2c_transciever_controller)(void);
i2c_mstatus_t (*i2c_transciever_reset)(void);
//i2c_error_status_t i2c_error = I2C_NO_ERROR;

volatile uint8_t transfer_done = 0;

volatile uint8_t i2c_transfer;

volatile i2c_bus_status_t i2c1_bus_status = I2C_BUS_DISABLED; 
volatile i2c_bus_status_t i2c2_bus_status = I2C_BUS_DISABLED; 

i2c_reset_sensor_t i2c_bus_reset[N_I2C_DEVICE_RESET_CALLBACKS];


i2c_message_t* i2c_message_queue [I2C_MESSAGE_BUFFER_LENGTH];
volatile uint16_t i2c_queue_idx = 0;
volatile uint16_t i2c_queue_valid = 0;
volatile uint16_t n_i2c_queued_messages = 0;

volatile uint8_t i2c_transfer_status = 0;

pin_t i2c1_scl_pin, i2c2_scl_pin;
pin_t i2c1_sda_pin, i2c2_sda_pin;

i2c_message_t* i2c_slave_mr_sw_message;

uint8_t n_slave_write_transfers = 0;

volatile uint8_t i2c_slave_continue = 0;

volatile uint8_t stop_detected = 0, start_detected = 0;

volatile i2c_slave_state_t i2c1_slave_state = I2C_SLAVE_STATE_IDLE;
volatile i2c_slave_state_t i2c_slave2_state = I2C_SLAVE_STATE_IDLE;


uint16_t n_i2c1_errors, n_i2c2_errors;

i2c_message_t* m = NULL;

extern void (*i2c_slave_mw_sr_callback) (i2c_message_t*);
extern void (*i2c_slave_mr_sw_callback) (i2c_message_t*);
extern volatile i2c_callback_status_t callback_status;

extern i2c_message_t i2c_slave_mw_sr_message;
extern uint8_t i2c_slave_mw_sr_data[I2C_W_MESSAGE_BUFFER_LENGTH];

void i2c_add_reset_callback(i2c_bus_t i2c_bus, void (*reset)(void), bool (*init)(void)){
    static size_t n_i2c_callbacks = 0;
    
    if(n_i2c_callbacks < N_I2C_DEVICE_RESET_CALLBACKS){
        i2c_bus_reset[n_i2c_callbacks].reset = reset;
        i2c_bus_reset[n_i2c_callbacks].init = init;
        i2c_bus_reset[n_i2c_callbacks].i2c_bus = i2c_bus;
    }
    n_i2c_callbacks++;
}

void i2c_reset_callback_init(void){
    // make sure this function is only executed once!
    static bool init_done = false;
    
    if(init_done == false){
        uint16_t i;
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            i2c_bus_reset[i].reset = NULL;
            i2c_bus_reset[i].init = NULL;
            i2c_bus_reset[i].i2c_bus = I2C1_BUS;
        }
        init_done = true;
    }
}

void dummy_i2c_transciever_reset(i2c_message_t* m){
}

void i2c_dummy_callback(i2c_message_t* m){
}

void i2c1_init_master(i2c_config_t* config) {
    uint16_t frequency;
    
    i2c_reset_callback_init();
    
#ifdef __LOG__
    uart_wait();
    sprintf(print_buffer, "Initialised I2C module 1.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    transfer_done = 1;
    i2c_transfer = 0;
    
    I2C1CONbits.A10M = 0; // 7-bit slave address
    I2C1CONbits.SCLREL = 1; // release clock

    frequency = FCY/(2*FREQUENCY_SCL) - DELAY_I2C*(FCY/2) - 2;
    if(frequency>0x1FF){
        frequency = 0x1FF; // max allowed value for this register
    }
    I2C1BRG = frequency;
    
    I2C1ADD = 0x0; // no slave address, this is I2C master
    I2C1MSK = 0x0; // disable address masking for this bit position
    
    I2C1CONbits.I2CEN = 1; // enable I2C module and configure pins as serial port pins
    IEC1bits.MI2C1IE = 1; // enable I2C interrupt
    IFS1bits.MI2C1IF = 0; // clear I2C interrupt flag
    
    i2c1_bus_status = I2C_BUS_ENABLED;
    
    T2CONbits.TON = 0;
    T2CONbits.TCS = 0; // use internal instruction cycle as clock source
    T2CONbits.TGATE = 0; // disable gated timer
    T2CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR2 = 0; // clear timer register
    PR2 = I2C_TIMER_PERIOD - 1; // set period of ADC_SAMPLE_FREQUENCY
    _T2IF = 0; // clear interrupt flag
    _T2IE = 1; // enable interrupt
    
    T3CONbits.TON = 0;
    T3CONbits.TCS = 0; // use internal instruction cycle as clock source
    T3CONbits.TGATE = 0; // disable gated timer
    T3CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR3 = 0; // clear timer register
    PR3 = 0xffff;
    _T3IF = 0; // clear interrupt flag
    _T3IE = 1; // enable interrupt
    
    T3CONbits.TON = 1;
}



void __attribute__ ( (interrupt, no_auto_psv) ) _MI2C1Interrupt( void ){
    transfer_done = 1;
    _MI2C1IF = 0;
}

void __attribute__ ( (interrupt, no_auto_psv) ) _MI2C2Interrupt( void ){
    transfer_done = 1;
    _MI2C2IF = 0;
}

inline void i2c_auto_queue_message(i2c_message_t* message){
    if((message->address & 0x01) == 0){
        switch(message->i2c_bus){
            case I2C1_BUS:
                message->controller = i2c1_write_controller;
                break;
            case I2C2_BUS:
                message->controller = i2c2_write_controller;
                break;
            default:
                report_error("I2C: I2C module not supported.");
                break;
        }
    } else {
        switch(message->i2c_bus) {
            case I2C1_BUS:
                message->controller = i2c1_read_controller;
                break;
            case I2C2_BUS:
                message->controller = i2c2_read_controller;
                break;
            default:
                report_error("I2C: I2C module not supported.");
                break;
        }
    }
    
    i2c_queue_message(message);
}

inline void i2c_queue_message(i2c_message_t* message){
    if(n_i2c_queued_messages == I2C_MESSAGE_BUFFER_LENGTH){
        message->status = I2C_MESSAGE_CANCELED;
        message->error = I2C_QUEUE_FULL;
        #ifdef __DEBUG__
            uart_wait();
            sprintf(print_buffer, "I2C queue full.");
            uart_print(print_buffer, strlen(print_buffer));
        #endif
    } else {
        if(message->n_attempts < 1){
            message->status = I2C_MESSAGE_HANDLED;
            message->error = I2C_ZERO_ATTEMPTS;
        } else {
            message->status = I2C_MESSAGE_QUEUED;
            i2c_message_queue[i2c_queue_valid] = message;
            i2c_queue_valid = (i2c_queue_valid+1) % I2C_MESSAGE_BUFFER_LENGTH;
            message->error = I2C_NO_ERROR;
            n_i2c_queued_messages++;
            #ifdef __DEBUG__
                uart_wait();
                sprintf(print_buffer, "I2C added message to %x on bus %x.", message->address, message->i2c_bus);
                uart_print(print_buffer, strlen(print_buffer));
            #endif
        }
    }
}


uint16_t reset_i2c1_bus(void){
    uint16_t n_reads = 0, data = 0, i;
    
    // disable I2C1 interrupt
    _MI2C1IE = 0;
    
    // disable I2C1 module
    I2C1CONbits.I2CEN = 0;
    delay_us(100);
    
    // SCL line is blocked by slave, so hard reset is needed
    if(GET_BIT(i2c1_scl_pin.port_r, i2c1_scl_pin.n) == 0){
        return MAX_N_I2C_RST_READS;
    }
    
    // try to get 
    while((GET_BIT(i2c1_sda_pin.port_r, i2c1_sda_pin.n) == 0) && (n_reads < MAX_N_I2C_RST_READS)){
        I2C1CONbits.I2CEN = 1;
        delay_us(100);
        
        I2C1CONbits.RCEN = 1;
        i = 0;
        while((I2C1CONbits.RCEN == 1) && (i < 0xffff)) {i++;};
        data = I2C1TRN;
        n_reads++;
        
        I2C1CONbits.PEN = 1;
        delay_us(100);
        
        I2C1CONbits.I2CEN = 0;
        delay_us(100);
    }
    
    // enable I2C1 module
    I2C1CONbits.I2CEN = 1;
    delay_us(100);
    
    // send stop condition
    I2C1CONbits.PEN = 1;
    
    // re-enable I2C interrupts
    _MI2C1IF = 0;
    _MI2C1IE = 1;
    
    return n_reads;
}

uint8_t reset_i2c2_bus(void){
    uint16_t n_reads = 0, data = 0, i;
    
    // disable I2C2 interrupt
    _MI2C2IE = 0;
    
    // disable I2C2 module
    I2C2CONbits.I2CEN = 0;
    delay_us(100);
    
    // SCL line is blocked by slave, so hard reset is needed
    if(GET_BIT(i2c2_scl_pin.port_r, i2c2_scl_pin.n) == 0){
        return MAX_N_I2C_RST_READS;
    }
    
    // try to get 
    while((GET_BIT(i2c2_sda_pin.port_r, i2c2_sda_pin.n) == 0) && (n_reads < MAX_N_I2C_RST_READS)){
        I2C2CONbits.I2CEN = 1;
        delay_us(100);
        
        I2C2CONbits.RCEN = 1;
        i = 0;
        while((I2C2CONbits.RCEN == 1) && (i < 0xffff)) {i++;};
        data = I2C2TRN;
        n_reads++;
        
        I2C2CONbits.PEN = 1;
        delay_us(100);
        
        I2C2CONbits.I2CEN = 0;
        delay_us(100);
    }
    
    // enable I2C1 module
    I2C2CONbits.I2CEN = 1;
    delay_us(100);
    
    // send stop condition
    I2C2CONbits.PEN = 1;
    
    // re-enable I2C interrupts
    _MI2C2IF = 0;
    _MI2C2IE = 1;
    
    return n_reads;
}

void disable_i2c1_bus(){
    uint16_t i;
    i2c_message_t* m;
    
    // disable I2C1 module
    I2C1CONbits.I2CEN = 0;
    _MI2C1IE = 0;
    _MI2C1IF = 0;
    
    // cancel all I2C1 messages
    for(i = 0; i < I2C_MESSAGE_BUFFER_LENGTH; i++){
        m = i2c_message_queue[i];
        if((m != NULL) && (m->i2c_bus == I2C1_BUS)){
            switch(m->status){
                case I2C_MESSAGE_TRANSFERRING:
                case I2C_MESSAGE_PROCESSING:
                case I2C_MESSAGE_QUEUED:
                    m->status = I2C_MESSAGE_CANCELED;
                default:
                    break;
            }
        }
    }
}

void disable_i2c2_bus(){
    uint16_t i;
    i2c_message_t* m;
    
    // disable I2C2 module
    I2C2CONbits.I2CEN = 0;
    _MI2C2IE = 0;
    _MI2C2IF = 0;
    
    // cancel all I2C2 messages
    for(i = 0; i < I2C_MESSAGE_BUFFER_LENGTH; i++){
        m = i2c_message_queue[i];
        if((m != NULL) && (m->i2c_bus == I2C2_BUS)){
            switch(m->status){
                case I2C_MESSAGE_TRANSFERRING:
                case I2C_MESSAGE_PROCESSING:
                case I2C_MESSAGE_QUEUED:
                    m->status = I2C_MESSAGE_CANCELED;
                default:
                    break;
            }
        }
    }
}

void __attribute__ ( (__interrupt__, no_auto_psv) ) _T2Interrupt( void ){
    //i2c_message_t* m; 
    
    // release I2C bus
    if(m->i2c_bus == I2C1_BUS){
        I2C1CONbits.SCLREL = 1;
        I2C1CONbits.PEN = 1;
    } else {
        I2C2CONbits.SCLREL = 1;
        I2C2CONbits.PEN = 1;
    }
    
    i2c1_slave_state = I2C_SLAVE_STATE_IDLE;
    transfer_done = 1;
    
    m->status = I2C_MESSAGE_CANCELED;
    m->n_attempts = 0;
    
    // disable timer
    T2CONbits.TON = 0;
    
    _T2IF = 0;
}


void i2c_process_queue(void){
    if(transfer_done == 1){
        if(m != NULL){
            switch(m->status){
                case I2C_MESSAGE_PROCESSING:
                    m->callback(m); // execute callback
                    m->status = I2C_MESSAGE_HANDLED;
#ifdef __DEBUG__
        uart_wait();            
        sprintf(print_buffer, "I2C callback.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
                case I2C_MESSAGE_HANDLED:
                    // check if transfer went OK or max attempts reached
                    if((m->error == I2C_NO_ERROR) || (m->n_attempts <= 0)){
#ifdef __DEBUG__
        uart_wait();
        sprintf(print_buffer, "I2C message handled.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
                        T2CONbits.TON = 0; // disable timer (everything OK!)
                        
                        i2c_queue_idx = (i2c_queue_idx + 1) % I2C_MESSAGE_BUFFER_LENGTH;
                        m = NULL;
                        n_i2c_queued_messages--;
                    } else {
#ifdef __DEBUG__
        uart_wait();
        sprintf(print_buffer, "I2C ERROR, restarting.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
                        m->n_attempts--;
                        m->status = I2C_MESSAGE_TRANSFERRING;
                        m->error = I2C_NO_ERROR;
                        
                        // reset the transfer status (just in case)
                        i2c_transfer_status = 0;
                    }
                    
                    break;
                case I2C_MESSAGE_QUEUED:
                    m->status = I2C_MESSAGE_TRANSFERRING;
                    TMR2 = 0; // clear timer register
                    // start timer: transfer must finish withing approx. 25ms
                    T2CONbits.TON = 1; 
#ifdef __DEBUG__
        uart_wait();
        sprintf(print_buffer, "I2C message queued.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
                case I2C_MESSAGE_TRANSFERRING:
#ifdef __DEBUG__
        uart_wait();
        sprintf(print_buffer, "I2C message transferring.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
                    m->controller(m);
                    break;
                case I2C_MESSAGE_CANCELED:
                    if(m->i2c_bus == I2C1_BUS){
                        I2C1CONbits.PEN = 1;
                    } else {
                        I2C2CONbits.PEN = 1;
                    }
                    
                    if(m->cancelled_callback != NULL){
                        m->cancelled_callback(m);
                    }

                    // move to next message
                    i2c_queue_idx = (i2c_queue_idx + 1) % I2C_MESSAGE_BUFFER_LENGTH;
                    m = NULL;
                    n_i2c_queued_messages--;

                    break;
                default:
                    T2CONbits.TON = 0; // disable timer (everything OK!)
                        
                    i2c_queue_idx = (i2c_queue_idx + 1) % I2C_MESSAGE_BUFFER_LENGTH;
                    m = NULL;
                    n_i2c_queued_messages--;
                    m->status = I2C_MESSAGE_CANCELED;
            }
        } else {
            if(i2c_queue_idx != i2c_queue_valid){
                m = i2c_message_queue[i2c_queue_idx];
#ifdef __DEBUG__
                uart_wait();
                sprintf(print_buffer, "Fetched message from queue: %x on bus %x.", m->address, m->i2c_bus);
                uart_print(print_buffer, strlen(print_buffer));
#endif
            }
        }
    }
}

void i2c1_init(i2c_config_t* config){
    switch(config->status){
        case I2C_STATUS_SLAVE_OFF:
        case I2C_STATUS_SLAVE_ON:
            i2c1_init_slave(config);
            config->status = I2C_STATUS_SLAVE_ON;
            break;
        case I2C_STATUS_MASTER_OFF:
        case I2C_STATUS_MASTER_ON:
            i2c1_init_master(config);
            config->status = I2C_STATUS_MASTER_ON;
            break;
        default:
            break;
    }
    
    n_i2c1_errors = 0;
    
}

void i2c1_init_slave(i2c_config_t* config){
    _SI2C1IF = 0;
    _SI2C1IE = 0;
    
    I2C1CONbits.I2CEN = 0; // disable I2C1 module
    I2C1CONbits.I2CSIDL = 0; // continue operation in idle mode
    I2C1CONbits.IPMIEN = 0; // do not acknowledge all addresses
    I2C1CONbits.A10M = 0; // 7-bit address mode
    I2C1CONbits.DISSLW = 0; // enable slew rate control
    I2C1CONbits.SMEN = 0; // disable SMBus input thresholds
    I2C1CONbits.GCEN = 0; // general call address is disabled
    
    
    #if defined( I2C_CLOCK_STRETCHING )
    I2C1CONbits.STREN = 1;  // enable clock stretch
    I2C1CONbits.SCLREL = 1; // release clock 
    #endif

    I2C1ADD = config->i2c_address;         // 7-bit I2C slave address must be initialised here.
    I2C1MSK = 0; // bit match required for all positions
    
    // init callbacks
    i2c_slave_mw_sr_callback = config->mw_sr_cb;
    i2c_slave_mr_sw_callback = config->mr_sw_cb;
    
    // save pin configuration
    i2c1_scl_pin = config->scl_pin;
    i2c1_sda_pin = config->sda_pin;
    
    I2C1CONbits.I2CEN = 1; // enable I2C1 module
    
    // configure internal data variables
    i2c_slave_mr_sw_message = NULL;
    i2c_init_message(&i2c_slave_mw_sr_message, 0, i2c_slave_mw_sr_data, 0, NULL, 0, NULL, NULL, NULL, 0, I2C1_BUS, NULL);
    n_slave_write_transfers = 0;
    
    // timer for stop condition
    i2c1_init_slave_timer();
    
    // init timer to release SCL 
    T2CONbits.TON = 0;
    T2CONbits.TCS = 0; // use internal instruction cycle as clock source
    T2CONbits.TGATE = 0; // disable gated timer
    T2CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR2 = 0; // clear timer register
    PR2 = I2C_TIMER_PERIOD - 1; // set period of ADC_SAMPLE_FREQUENCY
    _T2IF = 0; // clear interrupt flag
    _T2IE = 1; // enable interrupt
    
    _SI2C1IP = 1;
    
    // clear interrupt flag
    _SI2C1IF = 0;
    // enable interrupt
    _SI2C1IE = 1;
    
    
}

void i2c2_init_slave(i2c_config_t* config){
    _SI2C2IF = 0;
    _SI2C2IE = 0;
    
    I2C2CONbits.I2CEN = 0; // disable I2C2 module
    I2C2CONbits.I2CSIDL = 0; // continue operation in idle mode
    I2C2CONbits.IPMIEN = 0; // do not acknowledge all addresses
    I2C2CONbits.A10M = 0; // 7-bit address mode
    I2C2CONbits.DISSLW = 0; // enable slew rate control
    I2C2CONbits.SMEN = 0; // disable SMBus input thresholds
    I2C2CONbits.GCEN = 0; // general call address is disabled
    
    
    #if defined( I2C_CLOCK_STRETCHING )
    I2C2CONbits.STREN = 1;  // enable clock stretch
    I2C2CONbits.SCLREL = 1; // release clock 
    #endif

    I2C2ADD = config->i2c_address;         // 7-bit I2C slave address must be initialised here.
    I2C2MSK = 0; // bit match required for all positions
    
    // init callbacks
    i2c_slave_mw_sr_callback = config->mw_sr_cb;
    i2c_slave_mr_sw_callback = config->mr_sw_cb;
    
    // save pin configuration
    i2c2_scl_pin = config->scl_pin;
    i2c2_sda_pin = config->sda_pin;
    
    I2C2CONbits.I2CEN = 1; // enable I2C2 module
    
    // configure internal data variables
    i2c_slave_mr_sw_message = NULL;
    i2c_init_message(&i2c_slave_mw_sr_message, 0, i2c_slave_mw_sr_data, 0, NULL, 0, NULL, NULL, NULL, 0, I2C2_BUS, NULL);
    n_slave_write_transfers = 0;
    
    // init timer to release SCL 
    T2CONbits.TON = 0;
    T2CONbits.TCS = 0; // use internal instruction cycle as clock source
    T2CONbits.TGATE = 0; // disable gated timer
    T2CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR2 = 0; // clear timer register
    PR2 = I2C_TIMER_PERIOD - 1; // set period of ADC_SAMPLE_FREQUENCY
    _T2IF = 0; // clear interrupt flag
    _T2IE = 1; // enable interrupt
    
    // clear interrupt flag
    _SI2C2IF = 0;
    // enable interrupt
    _SI2C2IE = 1;
    
}

void i2c2_init(i2c_config_t* config){
    switch(config->status){
        case I2C_STATUS_SLAVE_OFF:
        case I2C_STATUS_SLAVE_ON:
            i2c2_init_slave(config);
            config->status = I2C_STATUS_SLAVE_ON;
            break;
        case I2C_STATUS_MASTER_OFF:
        case I2C_STATUS_MASTER_ON:
            i2c2_init_master(config);
            config->status = I2C_STATUS_MASTER_ON;
            break;
        default:
            break;
    }
    
    n_i2c2_errors = 0;
}

void i2c2_init_master(i2c_config_t* config) {
    uint16_t frequency;
    
    i2c_reset_callback_init();
    
#ifdef __LOG__
    uart_wait();
    sprintf(print_buffer, "Initialised I2C module 2.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    transfer_done = 1;
    i2c_transfer = 0;
    
    I2C2CONbits.A10M = 0; // 7-bit slave address
    I2C2CONbits.SCLREL = 1; // release clock
    
    frequency = FCY/(2*FREQUENCY_SCL) - DELAY_I2C*(FCY/2) - 2;
    if(frequency>0x1FF){
        frequency = 0x1FF; // max allowed value for this register
    }
    I2C2BRG = frequency;
    
    I2C2ADD = 0x0; // no slave address, this is I2C master
    I2C2MSK = 0x0; // disable address masking for this bit position
    
    I2C2CONbits.I2CEN = 1; // enable I2C module and configure pins as serial port pins
    _MI2C2IE = 1; // enable I2C interrupt
    _MI2C2IF = 0; // clear I2C interrupt flag
    
    i2c2_bus_status = I2C_BUS_ENABLED;
    
    T2CONbits.TON = 0;
    T2CONbits.TCS = 0; // use internal instruction cycle as clock source
    T2CONbits.TGATE = 0; // disable gated timer
    T2CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR2 = 0; // clear timer register
    PR2 = I2C_TIMER_PERIOD - 1; // set timeout value
    _T2IF = 0; // clear interrupt flag
    _T2IE = 1; // enable interrupt
    
    /*T3CONbits.TON = 0;
    T3CONbits.TCS = 0; // use internal instruction cycle as clock source
    T3CONbits.TGATE = 0; // disable gated timer
    T3CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR3 = 0; // clear timer register
    PR3 = 0xffff;
    _T3IF = 0; // clear interrupt flag
    _T3IE = 1; // enable interrupt
    
    T3CONbits.TON = 1;*/
}

void i2c_init_message(i2c_message_t* m, uint8_t address, uint8_t* data,
    uint16_t data_length, void (*controller)(i2c_message_t* m),
    int8_t n_attempts, void (*callback)(i2c_message_t* m), 
    void (*cancelled_callback)(i2c_message_t* m),
    uint8_t* processor_data, uint8_t processor_data_length,  i2c_bus_t i2c_bus,
    i2c_message_t* connected_message){
    m->address = address;
    m->data = data;
    m->data_length = data_length;
    m->controller = controller;
    m->status = I2C_MESSAGE_HANDLED;
    m->error = I2C_NO_ERROR;
    m->processor_data = processor_data;
    m->callback = callback;
    m->cancelled_callback = cancelled_callback;
    m->processor_data_length = processor_data_length;
    m->i2c_bus = i2c_bus;
    m->connected_message = connected_message;
    i2c_reset_message(m, n_attempts);
}

void i2c_init_read_message(
    i2c_message_t* m,
    uint8_t* data,
    uint16_t data_length){
        m->data = data;
        m->data_length = data_length;
        m->error = I2C_NO_ERROR;
        m->status = I2C_MESSAGE_READ_READY;
}


void i2c_init_connected_message(i2c_message_t* sm,
        i2c_message_t* mm,
        uint8_t* data,
        uint16_t data_length){
    sm->address = mm->address | 0b1;
    sm->data = data;
    sm->data_length = data_length;
    sm->i2c_bus = mm->i2c_bus;
}

bool i2c_check_message_sent(i2c_message_t* m){
    return (m->status == I2C_MESSAGE_HANDLED) || (m->status == I2C_MESSAGE_CANCELED);
}

void i2c_reset_message(i2c_message_t* m, uint8_t n_attempts){
    m->n_attempts = n_attempts;
    m->status = I2C_MESSAGE_HANDLED;
    m->error = I2C_NO_ERROR;
}

void i2c_set_read_message(i2c_message_t* m){
    i2c_slave_mr_sw_message = m;
    n_slave_write_transfers = 0;
}

void i2c_empty_queue(void){
    while(n_i2c_queued_messages > 0){
        i2c_process_queue();
    }
}

bool i2c_check_address(uint8_t address){
    address &= 0x7C;
    
    switch(address){
        case 0b0:
            return false;
            break;
        case 0b100:
            return false;
            break;
        case 0b1111100:
            return false;
            break;
        case 0b1111000:
            return false;
            break;
        default:
            break;
    }
    return true;
}

void __attribute__ ( (__interrupt__, no_auto_psv) ) _T3Interrupt( void ){
    size_t i;
    uint16_t j;
    uint8_t dummy;
    bool init_status;
    
    if(n_i2c1_errors > I2C_ERROR_TH){
        n_i2c1_errors = 0;
        // try to reset devices using dedicated reset
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            if(i2c_bus_reset[i].reset != NULL){
                if(i2c_bus_reset[i].i2c_bus == I2C1_BUS){
                    i2c_bus_reset[i].reset();
                }
            } else {
                break;
            }
        }
        
        I2C1CONbits.I2CEN = 0;
        // manually generate stop condition
        _ODCG2 = 0;  // configure I2C pins as open drain output
        _ODCG3 = 0; // configure I2C pins as open drain outputs
        _LATG2 = 1;
        _LATG3 = 0;
        delay_ms(1);
        _LATG2 = 1;
        _LATG3 = 1;
        delay_ms(1);
        _ODCG2 = 1;  // configure I2C pins as open drain output
        _ODCG3 = 1; // configure I2C pins as open drain outputs
        
        // initialise devices
        init_status = true;
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            if(i2c_bus_reset[i].reset != NULL){
                if(i2c_bus_reset[i].i2c_bus == I2C1_BUS){
                    init_status = init_status && i2c_bus_reset[i].init();
                }
            } else {
                break;
            }
        }
        
        // if init failed, try to read some bytes to free bus
        if(!init_status){
            _MI2C1IE = 0;
            for(i = 0; i < MAX_N_I2C_RST_READS; i++){
                I2C1CONbits.RCEN = 1;
                j = 0;
                while((I2C1CONbits.RCEN == 1) && (j < 0xffff)){j++; }
                dummy = I2C1TRN;
            }
            I2C1CONbits.PEN = 1;
            delay_us(10);
            _MI2C1IE = 1;
        }
        
        // init again
        init_status = true;
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            if(i2c_bus_reset[i].reset != NULL){
                if(i2c_bus_reset[i].i2c_bus == I2C1_BUS){
                    init_status = init_status && i2c_bus_reset[i].init();
                }
            } else {
                break;
            }
        }
        
        // if init failed, deactivate bus
        if(!init_status){
            // TODO: disable bus
        }
        
        // cancel all messages in I2C queue
        while(n_i2c_queued_messages > 0){
            i2c_message_queue[i2c_queue_idx]->status = I2C_MESSAGE_CANCELED;
            transfer_done = 1;
            i2c_process_queue();
        }
    }
    
    if(n_i2c2_errors > I2C_ERROR_TH){
        n_i2c2_errors = 0;
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            if(i2c_bus_reset[i].reset != NULL){
                if(i2c_bus_reset[i].i2c_bus == I2C2_BUS){
                    i2c_bus_reset[i].reset();
                }
            } else {
                break;
            }
        }
        
        I2C2CONbits.I2CEN = 0;
        // manually generate stop condition
        _ODCF4 = 0;
        _ODCF5 = 0;
        _LATF5 = 1;
        _LATF4 = 0;
        delay_ms(1);
        _LATF4 = 1;
        _LATF5 = 1;
        delay_ms(1);
        _ODCF4 = 1; // configure I2C pins as open drain output
        _ODCF5 = 1; // configure I2C pins as open drain output
        
        
        // initialise devices
        init_status = true;
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            if(i2c_bus_reset[i].reset != NULL){
                if(i2c_bus_reset[i].i2c_bus == I2C2_BUS){
                    init_status = init_status && i2c_bus_reset[i].init();
                }
            } else {
                break;
            }
        }
        
        if(!init_status) {
            _MI2C2IE = 0;
            for(i = 0; i < MAX_N_I2C_RST_READS; i++){
                I2C2CONbits.RCEN = 1;
                j = 0;
                while((I2C2CONbits.RCEN == 1) && (j < 0xffff)){j++; }
                dummy = I2C2TRN;
            }
            I2C2CONbits.PEN = 1;
            delay_us(10);
            _MI2C2IE = 1;
        }
        
        // init again
        init_status = true;
        for(i = 0; i < N_I2C_DEVICE_RESET_CALLBACKS; i++){
            if(i2c_bus_reset[i].reset != NULL){
                if(i2c_bus_reset[i].i2c_bus == I2C2_BUS){
                    init_status = init_status && i2c_bus_reset[i].init();
                }
            } else {
                break;
            }
        }
        
        // cancel all messages in I2C queue
        while(n_i2c_queued_messages > 0){
            i2c_message_queue[i2c_queue_idx]->status = I2C_MESSAGE_CANCELED;
            transfer_done = 1;
            i2c_process_queue();
        }
    }
    
    _T3IF = 0;
}
