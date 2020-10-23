#include <xc.h>
#include <i2c.h>

void (*i2c_slave_mw_sr_callback) (i2c_message_t*) = i2c_slave_dummy_mw_sr_callback;
void (*i2c_slave_mr_sw_callback) (i2c_message_t*) = i2c_slave_dummy_mr_sw_callback;
volatile i2c_callback_status_t callback_status = I2C_CALLBACK_NONE;

extern i2c_message_t* i2c_slave_mr_sw_message;
i2c_message_t i2c_slave_mw_sr_message;
uint8_t i2c_slave_mw_sr_data[I2C_W_MESSAGE_BUFFER_LENGTH];
extern uint8_t n_slave_write_transfers;

extern volatile uint8_t i2c_slave_continue;

extern volatile uint8_t stop_detected, start_detected ;

extern volatile i2c_slave_state_t i2c1_slave_state;
extern volatile i2c_slave_state_t i2c_slave2_state;


void i2c_slave_dummy_mw_sr_callback(i2c_message_t* m){
    
}
void i2c_slave_dummy_mr_sw_callback (i2c_message_t* m){
    
}

void i2c1_detect_stop(void){
    if((i2c1_slave_state != I2C_SLAVE_STATE_IDLE) && (I2C1STATbits.P == 1)){
        _SI2C1IF = 1;
    }
}

void i2c2_detect_stop(void){
    if((i2c1_slave_state != I2C_SLAVE_STATE_IDLE) && (I2C1STATbits.P == 1)){
        _SI2C1IF = 1;
    }
}

void run_i2c1_slave(void){
    uint8_t dummy_read;
    static uint8_t n_transfers = 0;
   
    
    // determine state
    switch(i2c1_slave_state){
        case I2C_SLAVE_STATE_IDLE:
            if((I2C1STATbits.D_A == 0) && (I2C1STATbits.S == 1)){
                i2c1_slave_state = I2C_SLAVE_STATE_ADDRESS;
                
                T8CONbits.TON = 1;
            } else{
                T8CONbits.TON = 0;
            }
            n_transfers = 0;
            break;
        case I2C_SLAVE_STATE_WRITE:
            if((I2C1STATbits.D_A == 0) && (I2C1STATbits.S == 1) && (I2C1STATbits.RBF == 1)){
                i2c_slave_mr_sw_callback(i2c_slave_mr_sw_message);
                i2c1_slave_state = I2C_SLAVE_STATE_REPEAT_START;
                break;
            }
            if (I2C1STATbits.P == 1){
                i2c_slave_mr_sw_callback(i2c_slave_mr_sw_message);
                i2c1_slave_state = I2C_SLAVE_STATE_STOP;
            }
            break;
        case I2C_SLAVE_STATE_READ:
            if((I2C1STATbits.D_A == 0) && (I2C1STATbits.S == 1) && (I2C1STATbits.RBF == 1)){
                i2c_slave_mw_sr_callback(&i2c_slave_mw_sr_message);
                i2c1_slave_state = I2C_SLAVE_STATE_REPEAT_START;
                break;
            }
            if (I2C1STATbits.P == 1){
                i2c_slave_mw_sr_callback(&i2c_slave_mw_sr_message);
                i2c1_slave_state = I2C_SLAVE_STATE_STOP;
            }
            break;
        case I2C_SLAVE_STATE_NO_MESSAGE:
            if (I2C1STATbits.P == 1){
                i2c1_slave_state = I2C_SLAVE_STATE_STOP;
            }
            break;
        default:
            break;
            
    }
    // execute state
    switch(i2c1_slave_state){
        case I2C_SLAVE_STATE_ADDRESS:
            // read address to clear buffer
            i2c_slave_mw_sr_message.address = I2C1RCV;
            break;
        case I2C_SLAVE_STATE_REPEAT_START:            
            n_transfers = 0;
            
            // read address to clear buffer
            i2c_slave_mw_sr_message.address = I2C1RCV;
            break;
        case I2C_SLAVE_STATE_WRITE:
            if(i2c_slave_mr_sw_message->status == I2C_MESSAGE_READ_READY){
                // message initialised correctly, ready to read
                if(n_transfers < i2c_slave_mr_sw_message->data_length){
                    // read valid message data
                    I2C1TRN = i2c_slave_mr_sw_message->data[n_transfers];
                    n_transfers++;
                    
                    if(n_transfers == i2c_slave_mr_sw_message->data_length){
                        i2c_slave_mr_sw_message->status = I2C_MESSAGE_HANDLED;
                    }
                    
                } else {
                    // too many read attempts, write 0s
                    I2C1TRN = 0;
                    
                    i2c_slave_mr_sw_message->error = I2C_TOO_MANY_READS;
                    
                }
            } else {
                // message not initialised correctly, write 0s
                I2C1TRN = 0;
                
                i2c_slave_mr_sw_message->error = I2C_MSG_NOT_INITIALISED;
            }
            break;
        case I2C_SLAVE_STATE_READ:
            if(n_transfers < I2C_W_MESSAGE_BUFFER_LENGTH){
                i2c_slave_mw_sr_message.data[n_transfers] = I2C1RCV;
                n_transfers++;
                i2c_slave_mw_sr_message.data_length = n_transfers;
                //I2C1CONbits.ACKDT = 1;
            } else {
                i2c_slave_mw_sr_message.error = I2C_BUFFER_FULL;
                
                dummy_read = I2C1RCV;
            }
            break;
        case I2C_SLAVE_STATE_STOP:
            // clear stop bit
            I2C1STATbits.P = 0;
            
            // clear overflow
            // read last received byte
            if(I2C1STATbits.RBF == 1){
                dummy_read = I2C1RCV;
            }
            // clear overflow condition
            if(I2C1STATbits.I2COV == 1){
                I2C1STATbits.I2COV = 0;
            }
            
            // clear overflow condition
            if(I2C1STATbits.IWCOL == 1){
                I2C1STATbits.IWCOL = 0;
            }
            
            if(I2C1STATbits.BCL == 1){
                I2C1STATbits.BCL = 0;
            }
            
            break;
        default:
            break;
    }
    
    // update state (if needed)
    switch(i2c1_slave_state){
        case I2C_SLAVE_STATE_REPEAT_START:
        case I2C_SLAVE_STATE_ADDRESS:
            if(I2C1STATbits.R_W == 1){
                if(i2c_slave_mr_sw_message != NULL){
                    i2c1_slave_state = I2C_SLAVE_STATE_WRITE;
                    if(i2c_slave_mr_sw_message->status == I2C_MESSAGE_READ_READY) {
                        if(n_transfers < i2c_slave_mr_sw_message->data_length){
                            I2C1TRN = i2c_slave_mr_sw_message->data[n_transfers];
                            n_transfers++;
                        } else {
                            I2C1TRN = 0;
                        }
                    } else {
                        i2c_slave_mr_sw_message->error = I2C_MSG_NOT_INITIALISED;
                        
                        I2C1TRN = 0;
                    }
                }
                else {
                    i2c1_slave_state = I2C_SLAVE_STATE_NO_MESSAGE;
                }
            } else {
                i2c1_slave_state = I2C_SLAVE_STATE_READ;
            }
            break;
        case I2C_SLAVE_STATE_STOP:
            i2c1_slave_state = I2C_SLAVE_STATE_IDLE;
            break;
        default:
            break;
    }
    
    I2C1CONbits.SCLREL = 1;
}

void i2c1_init_slave_timer(void){
    T8CONbits.TON = 0;
    T8CONbits.TCS = 0; // use internal instruction cycle as clock source
    T8CONbits.TGATE = 0; // disable gated timer
    T8CONbits.TCKPS = 0b00; // prescaler 1:1
    TMR8 = 0; // clear timer register
    PR8 = FCY / (2*FREQUENCY_SCL) - 1; // set period to get 2403.8 Hz
    _T8IF = 0; // clear interrupt flag
    _T8IE = 1; // enable interrupt
}

void __attribute__ ( (__interrupt__, no_auto_psv) ) _T8Interrupt( void ){
    if((i2c1_slave_state != I2C_SLAVE_STATE_IDLE) && (I2C1STATbits.P == 1)){
        _SI2C1IF = 1;
    }
    _T8IF = 0;
}



// TODO: add support for clock stretching
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C1Interrupt( void ) {
    run_i2c1_slave();

    _SI2C1IF = 0;               //clear I2C1 Slave interrupt flag
}
