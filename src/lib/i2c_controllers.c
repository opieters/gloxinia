#include <xc.h>
#include <i2c.h>
#include <stddef.h>
#include <string.h>
#include <uart.h>
#include <device_configuration.h>
#include <utilities.h>
#include <p33EP512MC806.h>

extern volatile uint8_t i2c_transfer_status;
extern volatile uint8_t transfer_done;

void i2c1_read_controller(i2c_message_t* m){
    static uint8_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            i2c_transfer_status = 1;
            n_transfers = 0;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C1CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for reading
            transfer_done = 0;
            I2C1TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // enable receive mode
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 6;
                I2C1CONbits.PEN = 1;
            } else {
                I2C1CONbits.RCEN = 1;
                i2c_transfer_status = 4;
            }
            break;
        case 4:
            // receive data byte
            transfer_done = 0;
            
            m->data[n_transfers] = I2C1RCV;
            n_transfers++;
            
            if(n_transfers == m->data_length){
                I2C1CONbits.ACKDT = 1; // send NACK
                i2c_transfer_status = 5;  
            } else {
                I2C1CONbits.ACKDT = 0; // send ACK
                i2c_transfer_status = 3;
            }
            I2C1CONbits.ACKEN = 1; // start ACK event
            
            break;
        case 5:
            // send stop event
            transfer_done = 0;
            i2c_transfer_status = 6;
            I2C1CONbits.PEN = 1;
            break;
        case 6:
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            I2C1CONbits.PEN = 1;
            break;
    }
}


void i2c1_write_read_controller(i2c_message_t* m){
    static uint16_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            i2c_transfer_status = 1;
            n_transfers = 0;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C1CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for reading
            transfer_done = 0;
            I2C1TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // transfer data byte
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C1CONbits.PEN = 1;
            } else {
                I2C1TRN = m->data[n_transfers];
                n_transfers++;
                
                if(n_transfers == m->data_length){
                    i2c_transfer_status = 4;
                } else {
                    i2c_transfer_status = 3;
                }
            }
            break;
        case 4:
            // check final ACK and send repeated start event
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C1CONbits.PEN = 1;           
            } else {
                I2C1CONbits.RSEN = 1;
                i2c_transfer_status = 5;
            }
            break;
        case 5:
            // send start byte with address for reading
            transfer_done = 0;
            I2C1TRN = m->connected_message->address;
            i2c_transfer_status = 6;
            n_transfers = 0;
            break;
        case 6:
            // enable receive mode
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C1CONbits.PEN = 1;  
            } else {
                I2C1CONbits.RCEN = 1;
                i2c_transfer_status = 7;      
            }
            break;
        case 7:
            // receive data byte
            transfer_done = 0;
            
            m->connected_message->data[n_transfers] = I2C1RCV;
            n_transfers++;
            
            if(n_transfers == m->connected_message->data_length){
                I2C1CONbits.ACKDT = 1; // send NACK
                i2c_transfer_status = 8;  
            } else {
                I2C1CONbits.ACKDT = 0; // send ACK
                i2c_transfer_status = 6;
            }
            I2C1CONbits.ACKEN = 1; // start ACK event
            
            break;
        case 8:
            // send stop event
            transfer_done = 0;
            i2c_transfer_status = 9;
            I2C1CONbits.PEN = 1;
            break;
        case 9:
            m->status = I2C_MESSAGE_PROCESSING;
            i2c_transfer_status = 0;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            i2c_transfer_status = 0;
            m->status = m->status = I2C_MESSAGE_PROCESSING;
            I2C1CONbits.PEN = 1;
            break;
    }
}


void i2c2_write_read_controller(i2c_message_t* m){
    static uint16_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            i2c_transfer_status = 1;
            n_transfers = 0;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C2CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for reading
            transfer_done = 0;
            I2C2TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // transfer data byte
            transfer_done = 0;
            if(I2C2STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C2CONbits.PEN = 1;
            } else {
                I2C2TRN = m->data[n_transfers];
                n_transfers++;
                
                if(n_transfers == m->data_length){
                    i2c_transfer_status = 4;
                } else {
                    i2c_transfer_status = 3;
                }
            }
            break;
        case 4:
            // check final ACK and send repeated start event
            transfer_done = 0;
            if(I2C2STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C2CONbits.PEN = 1;           
            } else {
                I2C2CONbits.RSEN = 1;
                i2c_transfer_status = 5;
            }
            break;
        case 5:
            // send start byte with address for reading
            transfer_done = 0;
            I2C2TRN = m->connected_message->address;
            i2c_transfer_status = 6;
            n_transfers = 0;
            break;
        case 6:
            // enable receive mode
            transfer_done = 0;
            if(I2C2STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 9;
                I2C2CONbits.PEN = 1;  
            } else {
                I2C2CONbits.RCEN = 1;
                i2c_transfer_status = 7;      
            }
            break;
        case 7:
            // receive data byte
            transfer_done = 0;
            
            m->connected_message->data[n_transfers] = I2C2RCV;
            n_transfers++;
            
            if(n_transfers == m->connected_message->data_length){
                I2C2CONbits.ACKDT = 1; // send NACK
                i2c_transfer_status = 8;  
            } else {
                I2C2CONbits.ACKDT = 0; // send ACK
                i2c_transfer_status = 6;
            }
            I2C2CONbits.ACKEN = 1; // start ACK event
            
            break;
        case 8:
            // send stop event
            transfer_done = 0;
            i2c_transfer_status = 9;
            I2C2CONbits.PEN = 1;
            break;
        case 9:
            m->status = I2C_MESSAGE_PROCESSING;
            i2c_transfer_status = 0;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            i2c_transfer_status = 0;
            m->status = m->status = I2C_MESSAGE_PROCESSING;
            I2C2CONbits.PEN = 1;
            break;
    }
};


void i2c2_read_controller(i2c_message_t* m){
    static uint16_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            i2c_transfer_status = 1;
            n_transfers = 0;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C2CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for reading
            transfer_done = 0;
            I2C2TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // enable receive mode
            transfer_done = 0;
            if(I2C2STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 6;
                I2C2CONbits.PEN = 1;
            } else {
                I2C2CONbits.RCEN = 1;
                i2c_transfer_status = 4;    
            }
            break;
        case 4:
            // receive data byte
            transfer_done = 0;
            
            m->data[n_transfers] = I2C2RCV;
            n_transfers++;
            
            if(n_transfers == m->data_length){
                I2C2CONbits.ACKDT = 1; // send NACK
                i2c_transfer_status = 5;  
            } else {
                I2C2CONbits.ACKDT = 0; // send ACK
                i2c_transfer_status = 3;
            }
            
            I2C2CONbits.ACKEN = 1; // start ACK event        
            
            break;
        case 5:
            // send stop event
            transfer_done = 0;
            i2c_transfer_status = 6;
            I2C2CONbits.PEN = 1;
            break;
        case 6:
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            I2C2CONbits.PEN = 1;
            break;
    }
}


void i2c1_write_controller(i2c_message_t* m){
    static uint16_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            n_transfers = 0;
            i2c_transfer_status = 1;
            break;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C1CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for writing
            transfer_done = 0;
            I2C1TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // transfer data byte
            transfer_done = 0;
            if(I2C1STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 0;
                I2C1CONbits.PEN = 1;
                m->status = I2C_MESSAGE_PROCESSING;
            } else {
                I2C1TRN = m->data[n_transfers];
                n_transfers++;
                
                if(n_transfers == m->data_length){
                    i2c_transfer_status = 4;
                } else {
                    i2c_transfer_status = 3;
                }
            }

            break;
        case 4:
            // send stop event
            transfer_done = 0;
            I2C1CONbits.PEN = 1;
            i2c_transfer_status = 5;
            break;
        case 5:
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            I2C1CONbits.PEN = 1;
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            break;
    }
}


void i2c2_write_controller(i2c_message_t* m){
    static uint16_t n_transfers = 0;
   
    switch(i2c_transfer_status){
        case 0:
            n_transfers = 0;
            i2c_transfer_status = 1;
            break;
        case 1:
            // i2c start condition
            transfer_done = 0;
            I2C2CONbits.SEN = 1;
            i2c_transfer_status = 2;
            break;
        case 2:
            // send start byte with address for writing
            transfer_done = 0;
            I2C2TRN = m->address;
            i2c_transfer_status = 3;
            break;
        case 3:
            // transfer data byte
            transfer_done = 0;
            if(I2C2STATbits.ACKSTAT == 1) {
                m->error = I2C_NO_ACK;
                i2c_transfer_status = 0;
                I2C2CONbits.PEN = 1;
                m->status = I2C_MESSAGE_PROCESSING;
            } else {
                I2C2TRN = m->data[n_transfers];
                n_transfers++;
                
                if(n_transfers == m->data_length ){
                    i2c_transfer_status = 4;
                } else {
                    i2c_transfer_status = 3;
                }
            }

            break;
        case 4:
            // send stop event
            transfer_done = 0;
            I2C2CONbits.PEN = 1;
            i2c_transfer_status = 5;
            break;
        case 5:
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            break;
        default:
            // go back to initial state
            transfer_done = 0;
            I2C2CONbits.PEN = 1;
            i2c_transfer_status = 0;
            m->status = I2C_MESSAGE_PROCESSING;
            break;
    }
}


void i2c_blocking_read_controller(i2c_message_t* m){
    switch(m->i2c_bus) {
        case I2C1_BUS:
            while(m->status != I2C_MESSAGE_HANDLED){
                i2c1_read_controller(m);
            }
            break;
        case I2C2_BUS:
            while(m->status != I2C_MESSAGE_HANDLED){
                i2c2_read_controller(m);
            }
            break;
        default:
            break;
    }
}


void i2c_blocking_write_controller(i2c_message_t* m){
    switch(m->i2c_bus) {
        case I2C1_BUS:
            while(m->status != I2C_MESSAGE_HANDLED){
                i2c1_write_controller(m);
            }
            break;
        case I2C2_BUS:
            while(m->status != I2C_MESSAGE_HANDLED){
                i2c2_write_controller(m);
            }
            break;
        default:
            break;
    }
}
