#include "actuator_growth_chamber.h"
#include <address.h>
#include <actuator_common.h>

uint8_t gc_uart1_rx_state = 0;
gc_uart_message_t gc_uart1_rx_message;
uint8_t n_gc_uart1_rx_messages = 0;
uint8_t gc_uart1_rx_idx = 0;

uint8_t gc_uart3_rx_state = 0;
gc_uart_message_t gc_uart3_rx_message;
uint8_t n_gc_uart3_rx_messages = 0;
uint8_t gc_uart3_rx_idx = 0;

uint8_t gc_tx_uart_data[ACTUATOR_GC_UART_BUFFER_SIZE+GC_UART_HEADER_SIZE];

uint8_t gc_rx_uart1_data[ACTUATOR_GC_UART_BUFFER_SIZE+GC_UART_HEADER_SIZE];
uint8_t gc_rx_uart3_data[ACTUATOR_GC_UART_BUFFER_SIZE+GC_UART_HEADER_SIZE];

uint8_t gc_tx_uart_queue_data[ACTUATOR_GC_UART_QUEUE_SIZE][ACTUATOR_GC_UART_BUFFER_SIZE];
gc_uart_message_t gc_tx_uart_queue[ACTUATOR_GC_UART_QUEUE_SIZE];
uint16_t gc_tx_uart_n;

uint8_t gc_tx_uart_buffer_idx = 0;
uint8_t gc_tx_uart_send_idx = 0;
uint8_t gc_tx_uart_ongoing = 0;

growth_chamber_config_t gc_config;

void init_growth_chamber_i2c(void){
    void (*controller)(i2c_message_t*);
    
    uart_simple_print("Initialising GC I2C.");
    
    switch(gc_config.i2c_bus){
        case I2C1_BUS:
            controller = i2c1_write_controller;
            break;
        case I2C2_BUS:
            controller = i2c2_write_controller;
            break;
        default:
            report_error("growth chamber: I2C module not supported.");;
            break;
    }
    
    reset_actuator_local_id();
    
    gc_config.general.local_id = get_actuator_local_id();
    gc_config.general.global_id = CAN_DATA_CMD_GROWTH_CHAMBER;
    
    actuator_init_common_config(&gc_config.general, ACTUATOR_GROWTH_CHAMBER_LOG_LENGTH);
    
    i2c_init_message(&gc_config.m_temp, 
            I2C_WRITE_ADDRESS(gc_config.address), 
            gc_config.i2c_temp_data,
            ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH,
            controller,
            3,
            i2c_cb_growth_chamber,
            NULL,
            (uint8_t*) &gc_config,
            0,
            gc_config.i2c_bus,
            NULL);

    i2c_init_message(&gc_config.m_rh, 
            I2C_WRITE_ADDRESS(gc_config.address), 
            gc_config.i2c_rh_data,
            ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH,
            controller,
            3,
            i2c_cb_growth_chamber,
            NULL,
            (uint8_t*) &gc_config,
            0,
            gc_config.i2c_bus,
            NULL);

    // set default values
    parse_gc_i2c_data(ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL);
    parse_gc_i2c_data(ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL);

    i2c_queue_message(&gc_config.m_rh);
    i2c_queue_message(&gc_config.m_temp);
    i2c_empty_queue();
    
    if(gc_config.m_temp.error != I2C_NO_ERROR){
        gc_config.general.status = ACTUATOR_STATE_ERROR;
    } else {
        gc_config.general.status = ACTUATOR_STATE_ACTIVE;
    }
}

void init_growth_chamber(growth_chamber_config_t* config){
#ifdef ENABLE_GC_UART
    gc_uart_message_t m;
#endif
    
    gc_config = *config;
    
    init_growth_chamber_i2c();
    
#ifdef ENABLE_GC_UART
    gc_uart1_init();
    gc_uart3_init();
    gc_uart_init_messages();

    
    // test interfaces
    //do {
        gc_uart_init_message(&m,GC_UART_IF1, GC_UART_CMD_VERSION_CHECK, NULL, 0);
        gc_uart_send(&m);
        delay_ms(100);
        if(!gc_config.uart1_interface_up){
            uart_simple_print("actuator: switch on UART1 not responding.");
        } 
    //} while(!gc_config.uart1_interface_up);
    
    /*do {
        gc_uart_init_message(&m,GC_UART_IF3, GC_UART_CMD_VERSION_CHECK, NULL, 0);
        gc_uart_send(&m);
        delay_ms(100);
        if(!gc_config.uart3_interface_up){
            uart_simple_print("actuator: switch on UART3 not responding.");
        }
    } while(!gc_config.uart3_interface_up);*/
    
    delay_ms(1);
#endif
}

void i2c_cb_growth_chamber(i2c_message_t* m){
    growth_chamber_config_t* config = (growth_chamber_config_t*) m->processor_data;
    
    
    if(m->error != I2C_NO_ERROR){
        send_actuator_error(&config->general, m);
    }
    
    send_actuator_data(&config->general, m->data, m->data_length);
}


void parse_gc_i2c_data(uint8_t channel_n){
    // parse I2C message data
    switch(channel_n){
        case ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL:
          gc_config.m_temp.data[0] = ACTUATOR_GROWTH_CHAMBER_DAC_CH_0;
          gc_config.m_temp.data[1] = (gc_config.temperature & 0xff0) >> 4;
          gc_config.m_temp.data[2] = (gc_config.temperature & 0xf) << 4; 
          break;
        case ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL:
          gc_config.m_rh.data[0] = ACTUATOR_GROWTH_CHAMBER_DAC_CH_1;
          gc_config.m_rh.data[1] = (gc_config.relative_humidity & 0xff0) >> 4;
          gc_config.m_rh.data[2] = (gc_config.relative_humidity & 0xf) << 4; 
          break;
        default:
          report_error("growth chamber: channel not configured.");
          break;
    }
    
    
    
}

void parse_gc_log_data(){
    // parse CAN/UART message data
    gc_config.general.tx_data[0] = (gc_config.temperature >> 8) & 0xff;
    gc_config.general.tx_data[1] = gc_config.temperature & 0xff;
    gc_config.general.tx_data[2] = (gc_config.relative_humidity >> 8) & 0xff;
    gc_config.general.tx_data[3] = gc_config.relative_humidity & 0xff;
}

void gc_i2c_send_temperature(void){
    i2c_reset_message(&gc_config.m_temp, 3);
    
    parse_gc_i2c_data(ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL);
    
    i2c_queue_message(&gc_config.m_temp);
}

void gc_i2c_send_rh(void){
    i2c_reset_message(&gc_config.m_rh, 3);
    
    parse_gc_i2c_data(ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL);
    
    i2c_queue_message(&gc_config.m_rh);
}

void gc_uart_init_message(gc_uart_message_t* m, 
        gc_uart_if_t interface,
        gc_uart_cmd_t command,          
        uint8_t* data,
        uint16_t length){
    m->interface = interface;
    m->command = command;
    m->data = data;
    m->length = length;
    m->status = UART_MSG_NONE;
}

void gc_uart1_init(void){
    
    uart_simple_print("Initialising GC UART1.");
    
    U1MODEbits.STSEL = 0;   // 1-stop bit
    U1MODEbits.PDSEL = 0;   // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0;   // Autobaud Disabled
    U1MODEbits.BRGH = 0; // 16x mode (16 clock cycles per data bit)
    U1MODEbits.RTSMD = 0;    // DTE-DTE mode
    U1MODEbits.URXINV = 0;
    U1BRG = ( (FCY / ACTUATOR_GC_UART_BAUDRATE) / 16 ) - 1;
    
    U1MODEbits.UEN   = 0b00; // do not use flow control
    
    //  Configure UART for DMA transfers
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0;  // Interrupt after one RX character is received
    U1STAbits.TXINV = 0;
    
    // clear error bit, do not enable error interrupt
    _U1EIF = 0;
    _U1EIE = 0;
    
    // only enable RX interrupt
    _U1RXIE = 1;
    _U1TXIE = 0;
    
    // enable UART RX and TX
    U1MODEbits.UARTEN = 1;  // Enable UART
    U1STAbits.UTXEN = 1;    // Enable UART Tx
    
    // wait for at least one baud period to continue
    delay_us(2 * (FCY / ACTUATOR_GC_UART_BAUDRATE));
    
    DMA13CONbits.DIR = 1; // RAM-to-Peripheral
    DMA13CONbits.SIZE = 1; // byte transfer mode
    DMA13CONbits.MODE = 0b01; // One-Shot, Ping-Pong modes disabled
    DMA13CNT = 0;              // number of  DMA requests
    DMA13REQ = DMAREQ_IRQ_U1TX;         // Select UART1 transmitter
    DMA13PAD = (volatile unsigned int) &U1TXREG;
    _DMA13IF = 0;      // Clear DMA Interrupt Flag
    _DMA13IE = 1;      // Enable DMA interrupt
    
    DMA13STAL = (uint16_t) gc_tx_uart_data;
    DMA13STAH = 0x0;
    
    // update interrupt priority 
    _DMA13IP = 7;
    
#ifdef __LOG__
    uart_simple_print("Initialised UART module.");
#endif
    
}

void gc_uart_init_messages(void){
    uint16_t i;
    
    for(i = 0; i < ACTUATOR_GC_UART_QUEUE_SIZE; i++){
        gc_uart_init_message(&gc_tx_uart_queue[i],
                GC_UART_IF1,
                GC_UART_CMD_NONE,
                gc_tx_uart_queue_data[i],
                0);
    }
    
    gc_uart_init_message(&gc_uart1_rx_message, 
            GC_UART_IF1,
            GC_UART_CMD_NONE,         
            gc_rx_uart1_data,
            0);
    
    gc_uart_init_message(&gc_uart3_rx_message, 
            GC_UART_IF3,
            GC_UART_CMD_NONE,         
            gc_rx_uart3_data,
            0);
    
    gc_tx_uart_buffer_idx = 0;
    gc_tx_uart_send_idx = 0;
    gc_tx_uart_ongoing = 0;
    
    gc_tx_uart_n = 0;
}

void gc_uart3_init(void){
    uart_simple_print("Initialising GC UART3.");
    
    U3MODEbits.STSEL = 0;   // 1-stop bit
    U3MODEbits.PDSEL = 0;   // No Parity, 8-data bits
    U3MODEbits.ABAUD = 0;   // Autobaud Disabled
    U3MODEbits.BRGH = 0; // 16x mode (16 clock cycles per data bit)
    U3MODEbits.RTSMD = 0;    // DTE-DTE mode
    U3MODEbits.URXINV = 0;
    U3BRG = ( (FCY / ACTUATOR_GC_UART_BAUDRATE) / 16 ) - 1;
    
    U3MODEbits.UEN   = 0b00; // do not use flow control
    
    //  Configure UART for DMA transfers
    U3STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U3STAbits.UTXISEL1 = 0;
    U3STAbits.URXISEL = 0;  // Interrupt after one RX character is received
    U3STAbits.TXINV = 0;
    
    // clear error bit, do not enable error interrupt
    _U3EIF = 0;
    _U3EIE = 0;
    
    // only enable RX interrupt
    _U3RXIE = 1;
    _U3TXIE = 0;
    
    // enable UART RX and TX
    U3MODEbits.UARTEN = 1;  // Enable UART
    U3STAbits.UTXEN = 1;    // Enable UART Tx
    
    // wait for at least one baud period to continue
    delay_us(2 * (FCY / ACTUATOR_GC_UART_BAUDRATE));
    
    DMA12CONbits.DIR = 1; // RAM-to-Peripheral
    DMA12CONbits.SIZE = 1; // byte transfer mode
    DMA12CONbits.MODE = 0b01; // One-Shot, Ping-Pong modes disabled
    DMA12CNT = 0;              // number of  DMA requests
    DMA12REQ = DMAREQ_IRQ_U3TX;         // Select UART1 transmitter
    DMA12PAD = (volatile unsigned int) &U3TXREG;
    _DMA12IF = 0;      // Clear DMA Interrupt Flag
    _DMA12IE = 1;      // Enable DMA interrupt
    
    DMA12STAL = (uint16_t) gc_tx_uart_data;
    DMA12STAH = 0x0;
    
    // update interrupt priority 
    _DMA12IP = 7;
    
#ifdef __LOG__
    uart_simple_print("Initialised UART module.");
#endif
    
    gc_tx_uart_n = 0;
}


void gc_uart_send(gc_uart_message_t* m){
#ifdef ENABLE_GC_UART
    uint16_t i;
    gc_uart_message_t* buffer_m = &gc_tx_uart_queue[gc_tx_uart_buffer_idx];
    
    if(gc_tx_uart_n == ARRAY_LENGTH(gc_tx_uart_queue)){
        return;
    }
    
    if((buffer_m->status == UART_MSG_TRANSFERRED) 
            || (buffer_m->status == UART_MSG_ERROR)
            || (buffer_m->status == UART_MSG_QUEUED)){
        return;
    }    
    
    buffer_m->interface = m->interface;
    buffer_m->command = m->command;
    buffer_m->length = MIN(m->length, ACTUATOR_GC_UART_BUFFER_SIZE);
    for(i = 0; i < buffer_m->length; i++){
        buffer_m->data[i] = m->data[i];
    }
    buffer_m->status = UART_MSG_QUEUED;
    
    // increase counter, go to next message
    gc_tx_uart_buffer_idx = (gc_tx_uart_buffer_idx+1) % ACTUATOR_GC_UART_QUEUE_SIZE;
    gc_tx_uart_n++;
    
    // check if DMA need to be manually enabled
    process_gc_uart_queue();
#endif
}


void __attribute__((__interrupt__,no_auto_psv)) _DMA13Interrupt(void){
#ifdef ENABLE_GC_UART
    // finish the current transfer
    gc_tx_uart_queue[gc_tx_uart_send_idx].status = UART_MSG_SENT;
    gc_tx_uart_send_idx = (gc_tx_uart_send_idx + 1) % ACTUATOR_GC_UART_QUEUE_SIZE; 
    
    gc_tx_uart_n--;
    
    // check if another message is available
    gc_tx_uart_ongoing = 0;
#endif
    
    _DMA13IF = 0;   // Clear the DMA0 Interrupt Flag
    
    process_gc_uart_queue();
}

void __attribute__((__interrupt__,no_auto_psv)) _DMA12Interrupt(void){
#ifdef ENABLE_GC_UART
    // finish the current transfer
    gc_tx_uart_queue[gc_tx_uart_send_idx].status = UART_MSG_SENT;
    gc_tx_uart_send_idx = (gc_tx_uart_send_idx + 1) % ACTUATOR_GC_UART_QUEUE_SIZE;    
    
    gc_tx_uart_n--;
    
    // check if another message is available
    gc_tx_uart_ongoing = 0;
#endif
    
    _DMA12IF = 0;   // Clear the DMA0 Interrupt Flag
    
    process_gc_uart_queue();
}

void process_gc_uart_queue(void){
#ifdef ENABLE_GC_UART
    gc_uart_message_t* m;
    
    if((gc_tx_uart_ongoing == 0) && (gc_tx_uart_n > 0)){
        // copy to actual message to transmit
        
        gc_tx_uart_ongoing = 1;
        
        // start transmission of message
        m = &gc_tx_uart_queue[gc_tx_uart_send_idx];
        
        gc_uart_parse_to_buffer(gc_tx_uart_data, m, ARRAY_LENGTH(gc_tx_uart_data));
    
        switch(m->interface){
            case GC_UART_IF1: 
            {
                while(U1STAbits.TRMT == 0);

                m->status = UART_MSG_TRANSFERRED;
                DMA13STAL = (uint16_t) gc_tx_uart_data;
                DMA13STAH = 0x0;
                DMA13CNT = GC_UART_HEADER_SIZE + (m->length) - 1;

                // start transfer 
                DMA13CONbits.CHEN = 1;
                DMA13REQbits.FORCE = 1; 
                
                break;
            }
            
            case GC_UART_IF3:
            {
                while(U3STAbits.TRMT == 0);

                m->status = UART_MSG_TRANSFERRED;
                DMA12STAL = (uint16_t) gc_tx_uart_data;
                DMA12STAH = 0x0;
                DMA12CNT = GC_UART_HEADER_SIZE + (m->length) - 1;

                // start transfer 
                DMA12CONbits.CHEN = 1;
                DMA12REQbits.FORCE = 1; 
                
                break;
            }
            default:
                report_error("Unknown UART interface.");
                break;
        }
        
    }
#endif
}


void gc_uart_parse_to_buffer(uint8_t* data, gc_uart_message_t* m, const size_t max_length){
    #ifdef ENABLE_GC_UART
    uint16_t i;
    
    m->length = MIN(m->length, max_length-3-1);
    
    data[0] = UART_CMD_START;
    data[1] = (uint16_t) m->command;
    data[2] = (uint8_t) m->length;
    for(i = 0; i < m->length; i++){
        data[3+i] = m->data[i];
    }
    data[3+m->length] = UART_CMD_STOP;
#endif
}


void __attribute__ ( (interrupt, no_auto_psv) ) _U1TXInterrupt( void ) {
    _U1TXIF = 0;
}

void __attribute__ ( (interrupt, no_auto_psv) ) _U3TXInterrupt( void ) {
    _U3TXIF = 0;
}

void __attribute__ ( (interrupt, no_auto_psv) ) _U1RXInterrupt( void ) {
#ifdef ENABLE_GC_UART
    register uint8_t rx_value = U1RXREG;
    
    switch(gc_uart1_rx_state){
        case 0:
            if(rx_value==UART_CMD_START){
                gc_uart1_rx_state = 1;
            }
            break;
        case 1:
            gc_uart1_rx_message.command = rx_value;
            gc_uart1_rx_message.status = UART_MSG_TRANSFERRED;
            
            if(rx_value < N_SERIAL_CMD) {
                gc_uart1_rx_state = 2;
            } else {
                gc_uart1_rx_state = 0;
                gc_uart1_rx_message.status = UART_MSG_ERROR;
                n_gc_uart1_rx_messages++;
            }
            
            break;
        case 2:
            gc_uart1_rx_message.length = rx_value;
            if(rx_value > 0){
                gc_uart1_rx_state = 3;
                gc_uart1_rx_idx = 0;
            } else {
                gc_uart1_rx_state = 4;
            }
            
            break;
        case 3:
            // only write data until buffer is full
            if(gc_uart1_rx_idx < PRINT_BUFFER_LENGTH){
                gc_uart1_rx_message.data[gc_uart1_rx_idx] = rx_value;
            } else {
                // continue reading data, but indicate error occurred
                gc_uart1_rx_message.status = UART_MSG_ERROR;
            }
            
            gc_uart1_rx_idx++;
            
            if(gc_uart1_rx_idx == gc_uart1_rx_message.length){
                gc_uart1_rx_state = 4;
            }
            
            break;
        case 4:
            if(rx_value == UART_CMD_STOP){
                if(gc_uart1_rx_message.status != UART_MSG_ERROR){
                    gc_uart1_rx_message.status = UART_MSG_RECEIVED;
                }
            } else {
                gc_uart1_rx_message.status = UART_MSG_ERROR;
            }
            
            // process message here!!
            process_gc_rx_message(&gc_uart1_rx_message);
            
            n_gc_uart1_rx_messages++;
            gc_uart1_rx_state = 0;

            break;
        default:
            gc_uart1_rx_state = 0;
    }
#endif
    _U1RXIF = 0;
}

void __attribute__ ( (interrupt, no_auto_psv) ) _U3RXInterrupt( void ) {
#ifdef ENABLE_GC_UART
    register uint8_t rx_value = U3RXREG;
    
    switch(gc_uart3_rx_state){
        case 0:
            if(rx_value==UART_CMD_START){
                gc_uart3_rx_state = 1;
            }
            break;
        case 1:
            gc_uart3_rx_message.command = rx_value;
            gc_uart3_rx_message.status = UART_MSG_TRANSFERRED;
            
            if(rx_value < N_SERIAL_CMD) {
                gc_uart3_rx_state = 2;
            } else {
                gc_uart3_rx_state = 0;
                gc_uart3_rx_message.status = UART_MSG_ERROR;
                n_gc_uart3_rx_messages++;
            }
            
            break;
        case 2:
            gc_uart3_rx_message.length = rx_value;
            if(rx_value > 0){
                gc_uart3_rx_state = 3;
                gc_uart3_rx_idx = 0;
            } else {
                gc_uart3_rx_state = 4;
            }
            
            break;
        case 3:
            // only write data until buffer is full
            if(gc_uart3_rx_idx < PRINT_BUFFER_LENGTH){
                gc_uart3_rx_message.data[gc_uart3_rx_idx] = rx_value;
            } else {
                // continue reading data, but indicate error occurred
                gc_uart3_rx_message.status = UART_MSG_ERROR;
            }
            
            gc_uart3_rx_idx++;
            
            if(gc_uart3_rx_idx == gc_uart3_rx_message.length){
                gc_uart3_rx_state = 4;
            }
            
            break;
        case 4:
            if(rx_value == UART_CMD_STOP){
                if(gc_uart3_rx_message.status != UART_MSG_ERROR){
                    gc_uart3_rx_message.status = UART_MSG_RECEIVED;
                }
            } else {
                gc_uart3_rx_message.status = UART_MSG_ERROR;
            }
            
            // process message here!!
            process_gc_rx_message(&gc_uart3_rx_message);
            
            n_gc_uart3_rx_messages++;
            gc_uart3_rx_state = 0;

            break;
        default:
            gc_uart3_rx_state = 0;
    }
#endif
    _U3RXIF = 0;
}

void process_gc_rx_message(gc_uart_message_t* m){
#ifdef ENABLE_GC_UART
    switch(m->command){
        case GC_UART_CMD_NONE:
            uart_simple_print("actuator: GC received NONE.");
            break;
        case GC_UART_CMD_SET_RELAY:
             uart_simple_print("actuator: GC received SET RELAY.");
            break;
        case GC_UART_CMD_READ_RELAY:
             uart_simple_print("actuator: GC received READ RELAY.");
            // TODO: forward message to PC
            break;
        case GC_UART_CMD_SET_DAC:
             uart_simple_print("actuator: GC received SET DAC.");
            break;
        case GC_UART_CMD_INFO:
             uart_simple_print("actuator: GC received INFO.");
            // TODO: forward message to PC
            break;
        case GC_UART_CMD_ERR:
             uart_simple_print("actuator: GC received ERR.");
            // TODO: forward message to PC
            break;
        case GC_UART_CMD_VERSION_CHECK:
             uart_simple_print("actuator: GC received VERSION CHECK.");
            switch(m->interface){
                case GC_UART_IF1:
                    gc_config.uart1_interface_up = true;
                    break;
                case GC_UART_IF3:
                    gc_config.uart3_interface_up = true;
                    break;
                default:
                    report_error("GC: unknown UART interface,");
                    break;
            }
            break;
        default:
            uart_simple_print("Received unknown GC UART message.");
            break;
    }
#endif
}


void actuator_gc_callback(void){
    
    static uint16_t timer_value = 0;
    
    if(gc_config.general.status == ACTUATOR_STATE_ACTIVE){
        if(timer_value == 0){
            timer_value = gc_config.period;
            
            gc_i2c_send_temperature();
            gc_i2c_send_rh();
        }

        timer_value--;
    }
}

void gc_update_temperature(uint16_t temperature){
    gc_config.temperature = temperature;
}

void gc_update_rh(uint16_t rh){
    gc_config.relative_humidity = rh;
}
