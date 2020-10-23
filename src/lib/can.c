#include <xc.h>
#include <can.h>
#include <uart.h>
#include <stdio.h>
#include <string.h>
#include <utilities.h>
#include <address.h>

extern char print_buffer[PRINT_BUFFER_LENGTH];

#define NUM_OF_ECAN_BUFFERS 32
#define MAX_CAN_TO_UART_SIZE (2+CAN_MAX_N_BYTES)

uint16_t ecan_message_buffer[NUM_OF_ECAN_BUFFERS][8] __attribute__((aligned(NUM_OF_ECAN_BUFFERS * 16)));

volatile uint8_t received_ecan_message;
volatile uint8_t __init_sensors = 0;

uint8_t uart_can_message_data[UART_MESSAGE_BUFFER_LENGTH][5+CAN_MAX_N_BYTES+2];
uart_message_t uart_can_messages[UART_MESSAGE_BUFFER_LENGTH];
volatile uint8_t uart_can_message_idx = 0;

uart_message_t uart_m;
uint8_t uart_data[CAN_MAX_N_BYTES];

can_module_status_t can_module_status = CAN_MODULE_OFF;

void init_rx_can_message(void){
}

void can_init(void) {
    uint16_t i;
    
#ifdef __LOG__    
    sprintf(print_buffer, "Initialised ECAN module");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    // switch ECAN module to configuration mode
    C1CTRL1bits.REQOP = 4;
    while(C1CTRL1bits.OPMODE != 4); // wait for mode switch
    
    C1CTRL1bits.WIN = 0; // use buffer control and status window
    
    /* Set up the CAN module for 250kbps speed with 10 Tq per bit. */
    C1CFG1bits.SJW = 2; // sync jump with bits to maintain sync with transmitted message
    C1CFG1bits.BRP = 7; // baud rate prescaler: 2xBRP/F_CAN = TQ
    C1CFG2bits.WAKFIL = 0;
    C1CFG2bits.SEG2PH = 2; // phase segment 2 bits in TQ -> 2TQ
    C1CFG2bits.SEG2PHTS = 1; // phase segment 2 time select bit is freely programmable
    C1CFG2bits.SAM = 1; // CAN line bus is sampled 3 time at the same point
    C1CFG2bits.SEG1PH = 4; // phase segment 1 bits -> length is 5 TQ
    C1CFG2bits.PRSEG = 2; // propagation time segment bits -> 3TQ
     
     /* Configure ECAN module buffers */
    C1FCTRLbits.DMABS = 0b110; // 32 buffers in device RAM
    C1FCTRLbits.FSA = 0b01000; // FIFO start address = read buffer RB8
    
    C1CTRL1bits.WIN = 1; // use filter control and status window
    
    // configure all filter mask 0 
    C1FMSKSEL1 = 0;
    C1FMSKSEL2 = 0; 
    
    // configure all filters to store messages in FIFO
    C1BUFPNT1 = 0xFFFF;
    C1BUFPNT2 = 0xFFFF;
    C1BUFPNT3 = 0xFFFF;
    C1BUFPNT4 = 0xFFFF;
    
    if(controller_address != 0){
        // configure mask 0 to ignore last 3 bits bits
        C1RXM0SIDbits.SID = 0x7F8;    

        // configure filter 0 to only accept messages from gateway
        C1RXF0SIDbits.SID = 0;
        
        // configure mask to check for standard and extended (SID) frames
        C1RXM0SIDbits.MIDE = 0;
    } else {
        // configure mask 0 to ignore all
        C1RXM0SIDbits.SID = 0b0;
        C1RXM0SIDbits.EID = 0;
        C1RXM0EIDbits.EID = 0;
        
        // configure mask to check for standard and extended (SID) frames
        C1RXM0SIDbits.MIDE = 0;
    }
    
    C1CTRL1bits.WIN = 0;
    
    // interrupt configuration
    //C1INTEbits.IVRIE = 0; // invalid message interrupt disabled
    //C1INTEbits.WAKIE = 0; // bus wake-up activity interrupt disabled
    //C1INTEbits.ERRIE = 1; // error interrupt enable bit enabled TODO: interrupt service!
    //C1INTEbits.FIFOIE = 0; // FIFO almost full interrupt enable off
    //C1INTEbits.RBOVIE = 0; // buffer overflow interrupt disabled
    //C1INTEbits.RBIE = 1; // RX buffer interrupt enabled
    //C1INTEbits.TBIE = 1; // TX buffer interrupt disabled
    
    // enable ECAN interrupts
    //_C1IE = 1;
        
    // configure first 8 buffers for sending data
    C1TR01CONbits.TXEN0 = 1; // configure buffer 0 for transmission
    C1TR01CONbits.TX0PRI = 3; // assign priority
    C1TR01CONbits.TXEN1 = 1; // configure buffer 1 for transmission
    C1TR01CONbits.TX1PRI = 3; // assign priority
    C1TR23CONbits.TXEN2 = 1; // configure buffer 2 for transmission
    C1TR23CONbits.TX2PRI = 3; // assign priority
    C1TR23CONbits.TXEN3 = 1; // configure buffer 3 for transmission
    C1TR23CONbits.TX3PRI = 3; // assign priority
    C1TR45CONbits.TXEN4 = 1; // configure buffer 4 for transmission
    C1TR45CONbits.TX4PRI = 3; // assign priority
    C1TR45CONbits.TXEN5 = 1; // configure buffer 5 for transmission
    C1TR45CONbits.TX5PRI = 3; // assign priority
    C1TR67CONbits.TXEN6 = 1; // configure buffer 6 for transmission
    C1TR67CONbits.TX6PRI = 3; // assign priority
    C1TR67CONbits.TXEN7 = 1; // configure buffer 7 for transmission
    C1TR67CONbits.TX7PRI = 3; // assign priority
    
    // other 24 buffers are for receiving data
    
    can_init_dma_channel();
    init_rx_can_message();
    
    // switch ECAN module to normal operation mode
    C1CTRL1bits.REQOP = 0b000;
    while(C1CTRL1bits.OPMODE != 0b000); // wait for mode switch
    
    for(i = 0; i < UART_MESSAGE_BUFFER_LENGTH; i++){
        uart_can_messages[i].data = uart_can_message_data[i];
        uart_can_messages[i].length = 0;
        uart_can_messages[i].status = UART_MSG_NONE;
        
        uart_can_message_data[i][0] = UART_CMD_START;
    }
    
    uart_can_message_idx = 0;
    
    can_module_status = CAN_MODULE_ON;
}

void can_disable(void){
    C1CTRL1bits.ABAT = 1; // abort all transmissions
    
    C1CTRL1bits.REQOP = 1; // disable CAN module
    while(C1CTRL1bits.OPMODE != 1); // wait for mode switch
    
    // disable DMA
    DMA2CONbits.CHEN = 0;
    DMA1CONbits.CHEN = 0;
    
    can_module_status = CAN_MODULE_OFF;
}


void can_init_dma_channel(void){
    // configure TX DMA channel
    DMA1CONbits.SIZE = 0x0; // Word Transfer Mode
    DMA1CONbits.DIR = 0x1; // Data Transfer Direction: device RAM to Peripheral
    DMA1CONbits.AMODE = 0x2; // peripheral indirect addressing mode
    DMA1CONbits.MODE = 0x0; // Operating Mode: Continuous, Ping-Pong modes disabled
    DMA1REQ = 0b01000110; // select TX data ready as IRQ
    DMA1CNT = 7; // 8 DMA Transfers per ECAN message
    DMA1PAD = (volatile unsigned int)&C1TXD; // ECAN transmit register
    
    // data source to be transferred
    DMA1STAL = (unsigned int) &ecan_message_buffer;
    DMA1STAH = 0;//(unsigned int) &ecan_message_buffer;
    
    _DMA1IE = 0;  // enable DMA channel interrupt
    _DMA1IF = 0;  // clear DMA interrupt flag
    DMA1CONbits.CHEN = 1; // enable DMA channel
    
    // configure RX DMA channel 
    DMA2CONbits.SIZE = 0; // Word Transfer Mode
    DMA2CONbits.DIR = 0; // Data Transfer Direction: device RAM to Peripheral
    DMA2CONbits.AMODE = 0x2; // peripheral indirect addressing mode
    DMA2CONbits.MODE = 0x0; // Operating Mode: Continuous, Ping-Pong modes disabled
    DMA2REQ = 0b00100010; // select RX data ready as IRQ
    DMA2CNT = 7; // 8 DMA Transfers per ECAN message
    DMA2PAD = (volatile unsigned int)&C1RXD; // ECAN transmit register
    
    // data source to be transferred
    DMA2STAL = (unsigned int) &ecan_message_buffer;
    DMA2STAH = 0;// (unsigned int) &ecan_message_buffer;
    
    _DMA2IE = 1;  // enable DMA channel interrupt
    _DMA2IF = 0;  // clear DMA interrupt flag
    DMA2CONbits.CHEN = 1; // enable DMA channel
}


void __attribute__ ( (interrupt, no_auto_psv) ) _DMA1Interrupt( void ) {
    _DMA1IF = 0;                // Clear the DMA0 Interrupt Flag;
}

void __attribute__ ( (interrupt, no_auto_psv) ) _DMA2Interrupt( void ) {
    received_ecan_message = 1;
#ifdef __DEBUG__
        sprintf(print_buffer, "DMA RX CAN message.");
        uart_print(print_buffer, strlen(print_buffer));
#endif
    _DMA2IF = 0;                // Clear the DMA2 Interrupt Flag;
}

inline can_status_t can_send_message(can_message_t* message, uint8_t can_channel){
    uint16_t i;
    
    if(can_module_status == CAN_MODULE_OFF){
        return CAN_NO_ERROR;
    }
        
    switch(can_channel){
        case 0:
            if(C1TR01CONbits.TXREQ0 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 1:
            if(C1TR01CONbits.TXREQ1 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 2:
            if(C1TR23CONbits.TXREQ2 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 3:
            if(C1TR23CONbits.TXREQ3 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 4:
            if(C1TR45CONbits.TXREQ4 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 5:
            if(C1TR45CONbits.TXREQ5 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 6:
            if(C1TR67CONbits.TXREQ6 == 1){
                return CAN_TX_PENDING;
            }
            break;
        case 7:
            if(C1TR67CONbits.TXREQ7 == 1){
                return CAN_TX_PENDING;
            }
            break;
        default:
            break;
    }
    
    ecan_message_buffer[can_channel][0] = ((message->identifier & 0x7FF) << 2) | (message->extended_frame==0?0b00:0b11) | (message->remote_frame==0?0b00:0b10);
    ecan_message_buffer[can_channel][1] = (message->extended_identifier >> 6) & 0xFFF;
    ecan_message_buffer[can_channel][2] = ((message->extended_identifier << 10) & 0xFC00) | (message->remote_frame==0?0x000:0x200) | (message->data_length & 0xF);
    for(i = 0; i < message->data_length; i++){
        if( (i % 2) == 0){
            ecan_message_buffer[can_channel][3 + i / 2] = message->data[i];
        } else {
            ecan_message_buffer[can_channel][3 + (i - 1) / 2] = ecan_message_buffer[can_channel][3 + (i - 1) / 2] | (((uint16_t) message->data[i]) << 8);
        }
    }
    
    switch(can_channel){
        case 0:
            C1TR01CONbits.TXREQ0 = 1;
            break;
        case 1:
            C1TR01CONbits.TXREQ1 = 1;
            break;
        case 2:
            C1TR23CONbits.TXREQ2 = 1;
            break;
        case 3:
            C1TR23CONbits.TXREQ3 = 1;
            break;
        case 4:
            C1TR45CONbits.TXREQ4 = 1;
            break;
        case 5:
            C1TR45CONbits.TXREQ5 = 1;
            break;
        case 6:
            C1TR67CONbits.TXREQ6 = 1;
            break;
        case 7:
            C1TR67CONbits.TXREQ7 = 1;
            break;
        default:
            break;
    }
    return CAN_NO_ERROR;
}

inline can_status_t can_send_message_any_ch(can_message_t* m){
    static uint16_t i = 0;
    uint16_t n_attempts = 0;
    can_status_t sent_status = CAN_NO_ERROR;
    do {
        sent_status = can_send_message(m, i);
        i = (i+ 1) % 8;
        n_attempts++;
    } while((sent_status != CAN_NO_ERROR) && (n_attempts < 8));
    return sent_status;
}

inline void can_process_node_message(){
    can_message_t can_message;
    uint8_t received_can_data[CAN_MAX_N_BYTES];
    can_message.data = received_can_data;
    
    if((C1RXFUL2 > 0) || (C1RXFUL1 > 0)){

        // read and parse message
        can_parse_message(&can_message, ecan_message_buffer[C1FIFObits.FNRB]);
        
        if(C1FIFObits.FNRB >= 16){
            C1RXFUL2 = C1RXFUL2 & (~(1<<(C1FIFObits.FNRB - 16)));
        } else {
            C1RXFUL1 = C1RXFUL1 & (~(1<<(C1FIFObits.FNRB)));
        }
        
        if (can_message.data_length >= 1){
#ifdef __DEBUG__
            sprintf(print_buffer, "Received CAN message %d.", can_message.data[0]);
            uart_print(print_buffer, strlen(print_buffer));
#endif
            switch(can_message.data[0]){
                case CAN_INFO_MSG_TEXT:
                    break;                    
                case CAN_INFO_MSG_START_MEAS:
#ifdef __LOG__
                    sprintf(print_buffer, "Starting measurements.");
                    uart_print(print_buffer, strlen(print_buffer));
#endif
                    //sensor_setup_mode = 0;
                    // TODO: fix
                    //schedule_i2c_sensors();
                    break;
                case CAN_INFO_MSG_STOP_MEAS:
                    // TODO go into sleep mode?
                    break;
                case CAN_INFO_MSG_RESET: 
                {
                    uint8_t reset_device = 0;
                    if(can_message.data_length == 2){
                        if(can_message.data[1] == controller_address){
                            reset_device = 1;
                        }
                    } else {
                        reset_device = 1;
                    } 
                    if(reset_device){
#ifdef __DEBUG__
                        sprintf(print_buffer, "Resetting...");
                        uart_print(print_buffer, strlen(print_buffer));       
#endif 
                        delay_us(10);
                        asm ("RESET");
                    }
                    break; 
                }
                case CAN_MSG_SENSOR_STATUS:
                    /*if(sensor_setup_mode && (can_message.data_length == 4)){
                        sensor_type_t sensor_type = (sensor_type_t) can_message.data[1];
                        uint8_t sensor_id = can_message.data[2];
                        sensor_state_t sensor_state = (sensor_state_t) can_message.data[3];
                        uint8_t update_status = 0;
                        sensor_state_t* active_sensors_list;
                        
                        switch(sensor_type){
                            case sensor_type_sht35:
                                if(sensor_id < N_SENSOR_SHT35){
                                    // TODO: fix
                                    //active_sensors_list = active_sensors_sht35;
                                    update_status = 1;
                                }
                                break;
                            case sensor_type_bh1721fvc:
                                if(sensor_id < N_SENSOR_BH1721){
                                   //active_sensors_list = active_sensors_bh1721fvc;
                                    update_status = 1;
                                }
                                break;
                            case sensor_type_opt3001q1:
                                if(sensor_id < N_SENSOR_OPT3001_Q1){
                                    //active_sensors_list = active_sensors_opt3001_q1;
                                    update_status = 1;
                                }
                                break;
                            case sensor_type_apds9301:
                                if(sensor_id < N_SENSOR_APDS9301){
                                    //active_sensors_list = active_sensors_apds_9301;
                                    update_status = 1;
                                }
                                break;
                            case sensor_type_lia:
                                if(sensor_id < N_SENSOR_LIA){
                                    //TODO: fix
                                    //active_sensors_list = active_sensors_lia;
                                    update_status = 1;
                                }
                                break;
                            default:
                                break;
                        }
                        
                        if(update_status){
                            switch(sensor_state) {
                                case sensor_inactive:
                                    active_sensors_list[sensor_id] = sensor_inactive;
                                case sensor_idle:
                                    active_sensors_list[sensor_id] = sensor_idle;
                                default:
                                    break;
                            }
                        }
                    }*/
                    break;
                case CAN_INFO_MSG_ACTUATOR_STATUS:
                    // TODO
                    break;
                case CAN_INFO_MSG_INIT_SENSORS:
                    __init_sensors = 1;
                    break;
                case CAN_INFO_MSG_SAMPLE:
                    //sample_sensors = 1;
                    break;
                case CAN_INFO_MSG_INT_ADC:
                    break;
                case CAN_INFO_MSG_INIT_DAC:
                    break;
                case CAN_INFO_MSG_SCHEDULE_I2C:
                    break;
                case CAN_INFO_MSG_HELLO:
#ifdef __LOG__
                    sprintf(print_buffer, "Sending HELLO message.");
                    uart_print(print_buffer, strlen(print_buffer));
#endif
                {
                    can_message_t can_hello;
                    uint8_t hello_data[1] = {CAN_INFO_MSG_HELLO};
                    can_hello.data = hello_data;
                    can_hello.data_length = 1;
                    can_hello.identifier = controller_can_address;
                    can_hello.extended_frame = 0;
                    can_hello.remote_frame = 0;
                    can_send_message_any_ch(&can_hello);
                }
                    break;
                case CAN_INFO_LIA_GAIN_SET:
                    if(can_message.data_length == 4){
                        if(can_message.data[1] == (controller_can_address >> 3)){
                            // TODO
                            //select_gain_pga113(can_message.data[3]);
                        }
                    }
                    break;
                default:
                    break;
            }
        } else {
#ifdef __DEBUG__
            sprintf(print_buffer, "Received CAN message without data.");
            uart_print(print_buffer, strlen(print_buffer));
#endif
        }

        // clear buffer

        if(C1FIFObits.FBP == C1FIFObits.FNRB){
            received_ecan_message = 0;
        }
    }
}


inline void can_parse_message(can_message_t* m, uint16_t* raw_data){
    uint16_t i;
    
    m->identifier = raw_data[0] >> 2;
    if((raw_data[0] & 0x03) == 0b11){
        m->extended_frame = 1;
        m->extended_identifier = (((uint32_t) raw_data[1]) << 6) | (raw_data[2] >> 10);
    } else {
        m->extended_frame = 0;
        m->extended_identifier = 0;
    }
    
    m->data_length = raw_data[2] & 0xF;
    
    if(((raw_data[2] & 0x10) == 0x10)){
        m->remote_frame = 1;
    } else {
        m->remote_frame = 0;
        
        for(i = 0; i < m->data_length; i++){
            if(i % 2 == 0){
                m->data[i] = (uint8_t) raw_data[i / 2 + 3];
            } else {
                m->data[i] = (uint8_t) (raw_data[(i - 1) / 2 + 3] >> 8);
            }
        }
    }

}

inline void parse_can_to_uart_message(can_message_t* can_message, uart_message_t* uart_message) {
    register uint16_t i = 0;
    uint8_t cmd;
    
    cmd = (uint8_t) ((can_message->extended_identifier >> 8) & 0xFF);
    switch(cmd){
        case NO_CAN_CMD:
            break;
        case CAN_DATA_CMD_APDS9301:
        case CAN_DATA_CMD_OPT3001Q1:
        case CAN_DATA_CMD_BH1721FVC:
        case CAN_DATA_CMD_APDS9306:
        case CAN_DATA_CMD_SHT35:
        case CAN_DATA_CMD_SYLVATICA:
        case CAN_DATA_CMD_PLANALTA:
        case CAN_DATA_CMD_LICOR:
            uart_message->command = SERIAL_SENSOR_DATA_CMD;
            break;
        case CAN_DATA_CMD_PUMP:
        case CAN_DATA_CMD_GROWTH_CHAMBER:
        case CAN_CONFIG_CMD_APDS9301:
        case CAN_CONFIG_CMD_OPT3001Q1:
        case CAN_CONFIG_CMD_BH1721FVC:
        case CAN_CONFIG_CMD_APDS9306:
        case CAN_CONFIG_CMD_SHT35:
        case CAN_CONFIG_CMD_SYLVATICA:
        case CAN_CONFIG_CMD_PLANALTA:
        case CAN_CONFIG_CMD_LICOR:
        case CAN_INFO_MSG_TEXT:
        case CAN_INFO_MSG_START_MEAS:
        case CAN_INFO_MSG_STOP_MEAS:
        case CAN_INFO_MSG_RESET:
        case CAN_MSG_SENSOR_STATUS:
            uart_message->command = SERIAL_SENSOR_STATUS_CMD;
            break;
        case CAN_INFO_MSG_ACTUATOR_STATUS:
        case CAN_INFO_MSG_INIT_SENSORS:
        case CAN_INFO_MSG_SAMPLE:
        case CAN_INFO_MSG_HELLO:
            uart_message->command = SERIAL_HELLO_CMD;
            break;
        case CAN_INFO_MSG_INT_ADC:
        case CAN_INFO_MSG_INIT_DAC:
        case CAN_INFO_MSG_SCHEDULE_I2C:
        case CAN_INFO_LIA_GAIN_SET:
        case CAN_MSG_SENSOR_ERROR:
            uart_message->command = SERIAL_SENSOR_ERROR_CMD;
            break;
        case CAN_INFO_CMD_MEASUREMENT_START:
        default:
            uart_message->command = SERIAL_UNKNOWN_CMD;
            break;
    }

    uart_message->id = (uint8_t) (can_message->identifier >> 3);
    uart_message->extended_id = can_message->extended_identifier;
    uart_message->length = can_message->data_length;
    
    for(i = 0; i < can_message->data_length; i++){
        uart_message->data[i] = can_message->data[i];
    }
}


inline void can_process_gateway_message(){
    can_message_t can_message;
    uint8_t received_can_data[CAN_MAX_N_BYTES];
    uart_message_t* m;
    
    can_message.data = received_can_data;
    
    if((C1RXFUL2 > 0) || (C1RXFUL1 > 0)){
        // read and parse message
        can_parse_message(&can_message, ecan_message_buffer[C1FIFObits.FNRB]);
        
        // clear buffer
        if(C1FIFObits.FNRB >= 16){
            C1RXFUL2 = C1RXFUL2 & (~(1<<(C1FIFObits.FNRB - 16)));
        } else {
            C1RXFUL1 = C1RXFUL1 & (~(1<<(C1FIFObits.FNRB)));
        }
        
        m = &uart_can_messages[uart_can_message_idx];
        parse_can_to_uart_message(&can_message, m);
        uart_reset_message(m);
        uart_queue_message(m);
        uart_can_message_idx = (uart_can_message_idx + 1) % UART_MESSAGE_BUFFER_LENGTH; 
        
        if(C1FIFObits.FBP == C1FIFObits.FNRB){
            received_ecan_message = 0;        
        }
        

    }
}



void can_cmd_info_rx(uint8_t cmd, uint8_t* data, uint8_t length){
    if(cmd >= N_CAN_MSG){
        return;
    }
    
    switch(cmd){
        case CAN_INFO_MSG_TEXT:
            if(controller_address == 0){
                size_t i;

                for(i = 0; i < length; i++){
                    uart_m.data[i] = data[i];
                }
                
                uart_init_message(&uart_m, 
                    SERIAL_TEXT_MESSAGE_CMD,
                    controller_address,
                    0,            
                    uart_data,
                    length);
                
                uart_queue_message(&uart_m);
            }
            break;
        case CAN_INFO_MSG_START_MEAS:
            break;
        case CAN_INFO_MSG_STOP_MEAS:
            break;
        case CAN_INFO_MSG_RESET:
            __asm__ volatile ("reset");
            break;
        case CAN_MSG_SENSOR_STATUS:
            if(controller_address == 0){
                size_t i;
                
                for(i = 0; i < length; i++){
                    uart_m.data[i] = data[i];
                }
                uart_init_message(&uart_m, 
                    SERIAL_SENSOR_STATUS_CMD,
                    controller_address,
                    0,            
                    uart_data,
                    length);
                
                uart_queue_message(&uart_m);
            }
            break;
        case CAN_INFO_MSG_ACTUATOR_STATUS:
            if(controller_address == 0){
                size_t i;

                for(i = 0; i < length; i++){
                    uart_m.data[i] = data[i];
                }
                uart_init_message(&uart_m, 
                    SERIAL_ACTUATOR_STATUS,
                    controller_address,
                    0,            
                    uart_data,
                    length);
                
                uart_queue_message(&uart_m);
            }
            break;
        case CAN_INFO_MSG_INIT_SENSORS:
            break;
        default:
            return;
    }
}

void can_cmd_info_tx(uint8_t cmd, uint8_t* data, uint8_t length){
    uint8_t message_data[8];
    uint16_t i;
    
    length = (length > 7)?7:length;
    
    message_data[0] = cmd;
    for(i = 0; i < length; i++){
        message_data[i+1] = data[i];
    }
    
    can_message_t m;
    m.identifier = controller_can_address;
    m.data = message_data;
    m.data_length = length;
    m.remote_frame = 0;
    m.extended_identifier = 0;
    
    can_send_message_any_ch(&m);
}

void can_init_message(can_message_t* m, 
        uint16_t identifier,
        uint8_t remote_frame,
        uint8_t extended_frame,
        uint32_t extended_identifier,
        uint8_t* data,
        uint8_t data_length){
    m->identifier = identifier;
    m->remote_frame = remote_frame;
    m->extended_frame = extended_frame;
    m->extended_identifier = extended_identifier;
    m->data_length = data_length;
    m->data = data;
}