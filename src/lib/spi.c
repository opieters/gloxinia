#include <xc.h>
#include <spi.h>
#include <utilities.h>
#include <uart.h>
#include <stdbool.h>

uint8_t spi1_transfer_status = 0, spi2_transfer_status = 0, spi3_transfer_status = 0;

void spi1_init(void){
#ifdef __DEBUG__
    sprintf(print_buffer, "Initialising SPI 1.");
    uart_print(print_buffer, strlen(print_buffer));
#endif
    
    _SPI1IF = 0; // clear interrupt flag
    _SPI1IE = 0; // disable SPI interrupt
    
    // SPI1CON1 Register Settings
    SPI1CON1bits.MODE16 = 1;  // communication is word-wide (16 bits)
    SPI1CON1bits.MSTEN = 1;   // Master mode enabled
    SPI1CON1bits.DISSCK = 0;  // internal serial clock enabled
    SPI1CON1bits.DISSDO = 0;  // SDO1 pin is controlled by module
    SPI1CON1bits.CKP = 0;     // idle state for clock is low level
    SPI1CON1bits.CKE = 1;
    SPI1CON1bits.SSEN = 0;    // slave select pin is controlled by software
    SPI1CON1bits.PPRE = 0b01;
    delay_ms(100);
    SPI1CON1bits.SPRE = 0b111; //0b111;
   
    //SPI1STATbits.SPISIDL = 0; // continue module operation in Idle mode
    SPI1STATbits.SPIBEC = 0;  // buffer Length = 1 Word
    SPI1STATbits.SPIROV = 0;  // no Receive Overflow has occurred
    SPI1STATbits.SPIEN = 1;   // enable SPI module
    
    _SPI1IF = 0; // clear interrupt flag
    _SPI1IE = 1; // enable SPI interrupt
    
    spi1_transfer_status = 0;
}

void spi3_init(void){
    _SPI3IF = 0; // clear interrupt flag
    _SPI3IE = 0; // disable SPI interrupt
    
    // SPI1CON1 Register Settings
    SPI3CON1bits.MODE16 = 1;  // communication is word-wide (16 bits)
    SPI3CON1bits.MSTEN = 1;   // Master mode enabled
    SPI3CON1bits.DISSCK = 0;  // internal serial clock enabled
    SPI3CON1bits.DISSDO = 0;  // SDO1 pin is controlled by module
    SPI3CON1bits.SMP = 0;     // data is sampled at centre of conversion (falling edge)
    SPI3CON1bits.CKP = 0;     // idle state for clock is low level
    SPI3CON1bits.CKE = 0;     // data changes on transition from idle to active state (rising edge here)
    SPI3CON1bits.SSEN = 0;    // slave select pin is controlled by software
    SPI3CON1bits.PPRE = 0b10;
    delay_ms(100);
    SPI3CON1bits.SPRE = 0b110; //0b111;
    
   
    //SPI1STATbits.SPISIDL = 0; // continue module operation in Idle mode
    SPI3STATbits.SPIBEC = 0;  // buffer Length = 1 Word
    SPI3STATbits.SPIROV = 0;  // no Receive Overflow has occurred
    SPI3STATbits.SPIEN = 1;   // enable SPI module
    
    _SPI3IF = 0; // clear interrupt flag
    _SPI3IE = 1; // enable SPI interrupt
    
    spi3_transfer_status = 0;
}


void spi2_init(void){    
    _SPI2IF = 0; // clear interrupt flag
    _SPI2IE = 0; // disable SPI interrupt
    
    // SPI2CON1 Register Settings
    SPI2CON1bits.MODE16 = 1;  // communication is word-wide (16 bits)
    SPI2CON1bits.MSTEN = 1;   // Master mode enabled
    SPI2CON1bits.DISSCK = 0;  // internal serial clock enabled
    SPI2CON1bits.DISSDO = 0;  // SDO2 pin is controlled by module
    SPI2CON1bits.SMP = 0;     // data is sampled at centre of conversion (falling edge)
    SPI2CON1bits.CKP = 0;     // idle state for clock is low level
    SPI2CON1bits.CKE = 0;     // data changes on transition from idle to active state (rising edge here)
    SPI2CON1bits.SSEN = 0;    // slave select pin is controlled by software
    SPI2CON1bits.PPRE = 0b11;
    delay_ms(100);
    SPI2CON1bits.SPRE = 0b001; //0b111;
    
    SPI2STATbits.SPIBEC = 0;  // buffer Length = 1 Word
    SPI2STATbits.SPIROV = 0;  // no Receive Overflow has occurred
    SPI2STATbits.SPIEN = 1;   // enable SPI module
    
    _SPI2IF = 0; // clear interrupt flag
    _SPI2IE = 1; // enable SPI interrupt
    
    spi2_transfer_status = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void){
    _SPI1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI2Interrupt(void){
    _SPI2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI3Interrupt(void){
    _SPI3IF = 0;
}

void spi1_send_message(spi_message_t* m){
    static uint8_t n_transfers;
    
    while(m->status != SPI_TRANSFER_DONE){
            switch(spi1_transfer_status){
                case 0:
                    if(m->data_length > 0){
                        n_transfers = 0;
                        m->status = SPI_TRANSFER_ONGOING;
                        spi1_transfer_status = 1;
                        CLEAR_BIT(m->cs_pin.lat_r, m->cs_pin.n);
                    } else {
                        spi1_transfer_status = 3;
                    }
                    
                    break;
                case 1:
                    SPI1BUF = m->write_data[n_transfers];
                    spi1_transfer_status = 2;
                    break;
                case 2:
                    while( !SPI1STATbits.SPIRBF );
                    m->read_data[n_transfers] = SPI1BUF;
                    n_transfers++;
                    if(n_transfers == m->data_length){
                        spi1_transfer_status = 3;
                    } else {
                        spi1_transfer_status = 1;
                    }
                    break;
                case 3:
                    SET_BIT(m->cs_pin.lat_r, m->cs_pin.n);
                    m->status = SPI_TRANSFER_DONE;
                    spi1_transfer_status = 0;
                    break;
                default:
                    spi1_transfer_status = 0;
                    break;
            }
    }
}

void spi2_send_message(spi_message_t* m){
    static uint8_t n_transfers;
    
    while(m->status != SPI_TRANSFER_DONE){
        switch(spi2_transfer_status){
            case 0:
                if(m->data_length > 0){
                    n_transfers = 0;
                    m->status = SPI_TRANSFER_ONGOING;
                    spi2_transfer_status = 1;
                    CLEAR_BIT(m->cs_pin.lat_r, m->cs_pin.n);
                } else {
                    spi2_transfer_status = 3;
                }

                break;
            case 1:
                SPI2BUF = m->write_data[n_transfers];
                spi2_transfer_status = 2;
                break;
            case 2:
                while( !SPI2STATbits.SPIRBF );
                m->read_data[n_transfers] = SPI2BUF;
                n_transfers++;
                if(n_transfers == m->data_length){
                    spi2_transfer_status = 3;
                } else {
                    spi2_transfer_status = 1;
                }
                break;
            case 3:
                SET_BIT(m->cs_pin.lat_r, m->cs_pin.n);
                m->status = SPI_TRANSFER_DONE;
                spi2_transfer_status = 0;
                break;
            default:
                spi2_transfer_status = 0;
                break;
        }
    }
}

void spi3_send_message(spi_message_t* m){
    static uint8_t n_transfers;
    
    while(m->status != SPI_TRANSFER_DONE){
            switch(spi3_transfer_status){
                case 0:
                    if(m->data_length > 0){
                        n_transfers = 0;
                        m->status = SPI_TRANSFER_ONGOING;
                        spi3_transfer_status = 1;
                        CLEAR_BIT(m->cs_pin.lat_r, m->cs_pin.n);
                    } else {
                        spi3_transfer_status = 3;
                    }
                    
                    break;
                case 1:
                    SPI3BUF = m->write_data[n_transfers];
                    spi3_transfer_status = 2;
                    break;
                case 2:
                    while( !SPI3STATbits.SPIRBF );
                    m->read_data[n_transfers] = SPI3BUF;
                    n_transfers++;
                    if(n_transfers == m->data_length){
                        spi3_transfer_status = 3;
                    } else {
                        spi3_transfer_status = 1;
                    }
                    break;
                case 3:
                    SET_BIT(m->cs_pin.lat_r, m->cs_pin.n);
                    m->status = SPI_TRANSFER_DONE;
                    spi3_transfer_status = 0;
                    break;
                default:
                    spi3_transfer_status = 0;
                    break;
            }
    }
}


void spi_software_send(pin_t* sck, pin_t* sdo, pin_t* sdi, pin_t* cs, uint8_t* tx_data, uint8_t* rx_data, uint8_t n, uint16_t period){
    uint8_t n_transfers = 0;
    uint8_t tris_sck, tris_sdo, tris_sdi, tris_cs;
    uint8_t lat_sck, lat_sdo, lat_sdi, lat_cs;
    
    // set minimum value of period
    if(period < 2){
        period = 2;
    }
    
    // read current state registers
    tris_sck = GET_BIT(sck->tris_r, sck->n);
    tris_sdo = GET_BIT(sdo->tris_r, sdo->n);
    tris_sdi = GET_BIT(sdi->tris_r, sdi->n);
    tris_cs  = GET_BIT(cs->tris_r, cs->n);
    
    lat_sck = GET_BIT(sck->lat_r, sck->n);
    lat_sdo = GET_BIT(sdo->lat_r, sdo->n);
    lat_sdi = GET_BIT(sdi->lat_r, sdi->n);
    lat_cs  = GET_BIT(cs->lat_r, cs->n);
    
    // update configuration
    CLEAR_BIT(sck->tris_r, sck->n);
    CLEAR_BIT(sdo->tris_r, sdo->n);
    SET_BIT(sdi->tris_r, sdi->n);
    CLEAR_BIT(cs->tris_r, cs->n);
    
    CLEAR_BIT(sck->lat_r, sck->n);
    CLEAR_BIT(sdo->lat_r, sdo->n);
    SET_BIT(cs->lat_r, cs->n);
    
    // SPI transfer
    // set CS pin
    delay_us(4*period);
    CLEAR_BIT(cs->lat_r, cs->n);
    delay_us(period / 2);
    while(n_transfers < n){
        // shift out byte
        uint16_t i;
        uint8_t current_tx_byte = tx_data[n_transfers];
        uint8_t current_rx_byte = 0;
        for(i = 0; i < 8; i++){
            if(GET_BIT(&current_tx_byte, 7-i) == 0){
                CLEAR_BIT(sdo->lat_r, sdo->n);
            } else {
                SET_BIT(sdo->lat_r, sdo->n);
            }
            delay_us(period / 2);
            SET_BIT(sck->lat_r, sck->n);
            current_rx_byte = (current_rx_byte << 1) | GET_BIT(sdi->port_r, sdi->n);
            delay_us(period / 2);
            CLEAR_BIT(sck->lat_r, sck->n);
        }
        rx_data[n_transfers] = current_rx_byte;
        n_transfers++;
    }
    delay_us(period / 2);
    CLEAR_BIT(sdo->lat_r, sdo->n);
    delay_us(period / 2);
    SET_BIT(cs->lat_r, cs->n);
    
    // reset configuration to old state
    if(tris_sck == 0){
        CLEAR_BIT(sck->tris_r, sck->n);
    } else {
        SET_BIT(sck->tris_r, sck->n);
    }
    if(tris_sdo == 0){
        CLEAR_BIT(sdo->tris_r, sdo->n);
    } else {
        SET_BIT(sdo->tris_r, sdo->n);
    }
    if(tris_sdi == 0){
        CLEAR_BIT(sdi->tris_r, sdi->n);
    } else {
        SET_BIT(sdi->tris_r, sdi->n);
    }
    if(tris_cs == 0){
        CLEAR_BIT(cs->tris_r, cs->n);
    } else {
        SET_BIT(cs->tris_r, cs->n);
    }
    
    if(lat_sck == 0){
        CLEAR_BIT(sck->lat_r, sck->n);
    } else {
        SET_BIT(sck->lat_r, sck->n);
    }
    if(lat_sdo == 0){
        CLEAR_BIT(sdo->lat_r, sdo->n);
    } else {
        SET_BIT(sdo->lat_r, sdo->n);
    }
    if(lat_sdi == 0){
        CLEAR_BIT(sdi->lat_r, sdi->n);
    } else {
        SET_BIT(sdi->lat_r, sdi->n);
    }
    if(lat_cs == 0){
        CLEAR_BIT(cs->lat_r, cs->n);
    } else {
        SET_BIT(cs->lat_r, cs->n);
    }
}

spi_message_t* spi_init_message(spi_message_t* m, uint16_t* write_data, uint16_t* read_data, uint8_t length, const pin_t* pin){
    m->cs_pin = (*pin);
    m->write_data = write_data;
    m->read_data = read_data;
    m->data_length = length;
    if(length == 0){
        m->status = SPI_TRANSFER_DONE;
    } else {
        m->status = SPI_TRANSFER_PENDING;
    }
    
    return m;
}

spi_handler_t spi_get_handler(const spi_module_selector_t spi_module){
    spi_handler_t hander;
    
    switch(spi_module){
        case SPI_MODULE_SELECTOR_1:
            hander = spi1_send_message;
            break;
        case SPI_MODULE_SELECTOR_2:
            hander = spi2_send_message;
            break;
        case SPI_MODULE_SELECTOR_3:
            hander = spi3_send_message;
            break;
        default:
            report_error("ADC: unsupported SPI module.");
            break;
    }
    return hander;
}