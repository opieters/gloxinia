

#include <dali.h>
#include <utilities.h>

dali_config_t config = {
    .dali_bus_invert = false,
};

volatile uint8_t bit_tx_state = DALI_FIRST_HALF;
volatile uint32_t dali_tx_data = 0;
volatile uint8_t dali_n_tx_bits = 0;

void init_dali(void){
    init_dali_timer();
    
    // set bus to idle state
    if(config.dali_bus_invert){
        CLEAR_PORT_BIT(config.tx_pin);
    } else {
        SET_PORT_BIT(config.tx_pin);
    }
    
    delay_ms(10);
	dali_transmit(DALI_BROADCAST_C, DALI_RESET);
	delay_ms(10);
	dali_transmit(DALI_BROADCAST_C, DALI_RESET);
	delay_ms(10);
	dali_transmit(DALI_BROADCAST_C, DALI_OFF_C);
	delay_ms(10);
	dali_transmit(0b10100101, 0b00000000); //initialise
	delay_ms(10);
	dali_transmit(0b10100101, 0b00000000); //initialise
	delay_ms(10);
	dali_transmit(0b10100111, 0b00000000); //randomise
	delay_ms(10);
	dali_transmit(0b10100111, 0b00000000); //randomise
    
    dali_transmit(0b10100001, 0b00000000);  //terminate
	dali_transmit(DALI_BROADCAST_C, DALI_ON_C);  //broadcast on
}

void init_dali_timer(void){
    // DALI operates at 1200 baud -> 2400 Hz interrupt rate
    T8CONbits.TON = 0;
    T8CONbits.TCS = 0; // use internal instruction cycle as clock source
    T8CONbits.TGATE = 0; // disable gated timer
    T8CONbits.TCKPS = 0b11; // prescaler 1:256
    TMR8 = 0; // clear timer register
    PR8 = 104 - 1; // set period to get 2403.8 Hz
    _T8IF = 0; // clear interrupt flag
    _T8IE = 1; // enable interrupt
}

void __attribute__ ( (__interrupt__, no_auto_psv) ) _T8Interrupt( void ){
    static uint8_t b;
    
    if(bit_tx_state == DALI_FIRST_HALF){
        bit_tx_state = DALI_SECOND_HALF;
    } else {
        bit_tx_state = DALI_FIRST_HALF;
    }
    
    if(bit_tx_state == DALI_FIRST_HALF){
        if(dali_n_tx_bits == 0){
            // the final bit was transmitted 
            //   -> stop and prepare for next transmission
            T8CONbits.TON = 0;
            bit_tx_state = DALI_SECOND_HALF;
            
            // set bus to idle state
            if(config.dali_bus_invert){
                CLEAR_PORT_BIT(config.tx_pin);
            } else {
                SET_PORT_BIT(config.tx_pin);
            }
        } else {
            // the previous bit was transmitted
            //   -> transmit new bit
            b = dali_tx_data & 0x1;
            dali_tx_data = dali_tx_data >> 1;
            dali_n_tx_bits--;
            
            if(b == 0){
                SET_PORT_BIT(config.tx_pin);
            } else {
                CLEAR_PORT_BIT(config.tx_pin);
            }
        }
        
    } else {
        // transmit second portion of bit
        if(b == 0){
            CLEAR_PORT_BIT(config.tx_pin);
        } else {
            SET_PORT_BIT(config.tx_pin);
        }
    }
    
    _T8IF = 0;
}


void dali_transmit(uint8_t address, uint8_t data){
    uint32_t tmp;
    uint16_t i;
    
    dali_n_tx_bits = 16 + 3;
    
    tmp = 1;
    tmp = (tmp << 8) | address;
    tmp = (tmp << 8) | data;
    tmp = tmp << 2;
    
    dali_tx_data = 0;
    for(i = 0; i < (32 - 13); i++){
        dali_tx_data = dali_tx_data | ((tmp >> i) & 0x1);
    }
    
    if(config.dali_bus_invert){
        dali_tx_data = ~dali_tx_data;
    }
    
    T8CONbits.TON = 1;
}
