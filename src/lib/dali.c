

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
}
