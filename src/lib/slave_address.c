#include "slave_address.h"

#if HARDWARE_CONFIG == 0
#define AS0 (PORTFbits.RF5)
#define AS1 (PORTFbits.RF3)
#define AS0_IO (TRISFbits.TRISF5)
#define AS1_IO (TRISFbits.TRISF3)
#define I2C_SLAVE_BASE_ADDRESS 0b00100000
#else 
#error "Hardware not supported."
#endif

volatile uint8_t slave_address = 0;

void init_slave_address(void){
    // configure port as input
    AS0_IO = 1;
    AS1_IO = 1;
    
    // read port
    slave_address = I2C_SLAVE_BASE_ADDRESS | (AS1 << 2) | (AS0 << 1);
}
