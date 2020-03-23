

#ifndef __DALI_H__
#define __DALI_H__

#include <xc.h>
#include <utilities.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

// DALI coomands
#define DALI_BROADCAST_DP  0b11111110
#define DALI_BROADCAST_C   0b11111111
#define DALI_ON_DP         0b11111110
#define DALI_OFF_DP        0b00000000
#define DALI_ON_C          0b00000101
#define DALI_OFF_C         0b00000000
# define DALI_QUERY_STATUS 0b10010000
# define DALI_RESET        0b00100000
    
#define DALI_FIRST_HALF  0
#define DALI_SECOND_HALF 1

typedef struct{
    pin_t rx_pin;
    pin_t tx_pin;
    bool dali_bus_invert;
    
} dali_config_t;

void init_dali();
void init_dali_timer(void);

void dali_transmit(uint8_t address, uint8_t data);

#ifdef	__cplusplus
}
#endif

#endif	/* __GROWTH_CHAMBER_H__ */
