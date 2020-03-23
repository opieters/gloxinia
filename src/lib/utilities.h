#ifndef __UTILITIES_H__
#define	__UTILITIES_H__

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdio.h>
#include <string.h>


//#define __DEBUG__
//#define __LOG__

#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))
#define ABS(x)   ((x) > 0?(x):-(x))
#define PRINT_BUFFER_LENGTH 64

#define DMAREQ_IRQ_INT0    0b00000000
#define DMAREQ_IRQ_IC1     0b00000001
#define DMAREQ_IRQ_IC2     0b00000101
#define DMAREQ_IRQ_IC3     0b00100101
#define DMAREQ_IRQ_IC4     0b00100110
#define DMAREQ_IRQ_OC1     0b00000010
#define DMAREQ_IRQ_OC2     0b00000110
#define DMAREQ_IRQ_OC3     0b00011001
#define DMAREQ_IRQ_OC4     0b00011010
#define DMAREQ_IRQ_TMR2    0b00000111
#define DMAREQ_IRQ_TMR3    0b00001000
#define DMAREQ_IRQ_TMR4    0b00011011
#define DMAREQ_IRQ_TMR5    0b00011100
#define DMAREQ_IRQ_SPI1    0b00001010
#define DMAREQ_IRQ_SPI2    0b00100001
#define DMAREQ_IRQ_SPI3    0b01011011
#define DMAREQ_IRQ_SPI4    0b01111011
#define DMAREQ_IRQ_U1RX    0b00001011
#define DMAREQ_IRQ_U1TX    0b00001100
#define DMAREQ_IRQ_U2RX    0b00011110
#define DMAREQ_IRQ_U2TX    0b00011111
#define DMAREQ_IRQ_U3RX    0b01010010
#define DMAREQ_IRQ_U3TX    0b01010011
#define DMAREQ_IRQ_U4RX    0b01011000
#define DMAREQ_IRQ_U4TX    0b01011001
#define DMAREQ_IRQ_ECAN1RX 0b00100010
#define DMAREQ_IRQ_ECAN1TX 0b01000110
#define DMAREQ_IRQ_ECAN2RX 0b00110111
#define DMAREQ_IRQ_ECAN2TX 0b01000111
#define DMAREQ_IRQ_DCI     0b00111100
#define DMAREQ_IRQ_ADC1    0b00001101
#define DMAREQ_IRQ_ADC2    0b00010101
#define DMAREQ_IRQ_PMP     0b00101101

#define PIN_INIT(x, y)   {.port_r = &PORT##x, .tris_r = &TRIS##x, .lat_r = &LAT##x, .n = y}
#define SET_BIT(x, y)    ((*(x)) |= 1UL << (y))
#define CLEAR_BIT(x, y)  ((*(x)) &= ~(1UL << (y)))
#define CLEAR_PORT_BIT(x)  ((*((x).port_r)) &= ~(1UL << ((x).n)))
#define SET_PORT_BIT(x)    ((*((x).port_r)) |= 1UL << ((x).n))
#define TOGGLE_BIT(x, y) ((*(x)) ^= 1UL << (y))

#define GET_BIT(x, y) (((*(x)) >> (y)) & 1UL)

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

#define report_error(X) print_error(X, strlen(X))

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    
    typedef struct {
        volatile uint16_t* port_r;
        volatile uint16_t* tris_r;
        volatile uint16_t* lat_r;
        uint8_t n;
    } pin_t;
    
    extern char print_buffer[PRINT_BUFFER_LENGTH];
    
    void delay_us(uint16_t delay);
    void delay_ms(uint16_t delay);
    
    void blinky_init(pin_t* pin, short timer_on);
    void toggle_led(void);
    
    void set_error_pin(pin_t* pin);
    void set_error_led(void);
    
    
    void set_error_loop_fn(void (*fn)(void));
    void print_error(char* message, unsigned int length);
    

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __UTILITIES_H__ */

