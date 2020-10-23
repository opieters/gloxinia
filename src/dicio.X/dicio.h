// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __DICIO_H__
#define	__DICIO_H__ 

#include <xc.h>
#include <utilities.h>
#include <sylvatica.h>
#include <planalta.h>
#include <i2c.h>
#include <uart.h>

#define DICIO_READ_FREQUENCY 10 // interrupt frequency of the readout function

#define N_SPI1_SS_PINS 3
#define N_SPI2_SS_PINS 2
#define N_I2C_BUSES    2

typedef struct {
    pin_t blinky_pin;
    pin_t error_pin;
    pin_t rst1_sensor_pin;
    pin_t rst2_sensor_pin;
    pin_t spi1_ss[N_SPI1_SS_PINS];
    pin_t spi2_ss[N_SPI2_SS_PINS];
    i2c_config_t i2c_config[N_I2C_BUSES];
    uint32_t output_frequency;
} dicio_config_t;

#ifdef	__cplusplus
extern "C" {
#endif
    
    void dicio_init(void);
    void dicio_loop(void);
    
    void dicio_i2c_mw_sr_callback(i2c_message_t* m);
    void dicio_i2c_mr_sw_callback(i2c_message_t* m);
    
    void dicio_init_pins(void);
    
    void dicio_init_clock_sync(void);
    void dicio_start_clock_sync(void);
    void dicio_stop_clock_sync(void);
    
    void dicio_dummy_callback(void);
    
    void dicio_set_sensor_callback(void (*cb)(void));
    void dicio_set_actuator_callback(void (*cb)(void));
    
    void dicio_send_message(serial_cmd_t cmd, uint16_t can_ext_id, 
        uint8_t* data, uint8_t data_length);
    


#ifdef	__cplusplus
}
#endif

#endif	/* __DICIO_H__ */
