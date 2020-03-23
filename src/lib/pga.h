#ifndef __PGA_H__
#define	__PGA_H__

#include <xc.h>
#include <utilities.h>
#include <spi.h>

typedef enum {
    PGA_MUX_VCAL_CH0,
    PGA_MUX_CH1,
    PGA_CAL1,
    PGA_CAL2,
    PGA_CAL3,
    PGA_CAL4,
    N_PGA_MUX_CHANNEL
} pga_mux_channel_t;

typedef enum {
    PGA_GAIN_1,
    PGA_GAIN_2,
    PGA_GAIN_5,
    PGA_GAIN_10,
    PGA_GAIN_20,
    PGA_GAIN_50,
    PGA_GAIN_100,
    PGA_GAIN_200,
    N_PGA_GAIN
} pga_gain_t;

typedef enum {
    PGA_STATUS_OFF,
    PGA_STATUS_ON,
    PGA_STATUS_ERROR,
} pga_status_t;

typedef struct {
    pga_mux_channel_t channel;
    pga_gain_t gain;
    pga_status_t status;
    pin_t cs_pin;
    void (*spi_message_handler) (spi_message_t* m);
} pga_config_t;

#ifdef	__cplusplus
extern "C" {
#endif
    
    void enable_pga113(void);
    void disable_pga113(void);
    void select_channel_pga113(pga_mux_channel_t channel);
    void select_gain_pga113(pga_gain_t gain);
    
    void init_pga(pga_config_t* config);
    void update_pga_status(pga_config_t* config);
    
#ifdef	__cplusplus
}
#endif

#endif

