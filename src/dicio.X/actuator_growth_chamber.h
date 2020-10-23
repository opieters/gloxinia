#ifndef __ACTUATOR_GROWTH_CHAMBER_H__
#define	__ACTUATOR_GROWTH_CHAMBER_H__

#include <xc.h>  
#include <i2c.h>
#include <actuator_common.h>

#define ACTUATOR_GROWTH_CHAMBER_LENGTH 2
#define ACTUATOR_GROWTH_CHAMBER_LOG_LENGTH 4
#define ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH 3
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_0 0b00010000
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_1 0b00010010
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_2 0b00010100
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_3 0b00010110
#define ACTUATOR_GROWTH_CHAMBER_DAC_CH_4 0b00110100
#define ACTUATOR_GROWTH_CHAMBER_DAC_ADDRESS 0b1001110
#define ACTUATOR_GROWTH_CHAMBER_TEMP_CHANNEL 0
#define ACTUATOR_GROWTH_CHAMBER_RH_CHANNEL 1

// 0x200 7.4 C
// 0x700 45.1
// 0xA00 31.7 
// 0xB00 69.4
// 21.0C and 64.0% RH
#define ACTUATOR_GROWTH_CHAMBER_TEMP_DEFAULT 0x69F
#define ACTUATOR_GROWTH_CHAMBER_RH_DEFAULT 0xA00

#define ACTUATOR_GC_UART_BAUDRATE 9600
#define ACTUATOR_GC_UART_QUEUE_SIZE 16
#define ACTUATOR_GC_UART_BUFFER_SIZE 16
#define GC_UART_HEADER_SIZE 4

#ifdef	__cplusplus
extern "C" {
#endif
    
typedef enum {
    GC_UART_CMD_NONE                  = 0x00,
    GC_UART_CMD_SET_RELAY             = 0x01,
    GC_UART_CMD_READ_RELAY            = 0x02,
    GC_UART_CMD_SET_DAC               = 0x03,
    GC_UART_CMD_INFO                  = 0x04,
    GC_UART_CMD_ERR                   = 0x05,
    GC_UART_CMD_VERSION_CHECK         = 0x06,
} gc_uart_cmd_t;

typedef enum {
    GC_UART_IF1,
    GC_UART_IF3,
} gc_uart_if_t;
    
typedef struct {
    actuator_general_config_t general;
    
    uint8_t address;
    i2c_bus_t i2c_bus; 
    
    uint16_t temperature;
    uint16_t relative_humidity;
    
    i2c_message_t m_temp;
    i2c_message_t m_rh;
    uint8_t i2c_temp_data[ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH];
    uint8_t i2c_rh_data[ACTUATOR_GROWTH_CHAMBER_I2C_DATA_LENGTH];
    
    uint32_t period;
    
    bool uart1_interface_up;
    bool uart3_interface_up;
} growth_chamber_config_t;

typedef struct {
    gc_uart_if_t interface;
    gc_uart_cmd_t command;       
    uint16_t length;
    uint8_t* data;
    volatile uart_message_status_t status;
} gc_uart_message_t;

void init_gc_uart();

void gc_uart_send(gc_uart_message_t* m);
    
void process_gc_uart1_queue(void);
void process_gc_uart3_queue(void);

void gc_uart_parse_to_buffer(uint8_t* data, gc_uart_message_t* m, const size_t max_length);

void init_growth_chamber(growth_chamber_config_t* config);

void parse_gc_i2c_data(uint8_t channel_n);

void gc_i2c_send_temperature(void);

void gc_i2c_send_rh(void);

void i2c_cb_growth_chamber(i2c_message_t* m);
void gc_uart_init_message(gc_uart_message_t* m, 
        gc_uart_if_t interface,
        gc_uart_cmd_t command,          
        uint8_t* data,
        uint16_t length);

void process_gc_rx_message(gc_uart_message_t* m);
void gc_uart_init_messages(void);
void process_gc_uart_queue(void);

void gc_uart1_init(void);
void gc_uart3_init(void);

void actuator_gc_callback(void);

void gc_update_temperature(uint16_t temperature);
void gc_update_rh(uint16_t rh);

#ifdef	__cplusplus
}
#endif

#endif

