#ifndef __UART_H__
#define	__UART_H__

#include <xc.h> // include processor files - each processor file is guarded.  
#include <utilities.h>
#include <stddef.h>

#define UART_CMD_START ('\x01')
#define UART_CMD_STOP  ('\x04')

#define UART_MESSAGE_BUFFER_LENGTH 128
#define UART_HEADER_SIZE 7

#define uart_simple_print(X) uart_print(X, strlen(X));

typedef enum {
    SERIAL_START_MEAS_CMD        = 0x00,
    SERIAL_STOP_MEAS_CMD         = 0x01,
    SERIAL_SENSOR_ACTIVATE_CMD   = 0x02,
    SERIAL_SENSOR_DEACTIVATE_CMD = 0x03,
    SERIAL_RESET_NODE_CMD        = 0x04,
    SERIAL_RESET_SYSTEM_CMD      = 0x05,
    SERIAL_TEXT_MESSAGE_CMD      = 0x06,
    SERIAL_SENSOR_DATA_CMD       = 0x07,
    SERIAL_SENSOR_STATUS_CMD     = 0x08,
    SERIAL_MEAS_PERIOD_CMD       = 0x09,
    SERIAL_ERR_MESSAGE_CMD       = 0x0A,
    SERIAL_LOOP_MESSAGE_CMD      = 0x0B,
    SERIAL_ACTUATOR_STATUS       = 0x0C,
    SERIAL_HELLO_CMD             = 0x0D,
    SERIAL_INIT_SAMPLING_CMD     = 0x0E,
    SERIAL_INIT_SENSORS_CMD      = 0x0F,
    SERIAL_SENSOR_ERROR_CMD      = 0x10,
    SERIAL_LIA_GAIN_SET_CMD      = 0x11,
    SERIAL_UNKNOWN_CMD           = 0x12,
    SERIAL_MEAS_TRIGGER_CMD      = 0x13,
    SERIAL_SENSOR_CONFIG_CMD     = 0x14,
    SERIAL_ACTUATOR_DATA_CMD     = 0x15,
    SERIAL_ACTUATOR_ERROR_CMD    = 0x16,
    SERIAL_ACTUATOR_TRIGGER_CMD  = 0x17,
    SERIAL_ACTUATOR_GC_TEMP      = 0x18,
    SERIAL_ACTUATOR_GC_RH        = 0x19,
    SERIAL_ACTUATOR_GC_LEDS      = 0x1A,
    SERIAL_ACTUATOR_PUMP         = 0x1B,
    SERIAL_START_INIT            = 0x1C,
            
    N_SERIAL_CMD                 = 0x1D,
} serial_cmd_t;

typedef enum {
    UART_MSG_NONE,
    UART_MSG_TRANSFERRED,
    UART_MSG_QUEUED,
    UART_MSG_SENT,
    UART_MSG_ERROR,
    UART_MSG_INIT_DONE,
    UART_MSG_RECEIVED,
    N_UART_MSG_STATUS_T
} uart_message_status_t;

typedef struct {
    serial_cmd_t command;
    uint8_t id;
    uint16_t extended_id;            
    uint16_t length;
    uint8_t* data;
    volatile uart_message_status_t status;
} uart_message_t;

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    extern volatile size_t n_uart_rx_messages;
    
    void uart_init(uint32_t baudrate);
    
    void uart_rx_command_cb(void);
        
    void uart_print(const char* message, size_t lengh);
    void uart_wait(void);

    void uart_init_message(uart_message_t* m, 
            serial_cmd_t command,
            uint8_t id,
            uint16_t extended_id,            
            uint8_t* data,
            size_t length);
    
    void uart_queue_message(uart_message_t* m);
    
    void uart_parse_to_buffer(uint8_t* data, uart_message_t* m, size_t max_length);
    
    void uart_await_tx(uart_message_t* m);
    
    void uart_reset_message(uart_message_t* m);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __UART_H__ */

