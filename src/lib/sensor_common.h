
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __SENSOR_COMMON_H__
#define	__SENSOR_COMMON_H__

#include <xc.h> // include processor files - each processor file is guarded.  
#include <i2c.h>
#include <uart.h>
#include <can.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
#define ERROR_DATA_SIZE 4
   
typedef enum {
    SENSOR_STATUS_INACTIVE   = 0,
    SENSOR_STATUS_IDLE       = 1,
    SENSOR_STATUS_ACTIVE     = 2,
    SENSOR_STATUS_ERROR      = 3,
    N_SENSOR_STATUS          = 4,
} sensor_staus_t;

typedef struct sensor_log_s {
    can_message_t can_message;
    uart_message_t uart_message;
} sensor_log_t;

typedef struct sensor_elog_s {
    can_message_t can_message;
    uart_message_t uart_message;
    uint16_t n_errors;
} sensor_elog_t;


typedef struct sensor_general_config_s {
    uint8_t local_id;
    uint8_t global_id;
    
    uint8_t address;
    i2c_bus_t i2c_bus;
    
    sensor_staus_t status;
    
    sensor_log_t dlog;
    sensor_elog_t elog;

    uint8_t tx_data[CAN_MAX_N_BYTES+UART_HEADER_SIZE];
    uint8_t error_data[UART_HEADER_SIZE+ERROR_DATA_SIZE];
} sensor_general_config_t;


void sensor_update_status(sensor_general_config_t* config, const i2c_error_t i2c_error);
void sensor_send_error(sensor_elog_t* config, i2c_message_t* m);
void sensor_send_data(sensor_log_t* config, uint8_t* data, uint8_t length);
void sensor_send_data_no_copy(sensor_log_t* config, uint8_t* data, uint8_t length);

uint8_t sensor_get_local_id(void);
void sensor_reset_local_id(void);

void sensor_init_common_config(sensor_general_config_t* config, uint8_t length);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

