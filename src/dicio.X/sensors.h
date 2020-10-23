#ifndef __SENSORS_H__
#define	__SENSORS_H__

#include <xc.h>
#include "sensor_apds9301.h"
#include "sensor_apds9306_065.h"
#include "sensor_bh1721fvc.h"
#include "sensor_planalta.h"
#include "sensor_opt3001q1.h"
#include "sensor_sht35.h"
#include "sensor_sylvatica.h"

#define N_SENSOR_APDS9301      0
#define N_SENSOR_SHT35         1
#define N_SENSOR_BH1721FVC     0
#define N_SENSOR_OPT3001Q1     0
#define N_SENSOR_APDS9306      1
#define N_SYLVATICA_SENSORS    2
#define N_PLANALTA_SENSORS     1
#define N_SENSOR_LICOR         1

#define SENSOR_ERROR_TH        1

typedef enum {
    sensor_type_sht35,
    sensor_type_bh1721fvc,
    sensor_type_apds9301,
    sensor_type_opt3001q1,
    sensor_type_apds9306,
    n_sensor_type,
} sensor_type_t;


typedef union {
    sensor_sht35_config_t sht35;
    sensor_bh1721fvc_config_t bh1721fvc;
    sensor_opt3001q1_config_t opt3001q1;
    sensor_apds9301_config_t apds9301;
    sensor_apds9306_config_t sensor_apds9306;
} sensor_custom_config_t;

typedef struct sensor_config_s {
    sensor_type_t type;

    sensor_custom_config_t config;
} sensor_config_t; 

#define N_SENSORS 15

extern i2c_message_t* i2c_message_buffer[N_SENSORS];
extern uint16_t n_sample_messages;

#ifdef	__cplusplus
extern "C" {
#endif
    
void sensors_init(void);
void sensors_start(void);

void sensor_callback(void);
void sensor_status_report(void);
void send_sensor_status(sensor_general_config_t* config);

i2c_error_t sensor_init_opt3001q1(void);
i2c_error_t sensor_init_apds9301(void);
i2c_error_t sensor_init_sht35(void);
i2c_error_t sensor_init_bh1721fvc(void);
i2c_error_t sensor_init_apds9306(void);
i2c_error_t sensor_init_sylvatica(void);
i2c_error_t sensor_init_planalta(void);
void sensor_init_licor();

void sensors_reset_environmental(void);
bool sensors_init_environmental(void); 

void sensors_reset_plant(void);
bool sensors_init_plant(void); 

void sensor_licor_all_low(void);
void sensor_licor_all_high(void);

void sensors_data_init(void);

void sensors_error_check(void);
void sensors_error_recover(void);

#ifdef	__cplusplus
}
#endif

#endif

