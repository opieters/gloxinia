#ifndef __LIA_H__
#define	__LIA_H__

#include <xc.h> 
#include <pga.h>
#include <utilities.h>
#include <dsp.h>

typedef enum {
    LIA_REF_R_OFF,
    LIA_REF_R_10R,
    LIA_REF_R_1K,
    LIA_REF_R_100K,
    LIA_REF_R_10M
} lia_ref_r_t;

typedef enum {
    LIA_STATUS_IDLE,
    LIA_STATUS_ON,
    LIA_STATUS_ERROR,
    LIA_STATUS_OFF,
} lia_status_t;

typedef struct {
    lia_ref_r_t impedance;
    lia_status_t status;
    pga_config_t pga;
    const pin_t pin_imp_sel0;
    const pin_t pin_imp_sel1;
    const pin_t pin_imp_en;
    const pin_t pin_buffer_shdn;
} lia_config_t; 

#ifdef	__cplusplus
extern "C" {
#endif 

    
    void update_lia(lia_config_t* config);
    void init_lia(lia_config_t* config);
    
    void lia_mixer_no_dc2(unsigned int n, fractional* src, fractional* i1, fractional* q1, fractional* i2, fractional* q2);

#ifdef	__cplusplus
}
#endif 

#endif

