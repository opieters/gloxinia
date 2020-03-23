#include <lia.h>
#include <utilities.h>

#if HARDWARE_CONFIG == 0
#define R_REF1_A0 (PORTBbits.RB14)
#define R_REF1_A1 (PORTBbits.RB13)
#define R_REF1_EN (PORTBbits.RB12)

#define R_REF1_A0_IO (TRISBbits.TRISB14)
#define R_REF1_A1_IO (TRISBbits.TRISB13)
#define R_REF1_EN_IO (TRISBbits.TRISB12)

#define R_REF2_A0 (PORTBbits.RB11)
#define R_REF2_A1 (PORTBbits.RB10)
#define R_REF2_EN (PORTBbits.RB9)

#define R_REF2_A0_IO (TRISBbits.TRISB11)
#define R_REF2_A1_IO (TRISBbits.TRISB10)
#define R_REF2_EN_IO (TRISBbits.TRISB9)
#else
#error "Hardware not supported."
#endif

void init_lia(lia_config_t* config){
    config->status = LIA_STATUS_IDLE;
    config->impedance = LIA_REF_R_OFF;
    config->pga.status = PGA_STATUS_OFF;
    
    // pin output configuration
    SET_BIT(config->pin_imp_sel0.tris_r, config->pin_imp_sel0.n);
    SET_BIT(config->pin_imp_sel1.tris_r, config->pin_imp_sel1.n);
    SET_BIT(config->pin_imp_en.tris_r, config->pin_imp_en.n);
    SET_BIT(config->pin_buffer_shdn.tris_r, config->pin_buffer_shdn.n);
    
    // init PGA
    init_pga(&config->pga);
    
    // pin configuration
    update_lia(config);
}

void update_lia_impedance(lia_config_t* config){
    switch(config->impedance){
        case LIA_REF_R_10R:
            R_REF1_A0 = 0;
            R_REF1_A1 = 0;
            break;
        case LIA_REF_R_1K:
            R_REF1_A0 = 1;
            R_REF1_A1 = 0;
            break;
        case LIA_REF_R_100K:
            R_REF1_A0 = 0;
            R_REF1_A1 = 1;
            break;
        case LIA_REF_R_10M:
            R_REF1_A0 = 1;
            R_REF1_A1 = 1;
            break;
        default:
            break;
    }
}

void update_lia_hardware(lia_config_t* config){
    switch(config->status){
        case LIA_STATUS_IDLE:
            // do nothing
            break;
        case LIA_STATUS_ON:
            SET_BIT(config->pin_imp_en.lat_r, config->pin_imp_en.n);
            SET_BIT(config->pin_buffer_shdn.lat_r, config->pin_buffer_shdn.n);
            break;
        case LIA_STATUS_ERROR:
            // disable all HW
        case LIA_STATUS_OFF:
            // disable all HW
            CLEAR_BIT(config->pin_imp_en.lat_r, config->pin_imp_en.n);
            CLEAR_BIT(config->pin_buffer_shdn.lat_r, config->pin_buffer_shdn.n);
            break;
        default: 
            break;
    }
}

void update_lia(lia_config_t* config){
    switch(config->status){
        case LIA_STATUS_IDLE:
            // leave as is
            break;
        case LIA_STATUS_ON:
            if(config->pga.status == PGA_STATUS_ERROR){
                config->status = LIA_STATUS_ERROR;
            } else {
                config->pga.status = PGA_STATUS_ON;
            }
            break;
        case LIA_STATUS_ERROR:
        case LIA_STATUS_OFF:
            if(config->pga.status != PGA_STATUS_ERROR){
                config->pga.status = PGA_STATUS_OFF;
            }
            break;
        default:
            break;
    }
    
    update_lia_impedance(config);
    update_lia_hardware(config);
    update_pga_status(&config->pga);
}