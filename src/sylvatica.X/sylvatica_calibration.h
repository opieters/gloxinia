#ifndef __SYLVATICA_CALIBRATION_H__
#define	__SYLVATICA_CALIBRATION_H__

#include <xc.h>
#include "sylvatica.h"

#define SYLVATICA_SCALE_MARGIN_PCT 100
#define SYLGATICA_MAX_N_CALIBRATIONS 100

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
   

    void sylvatica_run_calibration(sylvatica_config_t* config);
    void sylvatica_gain_calibration(sylvatica_config_t* config, const uint8_t channel_n);
    void init_calibration_timer(void);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif

