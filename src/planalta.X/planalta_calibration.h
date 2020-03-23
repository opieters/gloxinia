#ifndef __PLANALTA_CALIBRATION_H__
#define	__PLANALTA_CALIBRATION_H__

#include <xc.h> // include processor files - each processor file is guarded.  
#include "planalta.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    void planalta_input_calibration(planalta_config_t* config);
    void planalta_output_calibration(planalta_config_t* config);

#ifdef	__cplusplus
}
#endif

#endif

