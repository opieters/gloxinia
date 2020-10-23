#ifndef __PLANALTA_FREQ_SWEEP_H__
#define	__PLANALTA_FREQ_SWEEP_H__

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        PLANALTA_FS_FREQ_50KHZ,
        PLANALTA_FS_FREQ_20KHZ,
        PLANALTA_FS_FREQ_10KHZ,
        PLANALTA_FS_FREQ_5KHZ,
        PLANALTA_FS_FREQ_2KHZ,
        PLANALTA_FS_FREQ_1KHZ,
        PLANALTA_FS_FREQ_500HZ,
        PLANALTA_FS_FREQ_200HZ,
        PLANALTA_FS_FREQ_100HZ
        PLANALTA_FS_FREQ_20HZ,
        PLANALTA_FS_FREQ_10HZ,
    } planalta_fs_freq_t;
    
    typedef struct {
        planalta_fs_freq_t freq;
    } planalta_fs_config_t;


#ifdef	__cplusplus
}
#endif

#endif

