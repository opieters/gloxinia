// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __SLAVE_ADDRESS_H__
#define	__SLAVE_ADDRESS_H__

#include <xc.h> // include processor files - each processor file is guarded.  

extern volatile uint8_t slave_address;

#ifdef	__cplusplus
extern "C" {
#endif

    void init_slave_address(void);

#ifdef	__cplusplus
}
#endif

#endif

