#ifndef __FIR_COEFFS_H__
#define __FIR_COEFFS_H__

#include <xc.h>
#include <dsp.h>

#define N_FIR_COEFFS0 100
#define N_FIR_COEFFS1 100
#define N_FIR_COEFFS2 100
#define N_FIR_COEFFS3 192

#ifdef	__cplusplus
extern "C" {
#endif

extern fractional fir_coeffs_0[100];
extern fractional fir_coeffs_1[100];
extern fractional fir_coeffs_2[100];
extern fractional fir_coeffs_3[192];

#ifdef	__cplusplus
}
#endif
#endif
