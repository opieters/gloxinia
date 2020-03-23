#ifndef __FIR_COMMON_H__
#define __FIR_COMMON_H__

#include <xc.h>
#include <dsp.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief LIA mixer function to shift frequency components.
 * 
 * @param n: number of samples to mix, must be a multiple of 4!
 * @param src: array of data samples from ADC
 * @param i: array of output I samples
 * @param q: array of output Q samples
 * @param offset: DC offset that can be applied to each ADC sample to 
 * improve resolution
 * 
 * @details Assembly implementation to improve efficiency.
 */
void lia_mixer(unsigned int n, fractional* src, fractional* i, fractional* q, fractional offset);

void lia_mixer_no_dc(unsigned int n, fractional* src, fractional* i, fractional* q);
void lia_mixer_no_dc_eds(unsigned int n, __eds__ fractional* src, fractional* i, fractional* q);

fractional foldedFIR(unsigned int num, fractional* src, FIRStruct* fir);
void compressedFIR(unsigned int n_outputs, unsigned int decimation_factor, fractional* src, FIRStruct* fir, fractional* dst);

void fir_offset(uint16_t n_output, fractional* y,  fractional* x, FIRStruct* fir, uint16_t decimation, uint16_t offset);
void fir_compressed(uint16_t n_output, fractional* y,  fractional* x, FIRStruct* fir, uint16_t decimation);


void fir_cpr(unsigned int num, fractional* src, FIRStruct* fir, unsigned int dectimation_factor);

void sample_copy(unsigned int num, fractional* src, fractional* dst);

void covert_uint_to_fract(unsigned int num, uint16_t* src, fractional* dst);

void copy_adc_data(unsigned int num, fractional* dst, fractional* src);

void copy_uint_to_fract(unsigned int num, fractional* src, fractional* dst, unsigned int step);

#ifdef __cplusplus
}
#endif

#endif