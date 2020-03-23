; include common definitions
	.nolist
	.include	"../lib/dspcommon.inc"
	.list

	.section .dspExtensions, code
	


	
/*******************************************************************************
  FIR API

  Company:
    Microchip Technology Inc.

  File Name:
    fir.s

  Summary:
    This file has FIR routine

  Description:
    This file consists of FIR function which applies an FIR filter to a sequence
 of source samples and  places the result in a sequence of destination samples.
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

/*****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

 #.section .libdsp, code

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; _FIR: FIR block filtering.
;
; Operation:
; y[n] = sum_(m=0:M-1){h[m]*x[n-m]}, 0 <= n < N.
;
; x[n] defined for 0 <= n < N,
; y[n] defined for 0 <= n < N,
; h[m] defined for 0 <= m < M as an increasing circular buffer,
; NOTE: delay defined for 0 <= m < M as an increasing circular buffer.
;
; Input:
; w0 = number of samples to generate (numSamps, N)
; w1 = ptr to output samples (dstSamps, y)
; w2 = ptr to input samples (srcSamps, x)
; w3 = filter structure (FIRStruct, h)
; w4 = decimation factor
; w5 = configure DSP engine for signed/unsigned operations (0 = signed, 1 = unsigned)
;
; Return:
; w0 = ptr to output samples (dstSamps, y)
;
; System resources usage:
; {w0..w6} used, not restored
; {w8,w10} saved, used, restored
;  AccuA  used, not restored
;  CORCON  saved, used, restored
;  PSVPAG  saved, used, restored (if coeffs in P memory)
;  MODCON  saved, used, restored
;  XMODSRT saved, used, restored
;  XMODEND saved, used, restored
;  YMODSRT saved, used, restored
;  YMODEND saved, used, restored
;
; DO and REPEAT instruction usage.
; 1 DO instructions
; 1 REPEAT intructions
;
; Program words (24-bit instructions):
; 56
;
; Cycles (including C-function call and return overheads):
; 53 + N*(5+M), or
; 56 + N*(9+M) if coefficients in P memory.
;............................................................................

 .global _fir_compressed ; export
_fir_compressed:

;............................................................................

 ; Save working registers.
 push w8    ; w8 to TOS
 ;push w9
 push w10    ; w10 to TOS

;............................................................................

 ; Prepare CORCON for fractional computation.
 push CORCON
 ;mov.b w5, w9
 ;cp0.b w5
 ;bra z, _signed_setup
 ;fractsetup_unsigned w8
 ;bra _check_psv_access
 ;_signed_setup:
 fractsetup w8
 

;............................................................................

 ; Prepare CORCON and PSVPAG for possible access of data
 ; located in program memory, using the PSV.
 _check_psv_access:
 push PSVPAG

 mov [w3+oCoeffsPage],w10  ; w10= coefficients page
 mov #COEFFS_IN_DATA,w8  ; w8 = COEFFS_IN_DATA
 cp w8,w10    ; w8 - w10
 bra z,_noPSVfir   ; if w10 = COEFFS_IN_DATA
      ; no PSV management
      ; else
 psvaccess w8   ; enable PSV bit in CORCON
 mov w10,PSVPAG   ; load PSVPAG with program
      ; space page offset
_noPSVfir:

;............................................................................

 ; Prepare core registers for modulo addressing.
 push MODCON
 push XMODSRT
 push XMODEND
 push YMODSRT
 push YMODEND

;............................................................................

 ; Setup registers for modulo addressing.
 mov #0xC0A8,w10   ; XWM = w8, YWM = w10
      ; set XMODEND and YMODEND bits
 mov [w3+oDelayEnd],w8  ; w8 -> last byte of d[M-1] 
 mov w10,MODCON   ; enable X,Y modulo addressing
 ; do NOT use an indirect read operation to ANY w register after modifying MODCON
 
 mov w8,XMODEND   ; init'ed to coeffs end address
 mov [w3+oCoeffsEnd],w10  ; w10-> last byte of h[M-1]
 mov w10,YMODEND   ; init'ed to delay end address
 mov [w3+oDelayBase],w8  ; w8 -> d[0]
 mov w8,XMODSRT   ; init'ed to coeffs base address
      ; (increasing buffer,
      ;  2^n aligned)
 mov [w3+oCoeffsBase],w10  ; w10 -> h[0]
 mov w10,YMODSRT   ; init'ed to delay base address
      ; (increasing buffer,
      ;  2^n aligned)
; DO NOT read X/Y register indirectly immediately after modifying XMODi, YMODi

;............................................................................

 push w1    ; save return value (y)

;............................................................................

 ; Perpare to all filter.
 dec w4, w7
 mov [w3+oDelay],w8   ; w10 points at oldest delay entry
      ; sample d[m], 0 <= m < M
 mov [w3+oNumCoeffs],w4  ; w4 = M
 sub w4,#2,w4   ; W4 = M-2
 dec w0,w0    ; w0 = N-1


;............................................................................

 ; Perform filtering of all samples.
 do w0,_endFilterfir  ; { ; do (N-1)+1 times

 ; Prepare to filter sample.
 repeat w7
 mov [w2++],[w8++]
;.ifdef YMEM_ERRATA
;	nop
;.endif
 
	;do w7, _copy_x
	;mov [w2++],[w8++]
	;nop
	;nop
	;_copy_x:
	;nop

 clr a,[w8]+=2,w6,[w10]+=2,w5 ; a  = 0
      ; w5 = h[0]
      ; w8-> h[1]
      ; w6 = d[oldest]
      ; w10->d[oldest+1]

 ; Filter each sample.
 ; (Perform all but two last MACs.)
 repeat w4   ; { ; do (M-2)+1 times
 mac w5*w6,a,[w8]+=2,w6,[w10]+=2,w5 ; a += h[m]*d[current]
      ; w5 = h[m]
      ; w8-> h[m+1]
      ; w6 = d[oldest+m]
      ; w10->d[oldest+m+1]
; }
 ; (Perform second last MAC.)
 
 ;mac w5*w6,a,[w8]+=2,w5,[w10],w6 ; a += h[M-2]*d[current]
      ; w5 = h[M-1]
      ; w8-> h[0]
      ; w6 = d[next]
      ; w10->d[next]
 ; (Perform last MAC.)
 
 mac w5*w6,a    ; a += h[M-1]*d[current]
 
 ;cp0.b w9
 ;bra z, _endFilterfir
 ;sftac a, #1


_endFilterfir:
 ; Save filtered result.
 sac.r a,[w1++]   ; y[n] =
      ;   sum_{m=0:M-1}(h[m]*x[n-m])
      ; w1-> y[n+1]
; }

;............................................................................

 ; Update delay pointer.
 mov w8,[w3+oDelay]   ; note that the delay pointer
      ; may wrap several times around
      ; d[m], 0 <= m < M, depending
      ; on the value of N
      

;............................................................................

 pop w0    ; restore return value

;............................................................................

 ; Restore core registers for modulo addressing.
 pop YMODEND
 pop YMODSRT
 pop XMODEND
 pop XMODSRT
 pop MODCON

;............................................................................

 ; Restore PSVPAG and CORCON.
 pop PSVPAG
 pop CORCON

;............................................................................

 ; Restore working registers.
 pop w10    ; w10 from TOS
 ;pop w9
 pop w8    ; w8 from TOS

;............................................................................

 return 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
 
 
 .global _covert_uint_to_fract ; export
_covert_uint_to_fract:
    dec w0, w0
    mov #0x8000, w3
    repeat w0
    add w3, [w1++], [w2++]
    return
    
.global _copy_adc_data
    ; w0 = total number of coefficients to copy
    ; w1 = pointer to dst
    ; w2 = pointer to src
_copy_adc_data:
    ;lsr w0, #3, w0  
    dec w0, w0
    push DSRPAG
    mov #0x0001, w3
    mov w3, DSRPAG
    mov #16, w3
    do w0, _final_copy
    mov [w2], [w1++]
    nop
    nop
_final_copy:
    add w2, w3, w2
    pop DSRPAG
    return
    
.global _copy_uint_to_fract
_copy_uint_to_fract:
    ; w0 = number of coefficients in source array
    ; w1 = source array
    ; w2 = destination array
    ; w3 = step size
    dec w0, w0
    add w3, w3, w3
    ; setup read page to EDS memory
    push DSRPAG
    mov #0x0001, w4
    mov w4, DSRPAG
    mov #0x8000, w4
    do w0, _final_copy_conv
    subr w4, [w1], [w2++] ; [w2] = [w1] - w4
_final_copy_conv:
    add w1, w3, w1
    pop DSRPAG
    return
    
    