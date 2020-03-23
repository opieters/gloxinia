
; this routine computes the output of a mixer to build a LIA, assuming the 
; sampling frequency is 4x higher than the main component's frequency
    .global _lia_mixer ; name of the function
; this function has 5 arguments:
;  - w0 = number of samples to be filtered
;  - w1 = address of first sample to be filtered
;  - w2 = address of first I sample
;  - w3 = address of first Q sample
;  - w4 = DC offset to be added to samples (can be used to correct DC offset
;         that might saturate the output)
_lia_mixer:
    ;push       w5                ; save this register (not one of the arguments)

    lsr        w0, #2, w0        ; divide number of samples by 4
    dec        w0, w0            ; adjust for number of data entries

    do         w0, _end_mixer  ; do instructions: perform loop w0+1 times
    add        w4, [w1++], [w2++]; i(n)   =  1 * x(n)   + DC
    ;clr	       [w3++]
    add        w4, [w1++], [w3++]; q(n+1) =  1 * x(n+1) + DC
    ;clr	       [w2++]
    ;add        w4, [w1++], w5
    add        w4, [w1], [w1]
    neg        [w1++], [w2++]        ; i(n+2) = -1 * x(n+2) - DC
    ;clr	       [w3++]
    add        w4, [w1], [w1]
    ;clr	       [w2++]
_end_mixer:
    neg        [w1++], [w3++]        ; q(n+3) = -1 * x(n+3) - DC
    ;pop        w5
    return
    
;--------------
; alternative version 
;--------------
;
; this routine computes the output of a mixer to build a LIA, assuming the 
; sampling frequency is 4x higher than the main component's frequency
    .global _lia_mixer_no_dc ; name of the function
; this function has 5 arguments:
;  - w0 = number of samples to be filtered
;  - w1 = address of first sample to be filtered
;  - w2 = address of first I sample
;  - w3 = address of first Q sample
_lia_mixer_no_dc:
    lsr        w0, #2, w0        ; divide number of samples by 4
    dec        w0, w0            ; adjust for number of data entries

    do         w0, _end_mixer_no_dc  ; do instructions: perform loop w0+1 times
    mov        [w1++], [w2++]        ; i(n)   =  1 * x(n)
    mov        [w1++], [w3++]        ; q(n+1) =  1 * x(n+1)
    neg        [w1++], [w2++]        ; i(n+2) = -1 * x(n+2) TODO: attention!! if 0x8000 is located in w1, the data in w2 will be INVALID!
_end_mixer_no_dc:
    neg        [w1++], [w3++]        ; q(n+3) = -1 * x(n+3)
    return
    
; this routine computes the output of a mixer to build a LIA, assuming the 
; sampling frequency is 4x higher than the main component's frequency and the
; source buffer MUST be stored in EDS. The visibility is automatically changed
; and restored afterwards
    .global _lia_mixer_no_dc_eds ; name of the function
; this function has 5 arguments:
;  - w0 = number of samples to be filtered
;  - w1 = address of first sample to be filtered
;  - w2 = address of first I sample
;  - w3 = address of first Q sample
_lia_mixer_no_dc_eds:
    mov        DSRPAG, w4
    mov        #0x1, w5
    mov        w5, DSRPAG
    lsr        w0, #2, w0        ; divide number of samples by 4
    dec        w0, w0            ; adjust for number of data entries

    do         w0, _end_mixer_no_dc_eds  ; do instructions: perform loop w0+1 times
    mov        [w1++], [w2++]        ; i(n)   =  1 * x(n)
    mov        [w1++], [w3++]        ; q(n+1) =  1 * x(n+1)
    neg        [w1++], [w2++]        ; i(n+2) = -1 * x(n+2) TODO: attention!! if 0x8000 is located in w1, the data in w2 will be INVALID!
_end_mixer_no_dc_eds:
    neg        [w1++], [w3++]        ; q(n+3) = -1 * x(n+3)
    mov        w4, DSRPAG
    return
    
    
;--------------
; alternative version 
;--------------
;
; this routine computes the output of a mixer to build a LIA, assuming the 
; sampling frequency is 4x higher than the main component's frequency
    .global _lia_mixer_no_dc2 ; name of the function
; this function has 5 arguments:
;  - w0 = number of samples to be filtered
;  - w1 = address of first sample to be filtered
;  - w2 = address of first I sample of LIA1
;  - w3 = address of first Q sample of LIA1
;  - w4 = address of first I sample of LIA2
;  - w5 = address of first Q sample of LIA2
_lia_mixer_no_dc2:
    lsr        w0, #3, w0        ; divide number of samples by 8
    dec        w0, w0            ; adjust for number of data entries
    add        w4, w4, w4        ; multiply by 2 (we need word addresses)
    add        w4, w1, w1        ; shift start address

    do         w0, _end_mixer_no_dc2  ; do instructions: perform loop w0+1 times
    mov        [w1++], [w2++]        ; i1(n)   =  1 * x(n)
    mov        [w1++], [w4++]        ; i2(n)   =  1 * x(n+1)
    mov        [w1++], [w3++]        ; q1(n+1) =  1 * x(n+2)
    mov        [w1++], [w5++]        ; q2(n+1) =  1 * x(n+3)
    neg        [w1++], [w2++]        ; i1(n+2) = -1 * x(n+4) TODO: attention!! if 0x8000 is located in w1, the data in w2 will be INVALID!
    neg        [w1++], [w4++]        ; i2(n+2) = -1 * x(n+5) TODO: attention!! if 0x8000 is located in w1, the data in w2 will be INVALID!
    neg        [w1++], [w3++]        ; q1(n+3) = -1 * x(n+6)
_end_mixer_no_dc2:
    neg        [w1++], [w5++]        ; q2(n+3) = -1 * x(n+7)
    return
    
    
    .global _sample_copy
_sample_copy:
    dec w0, w0
    repeat w0
    mov [w1++], [w2++]
    return
    