.global _write_pwm_protected
_write_pwm_protected:
    push w10
    push w11
    mov #0xABCD, w10
    mov #0x4321, w11
    mov w10, PWMKEY
    mov w11, PWMKEY
    mov w0, [w1]
    pop w11
    pop w10
    return 
    
.global _delay_n_instructions
    _delay_n_instructions:
    sub w0, #1, w0
    repeat w0
    nop
    return
    