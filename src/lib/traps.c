#include <xc.h>
#include <utilities.h>

void __attribute__((__interrupt__, no_auto_psv)) _OscillatorFail(void)
{
        INTCON1bits.OSCFAIL = 0;        //Clear the trap flag
        set_error_led();
        while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _AddressError(void)
{
        INTCON1bits.ADDRERR = 0;        //Clear the trap flag
        set_error_led();
        while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _StackError(void)
{
        INTCON1bits.STKERR = 0;         //Clear the trap flag
        set_error_led();
        while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _MathError(void)
{
        INTCON1bits.MATHERR = 0;        //Clear the trap flag
        set_error_led();
        while (1);
}


void __attribute__((__interrupt__, no_auto_psv)) _DMACError(void)
{
		/* reset status bits, real app should check which ones */
		//DMACS0 = 0;
		
        INTCON1bits.DMACERR = 0;        //Clear the trap flag
        set_error_led();
        while (1);
}

