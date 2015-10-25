#include "mcc_generated_files/mcc.h"

// Main application
void main()
{
    // initialize the device
    SYSTEM_Initialize();
    
    // TODO: Figure out why MCC Configuration won't let me init PWM DC at 0%.
    // I actually do want to do that, then turn on PWM later when commanded by
    // I2C. I might have to manually setup the PWM hardware (which doesn't look
    // hard).
    // The code below might do it, since MCC Config currently initializes PWM
    // as disabled.
    // Test! As in, with a scope. And/or simulator.
    
    // TODO: It looks like the MCC Config generated code is designed to be edited
    // So I can probably just put whatever value I want in there for DC %.
    
    // TODO: mcc_generated_files/i2c.c allocating more RAM than the chip has.
    // That is why I think those files are supposed to be edited.
    // So, edit them! Talk about a dumb sample...
    
    // Set PWM to 0%, then enable
    PWM1_LoadDutyValue(0x0000);
    PWM2_LoadDutyValue(0x0000);
    PWM3_LoadDutyValue(0x0000);
    PWM1EN = 1;
    PWM2EN = 1;
    PWM3EN = 1;

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1)
    {
        // Add your application code
    }
}
