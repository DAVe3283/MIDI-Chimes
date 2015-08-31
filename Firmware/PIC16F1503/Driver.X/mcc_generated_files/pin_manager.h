/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1503
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

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
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set channel_AN0 aliases
#define channel_AN0_TRIS               TRISA0
#define channel_AN0_LAT                LATA0
#define channel_AN0_PORT               RA0
#define channel_AN0_WPU                WPUA0
#define channel_AN0_ANS                ANSA0
#define channel_AN0_SetHigh()    do { LATA0 = 1; } while(0)
#define channel_AN0_SetLow()   do { LATA0 = 0; } while(0)
#define channel_AN0_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define channel_AN0_GetValue()         RA0
#define channel_AN0_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define channel_AN0_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define channel_AN0_SetPullup()    do { WPUA0 = 1; } while(0)
#define channel_AN0_ResetPullup()   do { WPUA0 = 0; } while(0)
#define channel_AN0_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define channel_AN0_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set channel_AN1 aliases
#define channel_AN1_TRIS               TRISA1
#define channel_AN1_LAT                LATA1
#define channel_AN1_PORT               RA1
#define channel_AN1_WPU                WPUA1
#define channel_AN1_ANS                ANSA1
#define channel_AN1_SetHigh()    do { LATA1 = 1; } while(0)
#define channel_AN1_SetLow()   do { LATA1 = 0; } while(0)
#define channel_AN1_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define channel_AN1_GetValue()         RA1
#define channel_AN1_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define channel_AN1_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define channel_AN1_SetPullup()    do { WPUA1 = 1; } while(0)
#define channel_AN1_ResetPullup()   do { WPUA1 = 0; } while(0)
#define channel_AN1_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define channel_AN1_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set PWM3 aliases
#define PWM3_TRIS               TRISA2
#define PWM3_LAT                LATA2
#define PWM3_PORT               RA2
#define PWM3_WPU                WPUA2
#define PWM3_ANS                ANSA2
#define PWM3_SetHigh()    do { LATA2 = 1; } while(0)
#define PWM3_SetLow()   do { LATA2 = 0; } while(0)
#define PWM3_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define PWM3_GetValue()         RA2
#define PWM3_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define PWM3_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define PWM3_SetPullup()    do { WPUA2 = 1; } while(0)
#define PWM3_ResetPullup()   do { WPUA2 = 0; } while(0)
#define PWM3_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define PWM3_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set channel_AN3 aliases
#define channel_AN3_TRIS               TRISA4
#define channel_AN3_LAT                LATA4
#define channel_AN3_PORT               RA4
#define channel_AN3_WPU                WPUA4
#define channel_AN3_ANS                ANSA4
#define channel_AN3_SetHigh()    do { LATA4 = 1; } while(0)
#define channel_AN3_SetLow()   do { LATA4 = 0; } while(0)
#define channel_AN3_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define channel_AN3_GetValue()         RA4
#define channel_AN3_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define channel_AN3_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define channel_AN3_SetPullup()    do { WPUA4 = 1; } while(0)
#define channel_AN3_ResetPullup()   do { WPUA4 = 0; } while(0)
#define channel_AN3_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define channel_AN3_SetDigitalMode()   do { ANSA4 = 0; } while(0)
// get/set IO_RA5 aliases
#define IO_RA5_TRIS               TRISA5
#define IO_RA5_LAT                LATA5
#define IO_RA5_PORT               RA5
#define IO_RA5_WPU                WPUA5
#define IO_RA5_SetHigh()    do { LATA5 = 1; } while(0)
#define IO_RA5_SetLow()   do { LATA5 = 0; } while(0)
#define IO_RA5_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define IO_RA5_GetValue()         RA5
#define IO_RA5_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define IO_RA5_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define IO_RA5_SetPullup()    do { WPUA5 = 1; } while(0)
#define IO_RA5_ResetPullup()   do { WPUA5 = 0; } while(0)
// get/set SCL aliases
#define SCL_TRIS               TRISC0
#define SCL_LAT                LATC0
#define SCL_PORT               RC0
#define SCL_ANS                ANSC0
#define SCL_SetHigh()    do { LATC0 = 1; } while(0)
#define SCL_SetLow()   do { LATC0 = 0; } while(0)
#define SCL_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define SCL_GetValue()         RC0
#define SCL_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define SCL_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

#define SCL_SetAnalogMode()   do { ANSC0 = 1; } while(0)
#define SCL_SetDigitalMode()   do { ANSC0 = 0; } while(0)
// get/set SDA aliases
#define SDA_TRIS               TRISC1
#define SDA_LAT                LATC1
#define SDA_PORT               RC1
#define SDA_ANS                ANSC1
#define SDA_SetHigh()    do { LATC1 = 1; } while(0)
#define SDA_SetLow()   do { LATC1 = 0; } while(0)
#define SDA_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define SDA_GetValue()         RC1
#define SDA_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define SDA_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

#define SDA_SetAnalogMode()   do { ANSC1 = 1; } while(0)
#define SDA_SetDigitalMode()   do { ANSC1 = 0; } while(0)
// get/set IO_RC2 aliases
#define IO_RC2_TRIS               TRISC2
#define IO_RC2_LAT                LATC2
#define IO_RC2_PORT               RC2
#define IO_RC2_ANS                ANSC2
#define IO_RC2_SetHigh()    do { LATC2 = 1; } while(0)
#define IO_RC2_SetLow()   do { LATC2 = 0; } while(0)
#define IO_RC2_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IO_RC2_GetValue()         RC2
#define IO_RC2_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IO_RC2_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define IO_RC2_SetAnalogMode()   do { ANSC2 = 1; } while(0)
#define IO_RC2_SetDigitalMode()   do { ANSC2 = 0; } while(0)
// get/set PWM2 aliases
#define PWM2_TRIS               TRISC3
#define PWM2_LAT                LATC3
#define PWM2_PORT               RC3
#define PWM2_ANS                ANSC3
#define PWM2_SetHigh()    do { LATC3 = 1; } while(0)
#define PWM2_SetLow()   do { LATC3 = 0; } while(0)
#define PWM2_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define PWM2_GetValue()         RC3
#define PWM2_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define PWM2_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define PWM2_SetAnalogMode()   do { ANSC3 = 1; } while(0)
#define PWM2_SetDigitalMode()   do { ANSC3 = 0; } while(0)
// get/set PWM1 aliases
#define PWM1_TRIS               TRISC5
#define PWM1_LAT                LATC5
#define PWM1_PORT               RC5
#define PWM1_SetHigh()    do { LATC5 = 1; } while(0)
#define PWM1_SetLow()   do { LATC5 = 0; } while(0)
#define PWM1_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define PWM1_GetValue()         RC5
#define PWM1_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define PWM1_SetDigitalOutput()   do { TRISC5 = 0; } while(0)


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */