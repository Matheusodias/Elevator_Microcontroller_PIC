/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.6
        Device            :  PIC16F1827
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.30 and above
        MPLAB 	          :  MPLAB X 5.40	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set Im aliases
#define Im_TRIS                 TRISAbits.TRISA0
#define Im_LAT                  LATAbits.LATA0
#define Im_PORT                 PORTAbits.RA0
#define Im_ANS                  ANSELAbits.ANSA0
#define Im_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define Im_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define Im_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define Im_GetValue()           PORTAbits.RA0
#define Im_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define Im_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define Im_SetAnalogMode()      do { ANSELAbits.ANSA0 = 1; } while(0)
#define Im_SetDigitalMode()     do { ANSELAbits.ANSA0 = 0; } while(0)

// get/set Temp aliases
#define Temp_TRIS                 TRISAbits.TRISA1
#define Temp_LAT                  LATAbits.LATA1
#define Temp_PORT                 PORTAbits.RA1
#define Temp_ANS                  ANSELAbits.ANSA1
#define Temp_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define Temp_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define Temp_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define Temp_GetValue()           PORTAbits.RA1
#define Temp_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define Temp_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define Temp_SetAnalogMode()      do { ANSELAbits.ANSA1 = 1; } while(0)
#define Temp_SetDigitalMode()     do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set Dir aliases
#define Dir_TRIS                 TRISAbits.TRISA2
#define Dir_LAT                  LATAbits.LATA2
#define Dir_PORT                 PORTAbits.RA2
#define Dir_ANS                  ANSELAbits.ANSA2
#define Dir_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define Dir_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define Dir_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define Dir_GetValue()           PORTAbits.RA2
#define Dir_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define Dir_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define Dir_SetAnalogMode()      do { ANSELAbits.ANSA2 = 1; } while(0)
#define Dir_SetDigitalMode()     do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set RA3 procedures
#define RA3_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define RA3_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define RA3_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define RA3_GetValue()              PORTAbits.RA3
#define RA3_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define RA3_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define RA3_SetAnalogMode()         do { ANSELAbits.ANSA3 = 1; } while(0)
#define RA3_SetDigitalMode()        do { ANSELAbits.ANSA3 = 0; } while(0)

// get/set RA4 procedures
#define RA4_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define RA4_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define RA4_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define RA4_GetValue()              PORTAbits.RA4
#define RA4_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define RA4_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define RA4_SetAnalogMode()         do { ANSELAbits.ANSA4 = 1; } while(0)
#define RA4_SetDigitalMode()        do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set LedG aliases
#define LedG_TRIS                 TRISAbits.TRISA6
#define LedG_LAT                  LATAbits.LATA6
#define LedG_PORT                 PORTAbits.RA6
#define LedG_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define LedG_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define LedG_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define LedG_GetValue()           PORTAbits.RA6
#define LedG_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define LedG_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)

// get/set LedR aliases
#define LedR_TRIS                 TRISAbits.TRISA7
#define LedR_LAT                  LATAbits.LATA7
#define LedR_PORT                 PORTAbits.RA7
#define LedR_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define LedR_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define LedR_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define LedR_GetValue()           PORTAbits.RA7
#define LedR_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define LedR_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)

// get/set S1 aliases
#define S1_TRIS                 TRISBbits.TRISB0
#define S1_LAT                  LATBbits.LATB0
#define S1_PORT                 PORTBbits.RB0
#define S1_WPU                  WPUBbits.WPUB0
#define S1_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define S1_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define S1_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define S1_GetValue()           PORTBbits.RB0
#define S1_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define S1_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define S1_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define S1_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)

// get/set RB1 procedures
#define RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define RB1_GetValue()              PORTBbits.RB1
#define RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define RB1_SetPullup()             do { WPUBbits.WPUB1 = 1; } while(0)
#define RB1_ResetPullup()           do { WPUBbits.WPUB1 = 0; } while(0)
#define RB1_SetAnalogMode()         do { ANSELBbits.ANSB1 = 1; } while(0)
#define RB1_SetDigitalMode()        do { ANSELBbits.ANSB1 = 0; } while(0)

// get/set RB2 procedures
#define RB2_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define RB2_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define RB2_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define RB2_GetValue()              PORTBbits.RB2
#define RB2_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define RB2_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define RB2_SetPullup()             do { WPUBbits.WPUB2 = 1; } while(0)
#define RB2_ResetPullup()           do { WPUBbits.WPUB2 = 0; } while(0)
#define RB2_SetAnalogMode()         do { ANSELBbits.ANSB2 = 1; } while(0)
#define RB2_SetDigitalMode()        do { ANSELBbits.ANSB2 = 0; } while(0)

// get/set S2 aliases
#define S2_TRIS                 TRISBbits.TRISB3
#define S2_LAT                  LATBbits.LATB3
#define S2_PORT                 PORTBbits.RB3
#define S2_WPU                  WPUBbits.WPUB3
#define S2_ANS                  ANSELBbits.ANSB3
#define S2_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define S2_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define S2_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define S2_GetValue()           PORTBbits.RB3
#define S2_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define S2_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define S2_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define S2_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)
#define S2_SetAnalogMode()      do { ANSELBbits.ANSB3 = 1; } while(0)
#define S2_SetDigitalMode()     do { ANSELBbits.ANSB3 = 0; } while(0)

// get/set S3 aliases
#define S3_TRIS                 TRISBbits.TRISB4
#define S3_LAT                  LATBbits.LATB4
#define S3_PORT                 PORTBbits.RB4
#define S3_WPU                  WPUBbits.WPUB4
#define S3_ANS                  ANSELBbits.ANSB4
#define S3_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define S3_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define S3_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define S3_GetValue()           PORTBbits.RB4
#define S3_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define S3_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define S3_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define S3_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define S3_SetAnalogMode()      do { ANSELBbits.ANSB4 = 1; } while(0)
#define S3_SetDigitalMode()     do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set S4 aliases
#define S4_TRIS                 TRISBbits.TRISB5
#define S4_LAT                  LATBbits.LATB5
#define S4_PORT                 PORTBbits.RB5
#define S4_WPU                  WPUBbits.WPUB5
#define S4_ANS                  ANSELBbits.ANSB5
#define S4_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define S4_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define S4_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define S4_GetValue()           PORTBbits.RB5
#define S4_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define S4_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define S4_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define S4_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define S4_SetAnalogMode()      do { ANSELBbits.ANSB5 = 1; } while(0)
#define S4_SetDigitalMode()     do { ANSELBbits.ANSB5 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

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


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCBF0 pin functionality
 * @Example
    IOCBF0_ISR();
 */
void IOCBF0_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCBF0 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCBF0 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF0_SetInterruptHandler(MyInterruptHandler);

*/
void IOCBF0_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCBF0 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCBF0_SetInterruptHandler() method.
    This handler is called every time the IOCBF0 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF0_SetInterruptHandler(IOCBF0_InterruptHandler);

*/
extern void (*IOCBF0_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCBF0 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCBF0_SetInterruptHandler() method.
    This handler is called every time the IOCBF0 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF0_SetInterruptHandler(IOCBF0_DefaultInterruptHandler);

*/
void IOCBF0_DefaultInterruptHandler(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCBF3 pin functionality
 * @Example
    IOCBF3_ISR();
 */
void IOCBF3_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCBF3 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCBF3 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF3_SetInterruptHandler(MyInterruptHandler);

*/
void IOCBF3_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCBF3 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCBF3_SetInterruptHandler() method.
    This handler is called every time the IOCBF3 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF3_SetInterruptHandler(IOCBF3_InterruptHandler);

*/
extern void (*IOCBF3_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCBF3 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCBF3_SetInterruptHandler() method.
    This handler is called every time the IOCBF3 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF3_SetInterruptHandler(IOCBF3_DefaultInterruptHandler);

*/
void IOCBF3_DefaultInterruptHandler(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCBF4 pin functionality
 * @Example
    IOCBF4_ISR();
 */
void IOCBF4_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCBF4 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCBF4 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF4_SetInterruptHandler(MyInterruptHandler);

*/
void IOCBF4_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCBF4 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCBF4_SetInterruptHandler() method.
    This handler is called every time the IOCBF4 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF4_SetInterruptHandler(IOCBF4_InterruptHandler);

*/
extern void (*IOCBF4_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCBF4 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCBF4_SetInterruptHandler() method.
    This handler is called every time the IOCBF4 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF4_SetInterruptHandler(IOCBF4_DefaultInterruptHandler);

*/
void IOCBF4_DefaultInterruptHandler(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCBF5 pin functionality
 * @Example
    IOCBF5_ISR();
 */
void IOCBF5_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCBF5 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCBF5 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF5_SetInterruptHandler(MyInterruptHandler);

*/
void IOCBF5_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCBF5 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCBF5_SetInterruptHandler() method.
    This handler is called every time the IOCBF5 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF5_SetInterruptHandler(IOCBF5_InterruptHandler);

*/
extern void (*IOCBF5_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCBF5 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCBF5_SetInterruptHandler() method.
    This handler is called every time the IOCBF5 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCBF5_SetInterruptHandler(IOCBF5_DefaultInterruptHandler);

*/
void IOCBF5_DefaultInterruptHandler(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/