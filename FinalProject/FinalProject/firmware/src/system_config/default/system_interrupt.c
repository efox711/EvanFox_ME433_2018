/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
 
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

 

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Timer4ISR(void) {
  // code for PI control goes here
    float Kp=1,Ki=0.1, K=0.05;
    int uL=0, uR=0, eL, eiL, eR, eiR, err;
    int velL = 0; // 1 rev/s, 700 timer pulses / rev, runs 500 times /s
    int velR = 0;
    
    rxVal = 240;
    err = rxVal - 240;
    if(err<0) { //slow down left motor, speed up right motor
        velL = velL + K*err;
        velR = velR - K*err/2;
        if(velL < 0) {
            velL = 0;
        }
        if(velR > 5) {
            velR = 5;
        }
    }else{
        velR = velR - K*err;
        velL = velL + K*err/2;
        if(velR < 0) {
            velR = 0;
        }
        if(velL > 5) {
            velL = 5;
        }
    }
    
    
    eL = velL - TMR3;
    eiL = eiL + eL;
    if(eiL > 500){
        eiL = 500;
    } else if (eiL < -500){
        eiL = -500;
    }
    uL = Kp * eL + Ki * eiL;
    OC4RS = OC4RS + uL;
    if(OC4RS < 0){
        OC4RS = 0;
    } else if (OC4RS > 2399) {
        OC4RS = 2399;
    }
    
    eR = velR - TMR5;
    eiR = eiR + eR;
    if(eiR > 500){
        eiR = 500;
    } else if (eiR < -500){
        eiR = -500;
    }
    uR = Kp * eR + Ki * eiR;
    OC1RS = OC1RS + uR;
    if(OC1RS < 0){
        OC1RS = 0;
    } else if (OC1RS > 2399) {
        OC1RS = 2399;
    }
//            
//    
//    OC1RS = 850;    //Right
//    OC4RS = 1000; //LEFT
//    //up to 2399
//    OC1RS = 0;
//    OC4RS = 0;
//    
    TMR5 = 0;
    TMR3 = 0;

    
  IFS0bits.T4IF = 0; // clear interrupt flag, last line
}

/*******************************************************************************
 End of File
*/
