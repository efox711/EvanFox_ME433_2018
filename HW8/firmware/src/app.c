/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include "i2c_master_noint.h"
#include "ST7735.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
// Useful definitions
#define address 0b1101011

void drawChar(short x, short y, char* message, short color1, short color2) {
    char asciiRow = *message - 0x20;
    char pixels;
    int asciiColumn;
    for(asciiColumn = 0; asciiColumn < 5; asciiColumn++){
        pixels = ASCII[asciiRow][asciiColumn];
        int j;
        for(j = 0;j < 8;j++){
            if(((pixels >> j) & 1) == 1){
                LCD_drawPixel(x+asciiColumn,y+j,color1);
            }else{
                LCD_drawPixel(x+asciiColumn,y+j,color2);
            }
        }  
    }
}

void drawString(short x, short y, char* message, short color1, short color2) {
    int i = 0;
    
    while(message[i] != 0){
        drawChar(x+5*i,y,&message[i],color1,color2);
        i++;
    }
}

void drawProgressBar(short x, short y, short height, short length1, short color1, short length2, short color2){
    int xloc, yloc, currentHeight;
    for(xloc = x; xloc < x+length2; xloc++){
        for(currentHeight = 0; currentHeight < height+1; currentHeight++){
            for(yloc = y; yloc < y+currentHeight; yloc++){
                if(xloc - x < length1){
                    LCD_drawPixel(xloc,yloc,color1);                   
                }else{
                    LCD_drawPixel(xloc,yloc,color2);                   
                }

            }
        }
    }
}

void setRegister(char reg, char bits){
    i2c_master_start();
    i2c_master_send((address<<1)|0b00000000);
    i2c_master_send(reg);
    i2c_master_send(bits);
    i2c_master_stop();
}

void initIMU(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    i2c_master_setup();
    
    // turn on IMU
    // write to CTRL1_XL register, set smpl rate to 1.66 kHz, 2g sensitivity, 100Hz filter
    setRegister(0x10,0b10000010);
    //register - 10h
    //bits - 1000 00 10
    
    // write to CTRL2_G register, set smpl rate to 1.66 kHz, 1000 dps sensitivity
    setRegister(0x11,0b10001000);
    //register - 11h
    //bits - 1000 10 0 0
    
    // write to CTRL3_C - IF_INC bit, make it 1 to enable multiple read
    setRegister(0x12,0b00000100);
    //register - 12h
    //bits - 00000100
}   

void I2C_read_multiple(unsigned char reg, unsigned char * data, int length){
    i2c_master_start();
    i2c_master_send((address<<1)|0b00000000);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((address<<1)|0b00000001);
    int ii;
    for (ii = 0; ii < length;ii++){ 
        data[ii] = i2c_master_recv();
        if(ii == length-1){
            i2c_master_ack(1);
        }else{
            i2c_master_ack(0);
        }
    }
    i2c_master_stop();
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    /*__builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    //TRISB = 0b1111; //set for pushbutton  
    TRISAbits.TRISA4 = 0; //set A4 pin to output
    LATAbits.LATA4 = 1; //set A4 ON
    TRISBbits.TRISB4 = 1; //set B4 to input

    __builtin_enable_interrupts();*/
    
    __builtin_disable_interrupts();
  initIMU();                       // init I2C2, which we use as a master
  LCD_init();                       // init LCD
  LCD_clearScreen(WHITE);
  __builtin_enable_interrupts();
  
   TRISAbits.TRISA4 = 0; //set A4 pin to output
   LATAbits.LATA4 = 1; //set A4 OFF
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            /*while(1) {
            // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
            // remember the core timer runs at half the sysclk (CT runs at 24MHz)
                if(PORTBbits.RB4 == 1) {
                    _CP0_SET_COUNT(0);
                    LATAbits.LATA4 = 1;  //set LED ON
                    while(_CP0_GET_COUNT() < 12000) {
                        ;
                    }
                    LATAbits.LATA4 = 0;
                    _CP0_SET_COUNT(0);
                    while(_CP0_GET_COUNT() < 12000) {
                        ;
                    }
                }
            }*/
            
            char message[30];
    
    int ii,length = 14;
    unsigned char data[length];
    signed short adjData[(length/2)];
    
    int n=0;
    while(1){
        _CP0_SET_COUNT(0);
        I2C_read_multiple(0x20,data,length);
//        for (ii = 0; ii < length/2; ii++){ 
//            adjData[ii] = data[ii];//data[2*ii] || (data[2*ii+1]<<8);
//        }
        adjData[0] = (data[9]<<8)|data[8];
        adjData[1] = (data[11]<<8)|data[10];
        adjData[2] = (data[13]<<8)|data[12];
        
        sprintf(message,"X Accel: %d     ",adjData[0]);
        drawString(20, 20, message, BLACK, WHITE);
        sprintf(message,"Y Accel: %d     ",adjData[2]);
        drawString(20, 30, message, BLACK, WHITE);
//        sprintf(message,"%d",adjData[0]);
//        drawString(20, 40, message, RED, GREEN);
        if(n==0){
            LATAbits.LATA4 = 0; //set A4 OFF
            n=1;
        }else{
            LATAbits.LATA4 = 1;
            n=0;
        }
        
        signed int xval, yval;
        xval = (signed int) (100*adjData[0]/65535);
        yval = (signed int) (100*adjData[2]/65535);
        
        //draw bars
        //horizontal
        int xloc, yloc, currentHeight, length1 = xval+50, length2 = yval+50;
        for(xloc = 10; xloc < 10+100; xloc++){
            for(currentHeight = 0; currentHeight < 4; currentHeight++){
                for(yloc = 100; yloc < 100+currentHeight; yloc++){
                    if(xloc - 10 < length1){
                        LCD_drawPixel(xloc,yloc,BLACK);                   
                    }else{
                        LCD_drawPixel(xloc,yloc,RED);                   
                    }

                }
            }
        }
        //vertical
        for(yloc = 150; yloc > 150-99; yloc--){
            for(currentHeight = 0; currentHeight < 4; currentHeight++){
                for(xloc = 60; xloc < 60+currentHeight; xloc++){
                    if(150-yloc < length2){
                        LCD_drawPixel(xloc,yloc,BLACK);                   
                    }else{
                        LCD_drawPixel(xloc,yloc,RED);                   
                    }

                }
            }
        }
        //draw point
        LCD_drawPixel(xval+60,-yval+100, BLACK);
        
        while(_CP0_GET_COUNT() < 1200000){
            ;
        }
    }
        
            break;
        }

        /* TODO: implement your application state machine.*/
     

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
