#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>        //math functions
#include<stdio.h>
#include "ST7735.h"

// DEVCFG0
#pragma config DEBUG = 0b10 // no debugging
#pragma config JTAGEN = 0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b1111111 // no write protect
#pragma config BWP = 1 // no boot write protect
#pragma config CP = 1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0 // turn off secondary oscillator
#pragma config IESO = 0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 1 // disable secondary osc
#pragma config FPBDIV = 0 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b11 // do not enable clock switch
#pragma config WDTPS = 0 // use slowest wdt
#pragma config WINDIS = 1 // wdt no window mode
#pragma config FWDTEN = 0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0 // USB clock on

// DEVCFG3
#pragma config USERID = 0b0000000000000001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 1 // USB BUSON controlled by USB module

//#define CS LATBbits.LATB7       // chip select pin


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
int main() {
    LCD_init();
    LCD_clearScreen(WHITE);
    float fps;
    while(1){
        int num;
        for(num = 0;num < 101; num++) {
            _CP0_SET_COUNT(0);
            char message[30];
            
            sprintf(message,"Hello world %d!   ", num);
            drawString(28,32,message,BLACK,WHITE);
            drawProgressBar(12,50,10, num, RED, 100, BLUE);
            
            fps = _CP0_GET_COUNT()/24000
            while(_CP0_GET_COUNT() < 2400000){;}
        }
    }
      
    return 1;
    
}