#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include "i2c_master_noint.h"
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


//void I2C_read_multiple(unsigned char address, unsigned char register, unsigned char * data, int length){
//    ;
//}

int main() {
  __builtin_disable_interrupts();
  initIMU();                       // init I2C2, which we use as a master
  LCD_init();                       // init LCD
  LCD_clearScreen(WHITE);
  __builtin_enable_interrupts();
  
   TRISAbits.TRISA4 = 0; //set A4 pin to output
   LATAbits.LATA4 = 1; //set A4 OFF
    
    i2c_master_start();
    i2c_master_send((address<<1)|0b00000000);
    i2c_master_send(0x0F);
    i2c_master_restart();
    i2c_master_send((address<<1)|0b00000001);
    char input = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    char message[30];
    sprintf(message,"%d",input);
    drawString(50, 50, message, RED, GREEN);
    
    return(1);
}

//    i2c_master_start();
//    i2c_master_send((address<<1)|0b00000000);
//    i2c_master_send(0x09);
//    i2c_master_restart();
//    i2c_master_send((address<<1)|0b00000001);
//    char input = i2c_master_recv();
//    i2c_master_ack(1);
//    i2c_master_stop();
//    return input;