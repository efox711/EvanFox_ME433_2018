#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

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
#define address 0b0100000

void initExpander(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    i2c_master_setup();
}

void setExpander(char pin, char level){
    i2c_master_start();
    i2c_master_send((address<1)|0b00000000);
    i2c_master_send(0x0A);
    i2c_master_send((level<pin)|0b00000000);
    i2c_master_stop();
}

char getExpander(void){
    
}

int main() {

  __builtin_disable_interrupts();
  initExpander();                       // init I2C2, which we use as a master
  __builtin_enable_interrupts();
  
  //set GP0 to be output, GP7 to be input
    i2c_master_start();
    i2c_master_send(address<1|0);
    i2c_master_send(0x00);
    i2c_master_send(0x80);
    i2c_master_stop();
    
   TRISAbits.TRISA4 = 0; //set A4 pin to output
   LATAbits.LATA4 = 1; //set A4 ON
  while(1) {
    setExpander(0,1); 
    i2c_master_start();
    i2c_master_send(0b01000000);
    i2c_master_send(0x0A);
    i2c_master_send(0b00000001);
    i2c_master_stop();
  }
  return 0;
}