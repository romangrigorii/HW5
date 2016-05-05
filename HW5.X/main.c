#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include "ILI9163C.h"
#include "i2c.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = ON // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_1 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7
#define SLAVE_ADDR_W 0b11010100
#define SLAVE_ADDR_R 0b11010101

void initSPI1(){
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
   
    SPI1BRG = 1; // baud rate unknown 
    SPI1STATbits.SPIROV = 0; // clear overflow
    SPI1CONbits.ON = 1; // SPI peripheral enable
    SPI1CONbits.CKE = 1; // the output data changes from clock active -> idle
    SPI1CONbits.MSTEN = 1; // Master Enable
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    RPB7Rbits.RPB7R = 0b0011; // Set B7 to be SS1
    RPA1Rbits.RPA1R = 0b0011; // Set A1 to be SDO1
}

void initI2C2(){
    __builtin_disable_interrupts();
    i2c_master_setup();
    __builtin_enable_interrupts();
}
void writeMess(char dataByte, char REG){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR_W);
    i2c_master_send(REG); 
    i2c_master_send(dataByte); 
    i2c_master_stop(); 
}

void ReBoot(){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR_W);
    i2c_master_send(0x12); 
    //i2c_master_send(0b00000101); 
    i2c_master_send(0b11000100); 
    i2c_master_stop();
}
unsigned char WhoAreYou(){
    unsigned char read;
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR_W);
    i2c_master_send(0x0F); 
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR_R);
    read = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return read;  
}
void InitA(void){
  i2c_master_start();
  i2c_master_send(SLAVE_ADDR_W);
  i2c_master_send(0x10);
  i2c_master_send(0b01100000); // 104Hz mode, (0100), 2g (00), 400Hz BW filter (00)
  i2c_master_stop();
}
void InitG(void){
  i2c_master_send(SLAVE_ADDR_W);
  i2c_master_send(0x11);
  i2c_master_send(0b01000010); // 104Hz mode, (0100), 125dps (001), necessary (0)
  i2c_master_stop();
}  
void InitT(void){
}
void ReadALL(signed short * DATA){
    unsigned char readL;
    unsigned char readH;
    int i;
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR_W);
    i2c_master_send(0x20); 
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR_R);
    for(i=0;i<7;i++){
        readL = i2c_master_recv();
        i2c_master_ack(0);
        readH = i2c_master_recv();
        if(i!=6){
           i2c_master_ack(0); 
        }
        DATA[i] = (signed short) ((unsigned short) readH)*256 + (unsigned short) readL;
    }
    i2c_master_ack(1);
    i2c_master_stop();
 }

char SPI1_IO(char write){
  SPI1BUF = write;
  while(SPI1STATbits.SPIBUSY) {
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, unsigned char voltage){
    char firstbyte = 0;
    char secondbyte = 0;
    if (channel){
        firstbyte = firstbyte + 128; //set Channel to B, A by default
    }
    firstbyte = firstbyte + 64; // buffered
    firstbyte = firstbyte + 32; // scaled by 1
    firstbyte = firstbyte + 16; // SHTDWN disabled
    firstbyte = firstbyte + (voltage>>4);
    secondbyte = (voltage<<4);
    CS = 0;
    char a = SPI1_IO(firstbyte);
    a = SPI1_IO(secondbyte);    
    CS = 1;
}

int main() {
    signed short AllData[7]; // temperature, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    __builtin_disable_interrupts();
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    //ODCBbits.ODCB4 = 0;
    SPI1_init(); 
    initI2C2();
    InitA();
    InitG();
    InitT();
    LCD_clearScreen(0x0000);
    LCD_init();
    char message[25];
    __builtin_enable_interrupts();   
    while(1){
        LCD_writeText("Hello World 1337!",28,32,BLUE,BLACK);
        ReadALL(AllData);
        LATAbits.LATA4 = !LATAbits.LATA4;
        _CP0_SET_COUNT(0);
        //LCD_clearScreen(BLACK);
        while(_CP0_GET_COUNT()<100000){
           }
        snprintf(message,25,"A_X: %1.6f", (double)(2*9.8*(AllData[4]))/32767);
        LCD_writeText(message,0,42,BLUE,BLACK);
        snprintf(message,25,"A_Y: %1.6f", (double)(2*9.8*(AllData[5]))/32767);
        LCD_writeText(message,0,52,BLUE,YELLOW);
        snprintf(message,25,"A_Z: %1.6f", (double)(2*9.8*(AllData[6]))/32767);
        LCD_writeText(message,0,62,BLUE,RED);
        
        snprintf(message,25,"G_X: %1.6f", (double)(2*9.8*(AllData[1]))/32767);
        LCD_writeText(message,0,72,BLUE,BLACK);
        snprintf(message,25,"G_Y: %1.6f", (double)(2*9.8*(AllData[2]))/32767);
        LCD_writeText(message,0,82,BLUE,YELLOW);
        snprintf(message,25,"G_Z: %1.6f", (double)(2*9.8*(AllData[3]))/32767);
        LCD_writeText(message,0,92,BLUE,RED);
        
        snprintf(message,25,"T: %f", (double) (100*(AllData[0]))/32767);
        LCD_writeText(message,0,102,BLUE,BLACK);       
        
        while(!PORTBbits.RB4){
        }
    }
}


