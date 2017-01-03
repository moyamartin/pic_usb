/*
 * File:   main.c
 * Author: Martin
 *
 * Created on 2 de enero de 2017, 3:01 p.m.
 */


#pragma config FOSC = HS
#pragma config WDT = OFF, LVP = OFF, MCLRE = ON

#include <xc.h>
#include "usart.h"
#include "delays.h"
#include "lcd_driver.h"

#define _XTAL_FREQ 12000000

char c;
void interrupt ISR(void);

void main(void) {
    
    unsigned char config = 0, spbrg = 0;
    
    TRISC = 0b10000000;
    TRISD = 0b00000000;
    
    // Close USART port if already opened
    CloseUSART();
    
    config = USART_TX_INT_OFF& USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_BRGH_HIGH;
    spbrg = 77;
    
    OpenUSART(config, spbrg);
    
    //compare with the table above
    RCIF = 0; //reset RX pin flag
    RCIP = 0; //Not high priority
    RCIE = 1; //Enable RX interrupt
    PEIE = 1; //Enable pheripheral interrupt (serial port is a pheripheral)
    PIR1bits.RCIF = 0; //clear rx flag
    ei();       //remember the master switch for interrupt?

    lcd_init();
    lcd_clear();
    lcd_gotoxy(0,0);
    lcd_puts_const("Waska");
    
    while(1){
        //while(BusyUSART());
        //putrsUSART("Hello World\n");
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        LATDbits.LATD0 =~ LATDbits.LATD0;
    }
}

void interrupt ISR()
{
    if(PIR1bits.RCIF == 1)
    {
        PIR1bits.RCIF = 0; //clear rx flag
        c = ReadUSART();
        while(BusyUSART());
        WriteUSART(c);
        LATDbits.LATD1 =~ LATDbits.LATD1;
    }
}
