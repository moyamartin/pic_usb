/*
 * File:   main.c
 * Author: Martin
 *
 * Created on 2 de enero de 2017, 3:01 p.m.
 */

#pragma config FOSC = HS
#pragma config WDT = OFF, LVP = OFF, MCLRE = ON

#include <xc.h>
#include "string.h"
#include "pwm.h"
#include "usart.h"
#include "delays.h"
#include "lcd_driver.h"

#define _XTAL_FREQ 12000000
#define MAX_CHAR_BUFFER 200


char c;
char flag_newline = 0;
int i=0;
int k;
int cantidadLetrasUP = 0; 
int cantidadLetrasDOWN = 0 ;
unsigned int DutyCycle = 10;

unsigned char MessageBuffer[MAX_CHAR_BUFFER];
unsigned char lcd_upper_row_buffer[MAX_CHAR_BUFFER];
unsigned char lcd_lower_row_buffer[MAX_CHAR_BUFFER];

unsigned char lcd_lower_display_buffer[LCD_COLUMNS];
unsigned char lcd_upper_display_buffer[LCD_COLUMNS];

void rotarString(char* str, int length);
int cuentaLetras(char* str);

void interrupt ISR(void);

void main(void) {
    
    unsigned char config = 0, spbrg = 0;
    
    TRISC = 0b10000000;
    TRISD = 0b00000000;
    
    
    // UART Config & Initialization
    // Close USART port if already opened
    CloseUSART();
    config = USART_TX_INT_OFF& USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_BRGH_HIGH;
    spbrg = 77;
    OpenUSART(config, spbrg);
    RCIF = 0; //reset RX pin flag
    RCIP = 0; //Not high priority
    RCIE = 1; //Enable RX interrupt
    PEIE = 1; //Enable pheripheral interrupt (serial port is a pheripheral)
    PIR1bits.RCIF = 0; //clear rx flag
    ei();       //remember the master switch for interrupt?

    // PWM Configuration
    
    T2CON = 0b00000111 ;
    OpenPWM1(0b10111011);
    
    SetDCPWM1(DutyCycle);
    
    // LCD Init
    lcd_init();
    lcd_clear();
    lcd_gotoxy(0,0);
    while(1)
    {
        //lcd_put_blank(0,flag_newline, LCD_COLUMNS);
        lcd_gotoxy(0,0);
        strncpy(lcd_upper_display_buffer, lcd_upper_row_buffer, LCD_COLUMNS);
        lcd_puts(lcd_upper_display_buffer);
        
        lcd_gotoxy(0,1);
        strncpy(lcd_lower_display_buffer, lcd_lower_row_buffer, LCD_COLUMNS);
        lcd_puts(lcd_lower_display_buffer);
        if(cantidadLetrasUP > LCD_COLUMNS)
        {
            rotarString(lcd_upper_row_buffer, cantidadLetrasUP);
        }
        if(cantidadLetrasDOWN > LCD_COLUMNS)
        {
            rotarString(lcd_lower_row_buffer, cantidadLetrasDOWN);
        }
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        __delay_ms(50);
        LATDbits.LATD0 =~ LATDbits.LATD0;
        if(DutyCycle < 1020)
            DutyCycle += 10;
        else
            DutyCycle = 10;
       
        SetDCPWM1(DutyCycle);
    }
}

void interrupt ISR()
{
    //check if the interrupt is caused by RX pin
    if(PIR1bits.RCIF == 1)
    {
        if(i<MAX_CHAR_BUFFER) //our buffer size
        {
            
            MessageBuffer[i] = ReadUSART(); //read the byte from rx register
            if(MessageBuffer[i] == 0x0D) //check for return key
            {
                lcd_put_blank(0,flag_newline, LCD_COLUMNS);
                switch (flag_newline)
                {
                    case 0:
                        for(;k>0;k--)
                            lcd_upper_row_buffer[k] = 0x00; //clear the array
                        k = 0;
                        strcpy(lcd_upper_row_buffer, MessageBuffer);
                        cantidadLetrasUP = cuentaLetras(MessageBuffer);
                        lcd_upper_row_buffer[cantidadLetrasUP - 1] = ' ';
                        break;
                        
                    case 1:
                        for(;k>0;k--)
                            lcd_lower_row_buffer[k] = 0x00; //clear the array
                        k = 0;
                        strcpy(lcd_lower_row_buffer, MessageBuffer);
                        cantidadLetrasDOWN = cuentaLetras(MessageBuffer);
                        lcd_lower_row_buffer[cantidadLetrasDOWN - 1] = ' ';
                        break;
                }               
                putsUSART(MessageBuffer);
                
                LATDbits.LATD1 =~ LATDbits.LATD1;
                
                
                flag_newline^=1;
                for(;i>0;i--)
                    MessageBuffer[i] = 0x00; //clear the array
                i=0; //for sanity
                return;
            }
            i++;
            PIR1bits.RCIF = 0; // clear rx flag
        }
        else
        {
            
            putsUSART(MessageBuffer);
            for(;i>0;i--)
                MessageBuffer[i] = 0x00; //clear the array
            i=0; //for sanity
            return;
        }
    }
}


int cuentaLetras(char* str)
{
    int j=0;
    while(str[j]!= 0x0D)
    {
        j++;
    }
    return j+1;
}

void rotarString(char* str, int length)
{
    int j;
    char aux;
    aux = str[0];
    for(j = 0; j<length - 1; j++)
    {
        str[j] = str[j+1];
    }
    str[length - 1] = aux;
}