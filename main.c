/*
 * File:   main.c
 * Author: Martin
 *
 * Created on 2 de enero de 2017, 3:01 p.m.
 */

#pragma config FOSC = HS
#pragma config WDT = OFF, LVP = OFF, MCLRE = ON

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

unsigned char DC_str[5];

void rotarString(char* str, int length);
int cuentaLetras(char* str);
void cleanMessageBuffer();

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
        SetDCPWM1(DutyCycle);
        //lcd_put_blank(0,flag_newline, LCD_COLUMNS);
        lcd_gotoxy(0,0);
        lcd_puts("Duty Cycle: ");
        lcd_putnum(DutyCycle, 4, 0, 1);
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
    //check if the interrupt is caused by RX pin
    if(PIR1bits.RCIF == 1)
    {
        if(i<MAX_CHAR_BUFFER) //our buffer size
        {
            MessageBuffer[i] = ReadUSART(); //read the byte from rx register
            
            switch(MessageBuffer[i])
            {
                case 0x0D:  //check for return key
                    strncpy(DC_str, MessageBuffer,4); 
                    k = atoi(DC_str);
                    if(k >= 0 && k <= 1023)
                    {
                        DutyCycle = k;
                        putsUSART("Modificado el SetPoint a: ");
                        putsUSART(DC_str);
                        putsUSART(0x0D);
                    }
                    else
                    {
                        putsUSART("Error maquinola");
                        putsUSART(0x0D);
                    }
                    cleanMessageBuffer();
                    break;
                case 0x2B: //check for plus key
                    if(DutyCycle >= 0 && DutyCycle <= 1023)
                    {   
                        DutyCycle++;
                        itoa(DC_str, DutyCycle, 10);
                        putsUSART("Modificado el SetPoint a: ");
                        putsUSART(DC_str);
                        putsUSART(0x0D);
                    }
                    else
                    {
                        putsUSART("Error maquinola");
                        putsUSART(0x0D);
                    }
                    cleanMessageBuffer();
                    break;
                case 0x2D: //check for minus key
                    if(DutyCycle >= 0 && DutyCycle <= 1023)
                    {   
                        DutyCycle--;

                        putsUSART("Modificado el SetPoint a: ");
                        itoa(DC_str, DutyCycle, 10);
                        putsUSART(DC_str);
                        putsUSART(0x0D);
                    }
                    else
                    {
                        putsUSART("Error maquinola");
                        putsUSART(0x0D);
                    }  
                    cleanMessageBuffer();
                    break;                
                default:
                    i++;
                    
            }
            PIR1bits.RCIF = 0; // clear rx flag
        }
        else
        {
            putsUSART(MessageBuffer);
            cleanMessageBuffer();
        }
    }
    return;
}

void cleanMessageBuffer()
{
    for(;i>0;i--){
        MessageBuffer[i] = 0x00; //clear the array
    }
    i=0; //for sanity
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