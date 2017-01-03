#ifndef LCD_DRIVER_H
#define	LCD_DRIVER_H

#include <stdint.h>


/*
 * Platform dependant functions
 */
#include <xc.h>  
#define _XTAL_FREQ 4000000
//#include "../plat_cfg.h"

#define LCD_ENABLE_PULSE_SUSTAINED      200
#define LCD_ENABLE_PULSE_WAIT_OFF       500

#define LCD_WAIT_US(us)                 { __delay_us(us); }

//{PORTC = (PORTC & 0x0F) | (n << 4); }
// Atomic safe:
#define LCD_SET_NIBBLE(n)               { LATDbits.LATD4 = n & 1; LATDbits.LATD5 = (n & 2) >> 1; LATDbits.LATD6 = (n & 4) >> 2; LATDbits.LATD7 = (n & 8) >> 3; }
//#define LCD_SET_NIBBLE(n)               { PORTAbits.RA1 = n & 1; PORTAbits.RA2 = (n & 2) >> 1; PORTAbits.RA3 = (n & 4) >> 2; PORTAbits.RA4 = (n & 8) >> 3; }
#define LCD_PULSE_ENABLE()              { LATDbits.LATD3 = 1; LCD_WAIT_US(LCD_ENABLE_PULSE_SUSTAINED); LATDbits.LATD3 = 0; LCD_WAIT_US(LCD_ENABLE_PULSE_WAIT_OFF); }
#define LCD_SET_RS(rs)                  { LATDbits.LATD2 = rs; }

#define LCD_CHIP_MODEL                  LCD_HD44780
#define LCD_COLUMNS                     16
#define LCD_ROWS                        2

/*
 * LCD driver local defines
 */
#define LCD_KS0066              1
#define LCD_HD44780             0

#define LCD_WRITE_FUNCTION      0
#define LCD_WRITE_DATA          1

#if(LCD_CHIP_MODEL == LCD_HD44780 || LCD_CHIP_MODEL == LCD_KS0066)
    #define LCD_ROW_1               0x80
    #define LCD_ROW_2               LCD_ROW_1 + 64
    #define LCD_ROW_3               LCD_ROW_1 + 20
    #define LCD_ROW_4               LCD_ROW_1 + 84
#endif

/* First Group */
/* Function set */
#define	LCD_DATA_LENGTH_4		0x20
#define	LCD_DATA_LENGTH_8		0x30
#define LCD_ONE_LINES_MODE		0x20
#define	LCD_TWO_LINES_MODE  	0x28
#define LCD_MINIMAL_FONT_SIZE 	0x20
#define LCD_MAX_FONT_SIZE		0x24

/* Second Group */
/* Display and cursor ON/OFF Control */
#define LCD_DISPLAY_OFF			0x08
#define LCD_DISPLAY_ON			0x0C
#define	LCD_CURSOR_OFF			0x08
#define	LCD_CURSOR_ON			0x0A
#define	LCD_CURSOR_BLINK_OFF	0x08
#define	LCD_CURSOR_BLINK_ON		0x09

/* Third Group*/
/* Increment mode and LCD shift */
#define LCD_CURSOR_SHIFT_RIGHT	0x14
#define LCD_CURSOR_SHIFT_LEFT	0x10
#define	LCD_WITHOUT_SHIFT       0x10
#define LCD_SHIFT               0x18


/* Fourth Group */
/* LCD Useful Functions*/
#define LCD_CLEAR_LCD			0x01
#define LCD_CURSOR_RETURN_HOME  0x02
#define LCD_SET_CGRAM_ADDRESS   0x40
#define LCD_SET_DDRAM_ADDRESS	0x80

/*
 * LCD driver function exports prototypes
 */
void lcd_init(void);

void lcd_gotoxy(uint8_t x, uint8_t y);

//void lcd_putch(char c)    { lcd_write_byte(c, LCD_WRITE_DATA); )
#define lcd_putch(c)        {lcd_write_byte(c, LCD_WRITE_DATA);}

//void lcd_clear(void)      {lcd_write_byte(LCD_CLEAR_LCD, LCD_WRITE_FUNCTION); LCD_WAIT_US(5000);}
#define lcd_clear()         {lcd_write_byte(LCD_CLEAR_LCD, LCD_WRITE_FUNCTION); LCD_WAIT_US(5000);}

void lcd_putnum(uint16_t num, 
                uint8_t len,
                uint8_t x, 
                uint8_t y);


void lcd_puts(char * str);
void lcd_puts_const(const char * str);
void lcd_put_blank(uint8_t x, uint8_t y, uint8_t size);

void lcd_set_cgram(uint8_t addr, const char data[]);


void lcd_write_nibble(uint8_t nibble, uint8_t is_data);
void lcd_write_byte(uint8_t byte, uint8_t is_data);




#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* LCD_DRIVER_H */

