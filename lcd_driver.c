
#include "lcd_driver.h"
/*
 * LCD Rows addresses (each address depends on the selected controller)
 */
const uint8_t LCD_ROW_ADDRESS[] = {LCD_ROW_1, LCD_ROW_2, LCD_ROW_3, LCD_ROW_4};

uint8_t     lcd_x;
uint8_t     lcd_y;

void lcd_init(void)
{
    //RUTINA DE INICIALIZACION EN 4 BITS
    //RETARDO >15ms
    LCD_WAIT_US(15000);
    
    //ESCRIBIR 0011
    lcd_write_nibble(0x3, LCD_WRITE_FUNCTION);
    //RETARDO > 4.1ms
    LCD_WAIT_US(5000);
    //ESCRIBIR 0011
    lcd_write_nibble(0x3, LCD_WRITE_FUNCTION);
    //RETARDO > 100us
    LCD_WAIT_US(5000); //LCD_WAIT_US(200);
    //ESCRIBIR 0011
    lcd_write_nibble(0x3, LCD_WRITE_FUNCTION); 
    //Teoricamente este retardo no es necesario
    LCD_WAIT_US(5000);
    //ESCRIBIR 0010
    lcd_write_nibble(0x2, LCD_WRITE_FUNCTION);
    
    /*lcd_write_byte(LCD_DATA_LENGTH_4 | LCD_TWO_LINES_MODE | LCD_MAX_FONT_SIZE, LCD_WRITE_FUNCTION); LCD_WAIT_US(5000);
    lcd_write_byte(LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_CURSOR_BLINK_OFF, LCD_WRITE_FUNCTION);     LCD_WAIT_US(5000);
    lcd_write_byte(LCD_CLEAR_LCD, LCD_WRITE_FUNCTION);                                              LCD_WAIT_US(5000);
    lcd_write_byte(LCD_WITHOUT_SHIFT, LCD_WRITE_FUNCTION);                                          LCD_WAIT_US(5000);*/
    
    //Teoricamente aca no tienen que haber mas retardos...
    //Función
    lcd_write_byte(0x28, LCD_WRITE_FUNCTION); 
    LCD_WAIT_US(5000);
    //Apaga Display
    lcd_write_byte(0x0c, LCD_WRITE_FUNCTION); 
    LCD_WAIT_US(5000);
    //Borra Display
    lcd_write_byte(0x01, LCD_WRITE_FUNCTION); 
    LCD_WAIT_US(5000); 
    //Modo de Entrada
    lcd_write_byte(0x6, LCD_WRITE_FUNCTION); 
    LCD_WAIT_US(5000); 
    
    //FIN INICIALIZACION
}

void lcd_gotoxy(uint8_t x, 
                uint8_t y)
{
    uint8_t addr;
    
    if ((y > 3) || (x > 19))
    {
        return; /* bad argument */
    }
    lcd_x = x;
    lcd_y = y;
    addr = x;
    addr += LCD_ROW_ADDRESS[y];
    LCD_WAIT_US(1000);
    lcd_write_byte(addr, LCD_WRITE_FUNCTION);
    LCD_WAIT_US(1000);
    lcd_write_byte(addr, LCD_WRITE_FUNCTION);
    LCD_WAIT_US(1000);
}



void lcd_putnum(uint16_t num, 
                uint8_t len,
                uint8_t x, 
                uint8_t y)
{
    uint16_t    r;
    uint8_t     dec;
    
    x+=len;
    while (len)
    {
        r = num/10;
        dec = num - r*10;
        x--;
        len--;
        lcd_gotoxy(x,y);
        lcd_putch(dec + 0x30);
        num = r;
    }
}



void lcd_puts_const(const char * str)
{
    while (*str != 0)
    {
        if (*str != '\n')
        {
           lcd_putch(*str); 
        }        
        
        if ((++lcd_x == LCD_COLUMNS) || (*str == '\n'))
        {
            if (++lcd_y == LCD_ROWS)
            {
                lcd_gotoxy(0,0);
            }
            else
            {
                lcd_gotoxy(0, lcd_y);
            }
        }
        str++;
    }
}

void lcd_write_nibble(uint8_t nibble, 
                      uint8_t is_data)
{
    LCD_SET_RS(is_data);
    LCD_WAIT_US(100);
    LCD_SET_NIBBLE(nibble);
    LCD_PULSE_ENABLE();
}

void lcd_write_byte(uint8_t byte, 
                    uint8_t is_data)
{
    lcd_write_nibble(byte >> 4, is_data);
    lcd_write_nibble(byte, is_data);
}