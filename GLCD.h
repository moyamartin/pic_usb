/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "system.h"
// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 
/* ----- Definitions concerning LCD internal memory  ------ */

#define glcd_G_BASE 0x0200            // base address of graphics memory
#define glcd_T_BASE 0x0000            // base address of text memory
#define glcd_BYTES_PER_ROW 30         // how many bytes per row on screen

#define glcd_wr_high() LATCbits.LATC0 = 1;
#define glcd_wr_low()  LATCbits.LATC0 = 0;

#define glcd_rd_high() LATCbits.LATC1 = 1;
#define glcd_rd_low()  LATCbits.LATC1 = 0;

#define glcd_ce_high() LATCbits.LATC2 = 1;
#define glcd_ce_low()  LATCbits.LATC2 = 0;

#define glcd_cd_high() LATCbits.LATC6 = 1;
#define glcd_cd_low()  LATCbits.LATC6 = 0;

#define glcd_rst_high() LATCbits.LATC7 = 1;
#define glcd_rst_low()  LATCbits.LATC7 = 0;

#define glcd_SET_DATADIR_OUT()	(TRISD = 0x00)
#define glcd_SET_DATADIR_IN()	(TRISD = 0xFF)

#define glcd_XMAX 128        // limits of (x,y) LCD graphics drawing
#define glcd_XMIN 0
#define glcd_YMAX 64
#define glcd_YMIN 0

#define glcd_WAIT_US(us)                 { __delay_us(us); }
#define glcd_WAIT_MS(ms)                 { __delay_ms(ms); }
/* ----------------------------------------------------------- */
void glcd_dput(int byte); 			 // write data byte to LCD module
int  glcd_dget(void);    			 // get data byte from LCD module
int  glcd_sget(void);  			 // check LCD display status pbrt
void glcd_cput(int byte);			 // write command byte to LCD module
void glcd_setup();			 // make sure control lines are at correct levels
void glcd_init();     			 // initialize LCD memory and display modes
void glcd_print(int x,int y,const char *string);  	 // send string of characters to LCD
void glcd_print_ram(int x,int y,char *string);  	 // send string of characters to LCD
void glcd_set_address(unsigned int addr);
void glcd_clear_graph();    		 // clear graphics memory of LCD
void glcd_clear_text();  		 // clear text memory of LCD
void glcd_xy(int x, int y); 		 // set memory pointer to (x,y) position (text)
void glcd_clrpixel(int column, int row);  // set single pixel in 240x64 array
void glcd_setpixel(int column, int row);  // set single pixel in 240x64 array
void glcd_pixel(int column, int row,char show1);

void glcd_show(const char *,int,int);
void glcd_line(int x1, int y1, int x2, int y2, unsigned char show);
void glcd_circle(int x, int y, int radius, unsigned char show);
void glcd_circle_half(int x, int y, int radius, unsigned char show);
void glcd_box(int x1, int y1, int x2, int y2, unsigned char show);
//void glcd_degree_line(int x, int y, int degree, int inner_radius, int outer_radius, unsigned char show);
//void glcd_degree_line_bold(int x, int y, int degree, int inner_radius, int outer_radius, unsigned char show);
void glcd_fill(int x1, int y1, int x2, int y2, unsigned char persent,char first);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

