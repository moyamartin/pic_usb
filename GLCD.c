#include <stdio.h>
#include <stdlib.h>     // rand()
#include <string.h>     // strlen()
#include <math.h>
#include <pic18f4550.h>       // cos(),sin()
#include "GLCD.h"
/*
 *  FG                (1)  frame ground
 *  GND               (2)  ground 
 *  VDD               (3)  LCD logic supply  +5 DC
 *  VO                (4)  Contrast
 *  /WR               (5)  <--> 
 *  /RD               (6)  <--> 
 *  /CE or /CS        (7)  <--> 
 *  CD or RS          (8)  <-->
 *  /RST              (9)  <-->   

 *  D0                (10) <--> 
 *  D1                (11) <--> 
 *  D2                (12) <--> 
 *  D3                (13) <--> 
 *  D4                (14) <--> 
 *  D5                (15) <--> 
 *  D6                (16) <--> 
 *  D7                (17) <--> 

 *  FS                (18)           font select       HI : narrow            LOW : Wide   ( normal )
                                                       40 with 6x8 font       30 with 8x8 font 
 *  Vout              (19)           Variable Resistor with GND and centre with VO ( Pin 4 )
 *  BLA               (20)           Back light +
 *  BLK               (21)           Back light -
 *  DSPON             (22)           I don't Know   but i think means display ON or ...
 */



/***************************************
									//**
           PIN DEFINITIONS			//**
									//**
****************************************/

									//**
//**************************************
//**************************************

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





void glcd_set_address(unsigned int addr){
glcd_dput(addr%256);
glcd_dput(addr>>8);
glcd_cput(0x24);
}



void glcd_clear_graph()    // clear graphics memory of LCD
{
int i;
glcd_set_address(glcd_G_BASE);     // addrptr at address G_BASE1
for (i=0;i<3840;i++) {   // must be  3840 ?????????????????     5120
	glcd_dput(0); glcd_cput(0xc0);               // write data, inc ptr.
	}	 // end for(i)
} // end glcd_clear_graph()

//----------------------------------------------------------------

void glcd_clear_text()
{
 int i;   
 glcd_set_address(glcd_T_BASE);     // addrptr at address T_BASE1

 for (i=0;i<480;i++) {  //  must be 480   ???????????     640  
      glcd_dput(0); glcd_cput(0xc0);               // write data, inc ptr.
 } // end for(i)

} // glcd_clear_text1()

//-------------------------------------------------------------------------------


void glcd_print(int x,int y, const char *string)  // send string of characters to LCD
{
int i;
int c;
glcd_xy(x,y);
  for (i=0;string[i]!=0;i++) {
      c = string[i] - 0x20;     // convert ASCII to LCD char address
      if (c<0) c=0;
      glcd_dput(c);
      glcd_cput(0xc0);               // write character, increment memory ptr.
  } // end for

} // end glcd_string

//--------------------------------------------------------------------------------




void glcd_print_ram(int x,int y,char *string)  // send string of characters to LCD
{
int i;
int c;
glcd_xy(x,y);
  for (i=0;string[i]!=0;i++) {
      c = string[i] - 0x20;     // convert ASCII to LCD char address
      if (c<0) c=0;
      glcd_dput(c);
      glcd_cput(0xc0);               // write character, increment memory ptr.
  } // end for

} // end glcd_string

//--------------------------------------------------------------------------------


void glcd_setpixel(int column, int row)  // set single pixel in 240x64 array
{

int addr;       // memory address of byte containing pixel to write
  if( (column>=glcd_XMAX) || (row>=glcd_YMAX) )return;
  addr =  glcd_G_BASE + (row*glcd_BYTES_PER_ROW)  + (column/8);
  glcd_set_address(addr);  // set LCD addr. pointer
  glcd_cput(0xf8 | ((7-column%8)) );  // set bit-within-byte command
}

//--------------------------------------------------------------------------------

void glcd_clrpixel(int column, int row)
{
int addr;       // memory address of byte containing pixel to write
  if( (column>=glcd_XMAX) || (row>=glcd_YMAX) )return;
  addr =  glcd_G_BASE + (row*glcd_BYTES_PER_ROW)  + (column/8);
  glcd_set_address(addr);  // set LCD addr. pointer
  glcd_cput(0xf0 | ((7-column%8)) );  // set bit-within-byte command


}

//---------------------------------------------------------------------------------

void glcd_pixel(int column, int row,char show)
{
int addr;       // memory address of byte containing pixel to write
if( (column>=glcd_XMAX) || (row>=glcd_YMAX) )return;
addr =  glcd_G_BASE + (row*glcd_BYTES_PER_ROW)  + (column/8);
glcd_set_address(addr);   // set LCD addr. pointer
if(show)
  glcd_cput(0xf8 | ((7-column%8)) );  // set bit-within-byte command
else
  glcd_cput(0xf0 | ((7-column%8)) );  // set bit-within-byte command
}

//---------------------------------------------------------------------------------


void glcd_show(const char * s,int start_line,int how_many_line){
int addr,i;
addr =  glcd_G_BASE +start_line*30;
glcd_set_address(addr);   // set LCD addr. pointer
glcd_cput(0xB0);
for(i=0;i<how_many_line*30;i++){
	glcd_dput(s[i]); 
	}
glcd_cput(0xB2);
}



void glcd_xy(int x, int y)  // set memory pointer to (x,y) position (text)
{
int addr;

  addr = glcd_T_BASE + (y * glcd_BYTES_PER_ROW) + x;
  glcd_set_address(addr);   // set LCD addr. pointer

} // lcd_xy1()



/* ==============================================================
 * Low-level I/O routines to interface to LCD display
 * based on four routines:
 *
 *          glcd_dput(): write data byte
 *          glcd_cput(): write control byte
 *          glcd_dget(): read data byte         (UNTESTED)
 *          glcd_sget(): read status
 * ==============================================================
 */

void glcd_setup()  // make sure control lines are at correct levels
{
 
 /*DDR(glcd_ce_PORT) |= _BV(glcd_ce_PIN);
 DDR(glcd_rd_PORT) |= _BV(glcd_rd_PIN);
 DDR(glcd_wr_PORT) |= _BV(glcd_wr_PIN);
 DDR(glcd_cd_PORT) |= _BV(glcd_cd_PIN);
 DDR(glcd_rst_PORT) |= _BV(glcd_rst_PIN);*/
    
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;

    glcd_ce_high();
    glcd_rd_high();
    glcd_wr_high();
    glcd_cd_high();


 glcd_SET_DATADIR_OUT();

 glcd_rst_high();

 __delay_ms(5);

 glcd_rst_low();

 __delay_ms(5);

 glcd_rst_high();

} // end glcd_setup()

//---------------------------------------------------------------------------

void glcd_init()  // initialize LCD memory and display modes
{
 glcd_dput(glcd_G_BASE%256);
 glcd_dput(glcd_G_BASE>>8);
 glcd_cput(0x42);       // set graphics memory to address glcd_G_BASE

 glcd_dput(glcd_BYTES_PER_ROW%256);
 glcd_dput(glcd_BYTES_PER_ROW>>8);
 glcd_cput(0x43);  // n bytes per graphics line

 glcd_dput(glcd_T_BASE%256);
 glcd_dput(glcd_T_BASE>>8);
 glcd_cput(0x40);       // text memory at address glcd_T_BASE1

 glcd_dput(glcd_BYTES_PER_ROW%256);
 glcd_dput(glcd_BYTES_PER_ROW>>8);
 glcd_cput(0x41);  // n bytes per text line


 glcd_cput(0x81);  // mode set: Graphics OR Text, ROM CGen

 glcd_cput(0xa7);  // cursor is 8 lines high
 glcd_dput(0x00);
 glcd_dput(0x00);
 glcd_cput(0x21);  // put cursor at (x,y) location

 glcd_cput(0x9C);  
	      // (For cursor to be visible, need to set up position

} // end glcd_init()


// -------------------------------------------------------------------------------

int glcd_sget(void)  // get LCD display status byte
{
int glcd_status;

  glcd_SET_DATADIR_IN();	

  glcd_cd_high();		// bring LCD C/D line high (read status byte)
  glcd_rd_low();		// bring LCD /RD line low (read active)
  glcd_ce_low();		// bring LCD /CE line low (chip-enable active)

 // asm volatile ("nop"::);
 // asm volatile ("nop"::);
  NOP(); NOP();
  __delay_us(10);

  glcd_status = PORTD;      // read LCD status byte

  glcd_ce_high();         // bring LCD /CE line high, disabling it
  glcd_rd_high();         // deactivate LCD read mode
  glcd_SET_DATADIR_OUT(); // make 8-bit parallel port an output port

  return(glcd_status);
} // sget1()

//------------------------------------------------------------------------------------------

void glcd_dput(int byte) // write data byte to LCD module over par. port
		    // assume PC port in data OUTPUT mode
{
  do {} while ((0x03 & glcd_sget()) != 0x03); // wait until display ready
  glcd_cd_low();
  glcd_wr_low();         // activate LCD's write mode

  LATD = byte;          // write value to data port
  glcd_ce_low();                       // pulse enable LOW > 80 ns (hah!)

  //asm volatile ("nop"::);
  //asm volatile ("nop"::);
  NOP();
  NOP();
  __delay_us(10);

  glcd_ce_high();                       // return enable HIGH
  glcd_wr_high();                       // restore Write mode to inactive

 // using my P5/75 MHz PC with ISA bus, CE stays low for 2 microseconds

} // end dput1()

//------------------------------------------------------------------------------------------

int glcd_dget(void)      // get data byte from LCD module
{
int glcd_byte;

  do {} while ((0x03 & glcd_sget()) != 0x03); // wait until display ready
  glcd_SET_DATADIR_IN(); // make PC's port an input port
  glcd_wr_high();   // make sure WRITE mode is inactive
  glcd_cd_low();   // data mode
  glcd_rd_low();   // activate READ mode
  glcd_ce_low();   // enable chip, which outputs data

  glcd_byte = PORTD;  // read data from LCD

  glcd_ce_high();   // disable chip
  glcd_rd_high();   // turn off READ mode
  glcd_SET_DATADIR_OUT(); // make 8-bit parallel port an output port

  return(glcd_byte);
} // glcd_dget1()

//----------------------------------------------------------------------------------------

void glcd_cput(int byte) // write command byte to LCD module
		    // assumes port is in data OUTPUT mode
{
  do {} while ((0x03 & glcd_sget()) != 0x03); // wait until display ready

  LATD = byte;  // present data to LCD on PC's port pins

  __delay_us(10);
  
  glcd_cd_high();         // control/status mode
  glcd_rd_high();         // make sure LCD read mode is off
  glcd_wr_low();         // activate LCD write mode
  glcd_ce_low();         // pulse ChipEnable LOW, > 80 ns, enables LCD I/O
  glcd_ce_high();         // disable LCD I/O
  glcd_wr_high();         // deactivate write mode

} // glcd_cput()










/***********************************************************************
Draws a line from x1,y1 go x2,y2. Line can be drawn in any direction.
Set show1 to 1 to draw pixel, set to 0 to hide pixel.
***********************************************************************/
void glcd_line(int x1, int y1, int x2, int y2, unsigned char show) 
{
  int dy ;
  int dx ;
  int stepx, stepy, fraction;
  dy = y2 - y1;
  dx = x2 - x1;
  if (dy < 0) 
  {
    dy = -dy;
    stepy = -1;
  }
  else 
  {
    stepy = 1;
  }
  if (dx < 0)
  {
    dx = -dx;
    stepx = -1;
  }
  else
  {
    stepx = 1;
  }
  dy <<= 1;
  dx <<= 1;
  glcd_pixel(x1,y1,show);
  if (dx > dy)
  {
    fraction = dy - (dx >> 1); 
    while (x1 != x2)
    {
      if (fraction >= 0)
      {
        y1 += stepy;
        fraction -= dx;
      }
      x1 += stepx;
      fraction += dy;  
      glcd_pixel(x1,y1,show);
    }
  }
  else
  {
    fraction = dx - (dy >> 1);
    while (y1 != y2)
    {
      if (fraction >= 0)
      {
        x1 += stepx;
        fraction -= dy;
      }
      y1 += stepy;
      fraction += dx;
      glcd_pixel(x1,y1,show);
    }
  }
}







/***********************************************************************
Draws a circle with center at x,y with given radius.
Set show1 to 1 to draw pixel, set to 0 to hide pixel.
***********************************************************************/
void glcd_circle(int x, int y, int radius, unsigned char show)
{
  int xc = 0;
  int yc ;
  int p ;
  yc=radius;
  p = 3 - (radius<<1);
  while (xc <= yc)  
  {
    glcd_pixel(x + xc, y + yc, show);
    glcd_pixel(x + xc, y - yc, show);
    glcd_pixel(x - xc, y + yc, show);
    glcd_pixel(x - xc, y - yc, show);
    glcd_pixel(x + yc, y + xc, show);
    glcd_pixel(x + yc, y - xc, show);
    glcd_pixel(x - yc, y + xc, show);
    glcd_pixel(x - yc, y - xc, show);
    if (p < 0)
      p += (xc++ << 2) + 6;
    else
      p += ((xc++ - yc--)<<2) + 10;
  }
}

 
void glcd_circle_half(int x, int y, int radius, unsigned char show)
{
  int xc = 0;
  int yc ;
  int p ;
  yc=radius;
  p = 3 - (radius<<1);
  while (xc <= yc)  
  {
//    glcd_pixel(x + xc, y + yc, show);
    glcd_pixel(x + xc, y - yc, show);
//    glcd_pixel(x - xc, y + yc, show);
    glcd_pixel(x - xc, y - yc, show);
//    glcd_pixel(x + yc, y + xc, show);
    glcd_pixel(x + yc, y - xc, show);
//    glcd_pixel(x - yc, y + xc, show);
    glcd_pixel(x - yc, y - xc, show);
    if (p < 0)
      p += (xc++ << 2) + 6;
    else
      p += ((xc++ - yc--)<<2) + 10;
  }
}








void glcd_box(int x1, int y1, int x2, int y2, unsigned char show)
{
glcd_line(x1,y1,x2,y1,show);  // up
glcd_line(x1,y2,x2,y2,show);  // down
glcd_line(x2,y1,x2,y2,show);  // right
glcd_line(x1,y1,x1,y2,show);  // left
}


/*

//***********************************************************************
//Draws a line from x,y at given degree from inner_radius to outer_radius.
//Set show to 1 to draw pixel, set to 0 to hide pixel.
//***********************************************************************
void glcd_degree_line(int x, int y, int degree, int inner_radius, int outer_radius, unsigned char show)
{
  int fx,fy,tx,ty;

  fx = x + (inner_radius * sin(degree * 3.14 / 180));    //  ???????????????
  fy = y - (inner_radius * cos(degree * 3.14 / 180));
  tx = x + (outer_radius * sin(degree * 3.14 / 180));
  ty = y - (outer_radius * cos(degree * 3.14 / 180));
  glcd_line(fx,fy,tx,ty,show);
}


void glcd_degree_line_bold(int x, int y, int degree, int inner_radius, int outer_radius, unsigned char show)
{
  int fx,fy,tx,ty;

  fx = x + (inner_radius * sin(degree * 3.14 / 180));    //  ???????????????
  fy = y - (inner_radius * cos(degree * 3.14 / 180));
  tx = x + (outer_radius * sin(degree * 3.14 / 180));
  ty = y - (outer_radius * cos(degree * 3.14 / 180));
  glcd_line(fx,fy,tx,ty,show);
  glcd_line(fx+1,fy+1,tx+1,ty+1,show);
  glcd_line(fx-1,fy-1,tx-1,ty-1,show);
}




*/


 
void glcd_fill(int x1, int y1, int x2, int y2, unsigned char persent,char first)
{
char M,horizon_line,horizon_line2,i,str1[10];
if(persent>100)return;
if(!first){
	glcd_line(x1,y2,x2,y2,1);  // down
	glcd_line(x2,y1,x2,y2,1);  // right
	glcd_line(x1,y1,x1,y2,1);  // left
	first=1;
	}
M=100/abs(y2-y1);

horizon_line=persent/M;
for(i=0;i<horizon_line;i++)
	glcd_line(x1+2,y2-2-i,x2-2,y2-2-i,1);

horizon_line2=100/M;
for(i=horizon_line;i<horizon_line2;i++)
	glcd_line(x1+2,y2-2-i,x2-2,y2-2-i,0);


sprintf(str1,"%02d%% ",persent);
glcd_print_ram( (x2+x1)/16-1 , (y2+y1)/16,str1);
}
