/***************************************************************

       mouse.c

****************************************************************

       FUNCTION : All functions relating to mouse

       PASSED   : Various

       CALLS    : INTERRUPT 33h

       RETURNS  : various

****************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <graphics.h>
#include <conio.h>
#include <dos.h>
#include "mouse.h"

boolean mouseexists;

int arrow_masks[]=
 {
    /* screen mask */

    0x9fff , /* 1001111111111111 */
    0x8fff , /* 1000111111111111 */
    0x87ff , /* 1000011111111111 */
    0x83ff , /* 1000001111111111 */
    0x81ff , /* 1000000111111111 */
    0x80ff , /* 1000000011111111 */
    0x807f , /* 1000000001111111 */
    0x803f , /* 1000000000111111 */
    0x801f , /* 1000000000011111 */
    0x803f , /* 1000000000111111 */
    0x80ff , /* 1000000011111111 */
    0x887f , /* 1000100001111111 */
    0x987f , /* 1001100001111111 */
    0xfc3f , /* 1111110000111111 */
    0xfc3f , /* 1111110000111111 */
    0xfe3f , /* 1111111000111111 */

    /* cursor mask */

    0x0   ,  /* 0000000000000000 */
    0x2000,  /* 0010000000000000 */
    0x3000,  /* 0011000000000000 */
    0x3800,  /* 0011100000000000 */
    0x3c00,  /* 0011110000000000 */
    0x3e00,  /* 0011111000000000 */
    0x3f00,  /* 0011111100000000 */
    0x3f80,  /* 0011111110000000 */
    0x3fc0,  /* 0011111111000000 */
    0x3e00,  /* 0011111000000000 */
    0x3600,  /* 0011011000000000 */
    0x2300,  /* 0010001100000000 */
    0x300 ,  /* 0000001100000000 */
    0x180 ,  /* 0000000110000000 */
    0x180 ,  /* 0000000110000000 */
    0x0      /* 0000000000000000 */
 };

int mouse_masks[]=
 {
    /* screen mask */

    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfd7f,  /* 1111110101111111 */
    0x381 ,  /* 0000001110000001 */            /* hot spot in middle */
    0xfd7f,  /* 1111110101111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xfeff,  /* 1111111011111111 */
    0xffff,  /* 1111111111111111 */

    /* cursor mask */

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
 };


/***********************************************************************/

void mouse(int *m1,int *m2,int *m3,int *m4)
   {
     struct REGPACK regs;
     regs.r_ax = (*m1);    regs.r_bx = (*m2);
     regs.r_cx = (*m3);    regs.r_dx = (*m4);
     intr(0x33,&regs);


     (*m1) = regs.r_ax;    (*m2) = regs.r_bx;
     (*m3) = regs.r_cx;    (*m4) = regs.r_dx;
   }

void hidemouse(void)
{
  int m1 = mousehide,m2,m3,m4;

  mouse(&m1,&m2,&m3,&m4);
}


void showmouse(void)
{
  int m1 = mouseshow,m2,m3,m4;

  mouse(&m1,&m2,&m3,&m4);
}

boolean testbutton(int testtype,int whichbutton)
{
  int m1,m2,m3,m4 ;
  boolean returnvalue = FALSE;

  if((whichbutton == LEFTBUTTON)||(whichbutton == EITHERBUTTON))
    {
      m1 = testtype; m2 = LEFTBUTTON;
      mouse(&m1,&m2,&m3,&m4);
      if ( m2 > 0)
	 returnvalue = TRUE;
    }
    if ((!returnvalue) && ((whichbutton == RIGHTBUTTON)
			  ||(whichbutton == EITHERBUTTON)))
        {
          m1 = testtype;
          m2 = RIGHTBUTTON;
	  mouse(&m1,&m2,&m3,&m4);
	  if ( m2 > 0)
	    returnvalue= TRUE;
	 }

	return(returnvalue);
}



boolean resetmouse(void)
{
  int m1 = mousereset,m2,m3,m4;

  mouse(&m1,&m2,&m3,&m4);
  return (m1 < 0)  ? TRUE : FALSE;
}


void movemouse(int x,int y)
  {
    int m1 = setmousecoord,m2,m3 = x,m4 = y;

    if ( getmaxx() == 319) (m3 <<= 1); /* m3 x 2 */
    mouse(&m1,&m2,&m3,&m4);
  }


boolean initmouse(void)
  {

  mouseexists = TRUE;
  if(resetmouse())
    {
      if (getgraphmode() == HERCMONOHI)
	  poke(0x0040,0x0049,06);
      movemouse(0,0);
      showmouse();
      return TRUE;
    }
  return FALSE;

  }

void getmousecoords(int *x,int *y)
  {
    int m1 = getmousestatus,m2,m3 = *x,m4 = *y;

     mouse(&m1,&m2,&m3,&m4);
     *x = m3;
     *y = m4;
     if ( getmaxx() == 319) (*x >>= 1);
  }


void getmousemovement(int *x,int *y)
{
     int m1,m2,m3,m4;

     m1 = mousemovement;
     m3 = *x;
     m4 = *y;
     mouse(&m1,&m2,&m3,&m4);
     *x = m3;
     *y = m4;
     if ( getmaxx() == 319)
       *x /= 2;
}


boolean mousebuttonreleased(int whichbutton)
  {
    return(testbutton(checkbuttonrelease,whichbutton));
  }


boolean mousebuttonpressed(int whichbutton)
  {
    return(testbutton(checkbuttonpress,whichbutton));
  }

boolean mouseinbox(int left,int top,int right,int bottom,int x,int y)

     {
       return(
	       (x >= left)    &&
	       ( x <= right ) &&
	       (y >= top)     &&
	       (y  <= bottom)
	      );
     }

int getinput(int whichbutton)
  {
    if(mousebuttonpressed(whichbutton))
      {
	while (!mousebuttonreleased(whichbutton)) ;
	return -1;
      }
   else if(mousebuttonreleased(whichbutton))
	   return -1;
   return 0;
  }

void crosshaircursor(void)
{
  union REGS inregs, outregs;
  struct SREGS sregs;

  segread(&sregs);
  sregs.es = sregs.ds;
  inregs.x.ax = 9;    /* mouse function no. */
  inregs.x.bx = 7;    /* horizontal hotspot */
  inregs.x.cx = 7;    /* vertical hotspot   */
  inregs.x.dx = (int)mouse_masks;
  int86x(0x33,&inregs,&outregs,&sregs);
}

void arrowcursor(void)
  {
    union REGS inregs, outregs;
    struct SREGS sregs;

    segread(&sregs);
    sregs.es = sregs.ds;
    inregs.x.ax = 9;    /* mouse function no. */
    inregs.x.bx = 0;    /* horizontal hotspot */
    inregs.x.cx = -1;    /* vertical hotspot   */
    inregs.x.dx = (int)arrow_masks;
    int86x(0x33,&inregs,&outregs,&sregs);

}

void setmouserange(int hmin,int hmax,int vmin,int vmax)
  {
    int m1,m2,m3,m4;

    m1 = setmousehorizrange;
    m3 = hmin;
    m4 = hmax;
    mouse(&m1,&m2,&m3,&m4);

    m1 = setmousevertrange;
    m3 = vmin;
    m4 = vmax;
    mouse(&m1,&m2,&m3,&m4);
  }

void mousesoftwarereset(void)
  {
    int m1 = softwarereset,m2,m3,m4;
    mouse(&m1,&m2,&m3,&m4);
  }
/*---------------------------- END OF MODULE --------------------------*/

