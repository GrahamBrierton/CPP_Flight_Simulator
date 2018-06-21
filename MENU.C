#include <graphics.h>
#include <string.h>

extern void menu(int x,int y,char **optlist,int no_of_opts)
{
  int y_max,x_max=0,i;

  y_max=y+(no_of_opts*10);

  for(i=0;i<no_of_opts;i++)
	 {
	  if(strlen(optlist[i])>x_max)
		 x_max=strlen(optlist[i]);
	 }
  setfillstyle(SOLID_FILL,DARKGRAY);
  setcolor(LIGHTGREEN);

  bar(x,y,x_max+20,y_max+20);
  setcolor(WHITE);

  line(x+1,y+1,x_max+19,y+1);
  line(x+1,y+1,x+1,y_max+19);

  setcolor(LIGHTGRAY);

  line(x+1,y_max+19,x_max+19,y_max+19);
  line(x_max+19,y+1,x_max+19,y_max+19);

  setcolor(WHITE);

  for(i=0;i<no_of_opts;i++)
	  outtextxy(x+10,y+10+(i*10),optlist[i]);
}






