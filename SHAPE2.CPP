#include <stdio.h>
#include <time.h>
#include <graphics.h>
#include <conio.h>
#include <bios.h>
#include <alloc.h>
#include <stdlib.h>
#include <math.h>
#include <io.h>
#include <ctype.h>
#include <dos.h>
#include <string.h>
#include "objects2.h"
#include "graph_3d.c"

main()
{
  int driver=VGA;
  static int mode,i;

  shape *objects;
  int page=1,count,stime,read_key,turn_angle=0,speed=0;
  long ltime;

  /*VIEW POINT LOCATION IN WORLD CO-ORDINATES*/
  float tempx=10;
  float tempy=10;
  float tempz=-1000;

  int no_of_pyramids=4;
  int no_of_cubes=4;
  int no_of_diamonds=4;
  char *temp_str;
  matrix transform_mat=mat3d();

  /*INITIALIZE GRAPHICS*/
  mode=VGAMED;
  initgraph(&driver,&mode,"g:\\c\\bc3.1\\bgi");

  /*SEED RANDOM NUMBER GENERATOR*/
  ltime=time(NULL);
  stime=(unsigned int) ltime/2;
  srand(stime);

  /*CHOOSE RANDOM NUMBERS OF SHAPES*/
  no_of_pyramids=random_select(15);
  no_of_cubes=random_select(15);
  no_of_diamonds=random_select(15);

  /*ALLOCATE MEMORY TO HOLD SHAPE DATA*/
  objects=allocate_shape_mem(no_of_pyramids+no_of_cubes+no_of_diamonds+1);

  /*ALLOCATE DATA TO SHAPE STRUCTURES*/
  allocate_shapes(objects,no_of_pyramids,no_of_cubes,no_of_diamonds);

  /*CREATE BOUNDARY CUBE*/
  bounding_box(objects[no_of_pyramids+no_of_cubes+no_of_diamonds]);


  /*MAIN LOOP*/
  while(read_key!=RETURN)
  {
    if(bioskey(1)!=0)               /*CHECK IF A KEY HAS BEEN HIT*/
      read_key=bioskey(0);          /*READ KEY VALUE*/
      else read_key=0;
   if(read_key!=0)
     {
    switch(read_key)
      {
	case(UP) :speed++;         /*INCREASE SPEED*/
		  break;
	case(DOWN) :speed--;       /*DECREASE SPEED*/
		    break;
	case(LEFT): turn_angle++;
		    bank(turn_angle); /*BANK VIEW RIGHT (IE VIEW-UP VECTOR)*/
		    break;
	case(RIGHT): turn_angle--;
		    bank(turn_angle);
		    break;
      }
     }
       /*UPDATE CURRENT VIEWPOINT*/
	tempx=tempx+speed*dxn;
	tempz=tempz+speed*-dzn;
	tempy=tempy+speed*dyn;
       /*========================*/

	setactivepage(page&1);         /*SETUP FOR PAGE FLIPPING*/

	setviewport(0,0,639,440,1);    /*VIEWPORT UPDATES ARE FASTER THAN
					 NORMAL SCREEN*/

	clearviewport();               /*CLEAR SCREEN/VIEWPORT*/

	set_view_reference_point(tempx,tempy,tempz);
	set_perspective(tempx-view_distance*dxn,tempy-view_distance*dyn,tempz-view_distance*dzn);

	transform_mat=trans_eye();  /*DERIVE A TRANSFORMATION MATRIX TO MAP
				      FROM WORLD TO EYE CO-ORDINATE SYSTEMS*/

	/*LOOP TO DRAW ALL OBJECTS*/
	for(count=0;count<no_of_pyramids+no_of_cubes+no_of_diamonds+1;count++)
	   create_view(objects[count],transform_mat);

	/*PRINT CURRENT SPEED ON SCREEN*/
	setcolor(YELLOW);
	sprintf(temp_str,"SPEED = %d",speed);
	outtextxy(300,10,temp_str);

	/*FREE MEMORY ALLOCATED FOR TRANSFORMATION MATRIX*/
	free(transform_mat);

	/*DISPLAY WHAT HAS BEEN DRAWN*/
	setvisualpage(page&1);
	page++;

  }



}













