/*****************************************************************************
	This module contains all functions for the custom graphics.
*****************************************************************************/
#include <graphics.h>
#include <alloc.h>
#include <stdio.h>
#include <process.h>
#include <conio.h>
#include "objects.h"
#include "geo.h"
#include "graph3d.h"

/*============================================================================
								SAVE_SCREEN
==============================================================================
	Saves a screen in a memory buffer.
============================================================================*/
void save_screen(void far *buf[4])
{
   unsigned size;
   int ystart=0, yend, yincr, block;

   yincr = (MAXY+1) / 4;
   yend = yincr;
   size = imagesize(0, ystart, MAXX, yend);
/* get byte size of image */

   for (block=0; block<=3; block++)
   {
	  if ((buf[block] = farmalloc(size)) == NULL)
	  {
		 closegraph();
		 printf("Error: not enough heap space in save_screen().\n");
		 exit(1);
	  }

	  getimage(0, ystart, MAXX, yend, buf[block]);
	  ystart = yend + 1;
	  yend += yincr + 1;
   }
}

/*============================================================================
								RESTORE_SCREEN
==============================================================================
		Restores a screen saved in memory.
============================================================================*/
void restore_screen(void far *buf[4])
{
   int ystart=0, yend, yincr, block;

   yincr = (MAXY+1) / 4;
   yend = yincr;

   for (block=0; block<=3; block++)
   {
	  putimage(0, ystart, buf[block], COPY_PUT);
	  farfree(buf[block]);
	  ystart = yend + 1;
	  yend += yincr + 1;
   }
}

/*============================================================================
								  CLIP
==============================================================================
	This function clips a line against a front clipping plane.
============================================================================*/
int clip(float *x,float *y,float *z,float x2,float y2,float z2)
{
 float z_change;

 /*CHECK IF LINE INTERSECTS CLIPPING PLANE*/
 if ((*z<=FRONT_Z && z2>FRONT_Z)||(*z>=FRONT_Z && z2<FRONT_Z))
   {
	 z_change=(FRONT_Z-*z)/(*z-z2);
	 *x=(*x-x2)*(z_change)+*x;
	 *y=(*y-y2)*(z_change)+*y;

	 if (z2>=FRONT_Z) *z=FRONT_Z;
	 return CLIPPED;
   }
 else
   if ((*z<=FRONT_Z && z2<=FRONT_Z))   /*LINE IS TOTALLY INVISIBLE*/
	*z=-1000;
 return 0;

}

/*============================================================================
							   NEW_SCANLINE
==============================================================================
	Allocates memory to store a scanline structure.
============================================================================*/
scanline new_scanline()
{
  scanline temp;

  temp=(scanline)malloc(sizeof(SCANLINE));
  if(temp==NULL)
	 temp=NULL;
  temp->next_scan=NULL;
  temp->x_value=NULL;


  return temp;
}

/*============================================================================
							   NEW_INTERSECT
==============================================================================
	Allocates memory to store an intersection structure.
============================================================================*/
intersect new_intersect()
{
  intersect temp;

  temp=(intersect)malloc(sizeof(INTERSECT));

  temp->next_intersect=NULL;

  return temp;
}

/*============================================================================
							  CALCULATE_OUTCODE
==============================================================================
	   THIS FUNCTION CALCULATES THE OUTCODES FOR CLIPPING AGAINST THE FOUR
CLIPPING PLANES.
============================================================================*/
int *calculate_outcode(float x,float y,float z)
{
 int *outcode;
 int count=0;

 outcode=(int *)calloc(2,sizeof(int));
 outcode[0]=outcode[1]=0;

 if(x<-z)
	{
	 outcode[count]=LEFT_CLIP;     /*LINE CROSSES LEFT CLIPPING PLANE*/
	 ++count;
	}
 else if(x>z)
	{
	 outcode[count]=RIGHT_CLIP;  /*LINE CROSSES RIGHT CLIPPING PLANE*/
	 ++count;
	}
 if(y<-z) outcode[count]=BOTTOM_CLIP;   /*LINE CROSSES BOTTOM CLIPPING PLANE*/
 else if(y>z) outcode[count]=TOP_CLIP;  /*LINE CROSSES TOP CLIPPING PLANE*/
 return outcode;
}

/*============================================================================
							  CLIP_PLANES
==============================================================================
	THIS FUNCTION CLIPS A LINE AGAINST THE TOP,BOTTOM AND LEFT,RIGHT
CLIPPING PLANES.
============================================================================*/
int clip_planes(float *x,float *y,float *z,float *x2,float *y2,float *z2,int color,int fill)
{
  int outcode[2]={0,0};
  int *outcode1;
  int *outcode2;
  int count;
  float temp_x,temp_y,temp_z,t,div_value;

  outcode1=calculate_outcode(*x,*y,*z);  /*CALCULATE OUTCODES FOR END-POINTS*/
  outcode2=calculate_outcode(*x2,*y2,*z2);

  while(outcode1[0]!=0 || outcode1[1]!=0 || outcode2[0]!=0 || outcode2[1]!=0)
  {
   if((outcode1[0]==outcode2[0] && outcode2[0]!=0)||((outcode2[1]==outcode1[1] && outcode2[1]!=0))
	  ||(outcode1[1]==outcode2[0] && outcode2[0]!=0)||((outcode2[1]==outcode1[0] && outcode2[1]!=0)))
     {
	 if(fill==SOLID)
	{
	  convert_3d_to_2d(count,x,y,-*z,x2,y2,-*z2);
	}
	 else
	{
	  *z=-9999;
	  *z2=-9999;
	}
	  free(outcode1);
	  free(outcode2);
	  return CLIPPED;       /*LINE NOT VISIBLE*/
     }
	  for(count=0;count<2;count++)
	 {
	  if(outcode1[0]!=0 || outcode1[1]!=0)
		 outcode[count]=outcode1[count];
	  else
		 outcode[count]=outcode2[count];
	}
	   if(outcode[0]==LEFT_CLIP)          /*CLIP AGAINST LEFT-CLIP PLANE*/
	  {
	   div_value=round_error(((*x-*x2)-(*z2-*z)));
	   if(div_value==0)
		  t=0;
	   else
	   t=(*z+*x)/div_value;
	   temp_z=t*(*z2-*z)+*z;
	   temp_x=-temp_z;
	   temp_y=t*(*y2-*y)+*y;
	  }

	   if(outcode[0]==RIGHT_CLIP)         /*CLIP AGAINST RIGHT-CLIP PLANE*/
	  {
	   div_value=round_error(((*x2-*x)-(*z2-*z)));
	   if(div_value==0)
		  t=0;
	   else
	   t=(*z-*x)/div_value;
	   temp_z=t*(*z2-*z)+*z;
	   temp_x=temp_z;
	   temp_y=t*(*y2-*y)+*y;
	  }
					 /*CLIP AGAINST BOTTOM-CLIP PLANE*/
	   if((outcode[0]==BOTTOM_CLIP)||(outcode[1]==BOTTOM_CLIP))
	  {
		div_value=round_error(((*y-*y2)-(*z2-*z)));
		if(div_value==0)
		  t=0;
		else
		t=(*z+*y)/div_value;
		temp_z=t*(*z2-*z)+*z;
		temp_x=t*(*x2-*x)+*x;
		temp_y=-temp_z;
	  }
					/*CLIP AGAINST TOP-CLIP PLANE*/
	   if((outcode[0]==TOP_CLIP)||(outcode[1]==TOP_CLIP))
	  {
		div_value=round_error(((*y-*y2)-(*z2-*z)));
		if(div_value==0)
		  t=0;
		else
		t=(*z-*y)/div_value;
		temp_z=t*(*z2-*z)+*z;
		temp_x=t*(*x2-*x)+*x;
		temp_y=temp_z;
	  }

	   if((outcode[0]==outcode1[0])&&(outcode[1]==outcode1[1]))
	 {
	   temp_x=round_error(temp_x);
	   temp_y=round_error(temp_y);
	   temp_z=round_error(temp_z);
	   *x=round_error(temp_x);
	   *y=round_error(temp_y);
	   *z=round_error(temp_z);
	   free(outcode1);
	   /*CALCULATE NEW OUTCODES AND START AGAIN*/
	   outcode1=calculate_outcode(temp_x,temp_y,temp_z);
	 }
	   else
	 {
	   temp_x=round_error(temp_x);
	   temp_y=round_error(temp_y);
	   temp_z=round_error(temp_z);
	   *x2=round_error(temp_x);
	   *y2=round_error(temp_y);
	   *z2=round_error(temp_z);
	   free(outcode2);
	   /*CALCULATE NEW OUTCODES AND START AGAIN*/
	   outcode2=calculate_outcode(temp_x,temp_y,temp_z);
	 }
	   }
	count=0;
	free(outcode1);
	free(outcode2);
	if(fill!=EXTENT_FILL)
	  {
	   setcolor(color);
	   convert_3d_to_2d(count,x,y,-*z,x2,y2,-*z2);
	  }
}



/*============================================================================
								  MYPOLY
==============================================================================
============================================================================*/
void mypoly(float vert[],int no_of_verts,int color,int shape_num)
{
 float minx=99999,maxx=-99999,miny=99999,maxy=-99999;
 int i,no_of_edges,edge,edge_vert;
 float slope[200],current_x;
 float yval[100][2];
 float xval[100];
 scanline scanlist=new_scanline();
 scanline head;

 head=scanlist;


 /*FIND MIN AND MAX Y VALUES OF POLYGON*/
 for(i=0;i<no_of_verts*2;i+=2)
	{
	  minx=min(minx,vert[i]);
	  maxx=max(maxx,vert[i]);
	  miny=min(miny,vert[i+1]);
	  maxy=max(maxy,vert[i+1]);
	}

 /*IF THERE IS LESS THAN ONE SCANLINE DON'T BOTHER DRAWING IT*/
 if(maxy-miny<1)
   {
	 free(scanlist);
	 return;
   }

 /*SETUP ALL THE SCANLINES FOR THE POLYGON*/
 for(i=miny;i<maxy;i++)
	{
	 scanlist->scanline_no=i;
	 scanlist->next_scan=new_scanline();
	 scanlist=scanlist->next_scan;
	}
 scanlist->scanline_no=i;
 scanlist=head;

 no_of_edges=no_of_verts/2;

 /*CALCULATE THE SLOPES AND STARTING POINTS FOR SCANLINES OF THE EDGES*/
 for(edge=0;edge<no_of_edges;edge++)
	{
	 edge_vert=edge<<2;

	 if(vert[edge_vert+3]-vert[edge_vert+1]==0)
		slope[edge]=-99;        /*ELIMINATE HORIZONTAL EDGES*/
	 else
	 if(vert[edge_vert+2]-vert[edge_vert]==0)
		slope[edge]=0;
	 else                       /*CALCULATE THE SLOPE OF AN EDGE*/
		slope[edge]=1.0/((float)(vert[edge_vert+3]-vert[edge_vert+1])/(float)(vert[edge_vert+2]-vert[edge_vert]));

	 /*START AT TOPMOST POINT OF EDGE*/
	 yval[edge][0]=min(vert[edge_vert+3],vert[edge_vert+1]);
	 yval[edge][1]=max(vert[edge_vert+3],vert[edge_vert+1]);


	 if(vert[edge_vert+3]==yval[edge][0])
	 xval[edge]=vert[edge_vert+2];
	 else
	 xval[edge]=vert[edge_vert];

	}

 /*CALCULATE INTERSECTION POINTS BETWEEN THE SCANLINES AND THE EDGES*/
 for(i=0;i<no_of_edges;i++)
   {
	if(slope[i]!=-99)/*IF THE EDGE IS NOT HORIZONTAL*/
	  {
	   /*FIND FIRST SCANLINE TO INTERSECT THIS EDGE*/
	   while(scanlist->scanline_no<(int)yval[i][0])
		 scanlist=scanlist->next_scan;

	   current_x=xval[i];

	   /*FIND ALL OTHER SCANLINES INTERSECTING THE EDGE*/
	   while(scanlist->scanline_no<(int)yval[i][1])
		   {
			add_xval(scanlist,current_x);
			scanlist=scanlist->next_scan;
			current_x=current_x+slope[i];
		   }
	   scanlist=head;
	  }
   }

 draw_scanlines(scanlist,color);

 delete_scanlines(scanlist);
}

/*============================================================================
							  DRAW_SCANLINES
==============================================================================
	This function draws all the scanlines produced by the MYPOLY() function.
============================================================================*/
void draw_scanlines(scanline scanlist,int color)
{
  int x,x2,y;
  scanline lastscan,scanhead;
  intersect first,second,intersect_head;
  setcolor(color);

  scanhead=scanlist;

  /*FOR EVERY SCANLINE*/
  while(scanlist!=NULL)
	 {
	  lastscan=scanlist;
	  intersect_head=scanlist->x_value;

	  /*DRAW LINES BETWEEN THE INTERSECTION POINTS OF THIS SCANLINE AND THE
		POLYGON EDGES*/
	  while(scanlist->x_value!=NULL)
		{
		 first=scanlist->x_value;
		 second=scanlist->x_value->next_intersect;
		 x=scanlist->x_value->x_coord;

		 /*IF THERE IS AN UNEVEN NUMBER OF INTERSECTION POINTS*/
		 if(second==NULL)
			x2=x;
		 else
			x2=scanlist->x_value->next_intersect->x_coord;

		 y=scanlist->scanline_no;
		 line(x,y,x2,y);         /*DRAW SCANLINE*/

		 if(second==NULL)
			scanlist->x_value=scanlist->x_value->next_intersect;
		 else
		   {
			scanlist->x_value=scanlist->x_value->next_intersect;
		   }
	  }
	  scanlist->x_value=intersect_head;
	  scanlist=scanlist->next_scan;
	 }
  scanlist=scanhead;
}

/*============================================================================
								ADD_XVAL
==============================================================================
	This function adds an intersection point to the list of intersections
associated with a scanline.
============================================================================*/
void add_xval(scanline scanlist, float current_x)
{
  intersect temp,head,last;

  temp=new_intersect();

  temp->x_coord=current_x;

  /*CHECK IF THIS IS THE FIRST INTERSECTION*/
  if(scanlist->x_value==NULL)
	 scanlist->x_value=temp;
  else
  /*IF NOT*/
   {
	head=scanlist->x_value;

	/*FIND A PLACE IN THE LIST FOR THE NEW POINT  NOTE:- ALL POINTS ARE
	  KEPT IN INCREASING ORDER*/
	while(scanlist->x_value->next_intersect->x_coord<=current_x
		  && scanlist->x_value->next_intersect!=NULL)
		  scanlist->x_value=scanlist->x_value->next_intersect;

	/*CHECK IF POINT IS BEING ADDED TO THE HEAD OF THE LIST*/
	if(scanlist->x_value!=head)
	 {
	   temp->next_intersect=scanlist->x_value->next_intersect;
	   scanlist->x_value->next_intersect=temp;
	   scanlist->x_value=head;
	 }
	else
	 {
	   temp->next_intersect=scanlist->x_value;
	   scanlist->x_value=temp;
	 }
   }
}

/*============================================================================
							DELETE_SCANLINES
==============================================================================
	This function frees all the memory taken up by the scanlines.
============================================================================*/
void delete_scanlines(scanline scanlist)
{
 scanline nextscan;
 intersect nextintersect;

 nextscan=scanlist;

 while(nextscan!=NULL)
   {
	 scanlist=nextscan;
	 nextintersect=scanlist->x_value;

	 while(nextintersect!=NULL)
	   {
		 scanlist->x_value=nextintersect;
		 nextintersect=scanlist->x_value->next_intersect;
		 free(scanlist->x_value);
	   }

	 nextscan=scanlist->next_scan;
	 free(scanlist);
   }
}

/*============================================================================
								 POLYCLIP
==============================================================================
	This function clips a polygon against a given boundary.
============================================================================*/
void polyclip(float in_verts[],int *no_of_sides,int boundary[4],int boundary_type)
{
 float outverts[200];
 float clipped[4];
 int outcount=0;
 int i,vert,clip=0,count;


 float x1,y1,x2,y2;
 float clip_x,clip_y;


 for(i=0;i<*no_of_sides/4;i++)
 {
   vert=i<<2;
   x1=in_verts[vert];
   y1=in_verts[vert+1];
   x2=in_verts[vert+2];
   y2=in_verts[vert+3];

   /*IF THE FIRST END IS WITHIN THE BOUNDARY*/
   if(inside(x1,y1,boundary_type,boundary))
   /*IF THE SECOND POINT IS WITHIN THE BOUNDARY*/
	   if(inside(x2,y2,boundary_type,boundary))
		  {
		   /*THESE POINTS REQUIRE NO CLIPPING*/
		   outverts[outcount]=x1;
		   outverts[outcount+1]=y1;
		   outverts[outcount+2]=x2;
		   outverts[outcount+3]=y2;
		   outcount+=4;
		  }
		  else
		  /*OTHERWISE THE SECOND POINT NEEDS CLIPPING*/
		  {
		   /*CALCULATE INTERSECTION OF LINE WITH BOUNDARY*/
		   line_intersect(x1,y1,x2,y2,&clip_x,&clip_y,boundary,boundary_type);
		   outverts[outcount]=x1;
		   outverts[outcount+1]=y1;
		   outverts[outcount+2]=clip_x;
		   outverts[outcount+3]=clip_y;
		   outcount+=4;
		   clipped[clip]=clip_x;
		   clipped[clip+1]=clip_y;
		   clip+=2;
		  }

   else
	  if(inside(x2,y2,boundary_type,boundary))
	  /*SECOND END IS WITHIN BOUNDERY BUT FIRST ONE IS NOT*/
		{
		 line_intersect(x1,y1,x2,y2,&clip_x,&clip_y,boundary,boundary_type);
		 outverts[outcount]=clip_x;
		 outverts[outcount+1]=clip_y;
		 outverts[outcount+2]=x2;
		 outverts[outcount+3]=y2;
		 outcount+=4;
		 clipped[clip]=clip_x;
		 clipped[clip+1]=clip_y;
		 clip+=2;
		}
   /*IF BOTH ENDS HAVE BEEN CLIPPED*/
   if(clip>=4)
	 {
	  outverts[outcount]=clipped[0];
	  outverts[outcount+1]=clipped[1];
	  outverts[outcount+2]=clipped[2];
	  outverts[outcount+3]=clipped[3];
	  outcount+=4;
	  clip=0;
	 }

  }

  for(count=0;count<outcount;count++)
	 in_verts[count]=outverts[count];
  *no_of_sides=outcount;
}

/*============================================================================
							   INSIDE
==============================================================================
	 Test to see if a point is within a given boundary.
============================================================================*/
int inside(float x,float y,int boundary_type,int boundary[4])
{
 switch(boundary_type)
	 {
	  case TOP_CLIP: return(y>=boundary[1]);
	  case LEFT_CLIP : return(x>=boundary[0]);
	  case RIGHT_CLIP : return(x<=boundary[0]);
	  case BOTTOM_CLIP : return(y<=boundary[1]);
	 }
}

/*============================================================================
							  LINE_INTERSECT
==============================================================================
	Calculate the intersection between a given boundary and a line.
============================================================================*/
void line_intersect(float x1,float y1,float x2,float y2,float *clip_x,float *clip_y,int boundary[4],int boundary_type)
{
   if(boundary_type==TOP_CLIP || boundary_type==BOTTOM_CLIP)
	 {
	  *clip_x=x1+(x2-x1)*(boundary[1]-y1)/(y2-y1);
	  *clip_y=boundary[1];
	 }
   if (boundary_type==LEFT_CLIP || boundary_type==RIGHT_CLIP)
	 {
	  *clip_y=y1+(y2-y1)*(boundary[0]-x1)/(x2-x1);
	  *clip_x=boundary[0];
	 }
}


/*============================================================================
							  MAPMASK
==============================================================================
	Load the map mask register withe the byte define by plane.
============================================================================*/
void mapmask(char plane)
{
  outp(0x3c4,2);
  outp(0x3c5,plane);
}


/*============================================================================
								 SETRES
==============================================================================
	Load the set reset register with the byte 'value'.
============================================================================*/
void setres(char value)
{
 outp(0x3ce,0);
 outp(0x3cf,value);
}


/*============================================================================
								  ESETRES
==============================================================================
	Load the enable set register with the value 'mask'
============================================================================*/
void esetres(char mask)
{
 outp(0x3ce,1);
 outp(0x3cf,mask);
}


/*============================================================================
								 H_LINE
==============================================================================
	Special EGA/VGA routine for drawing horizontal lines.
============================================================================*/
void hline(int page,int x1,int x2,int y,int color)
{
 int i,x,x1byte,x2byte,nbyte,address,dx,offset=80;
 char mask1,mask2;

 esetres(0x0f);
 setres(color);

 if(x1>x2)   /*SWAP VALUES*/
	{
	  x=x1;
	  x1=x2;
	  x2=x;
	}

 x1byte=x1>>3;
 x2byte=x2>>3;
 dx=x2byte-x1byte;
 nbyte=(x2byte-x1byte)-1;
 address=(y*offset)+x1byte;

 /*CALCULATE THE BIT-MASK FOR THE BEGINNING OF THE LINE*/
 switch(x1%8)
	   {
	 case 0: mask1=0xff;
		 break;
	 case 1: mask1=0x7f;
		 break;
	 case 2: mask1=0x3f;
		 break;
	 case 3: mask1=0x1f;
		 break;
	 case 4: mask1=0x0f;
		 break;
	 case 5: mask1=0x07;
		 break;
	 case 6: mask1=0x03;
		 break;
	 case 7: mask1=0x01;
		 break;
	   }
 /*CALCULATE THE BIT-MASK FOR THE END OF THE LINE*/
 switch(x2%8)
	   {
	 case 0: mask1=0x80;
		 break;
	 case 1: mask1=0xc0;
		 break;
	 case 2: mask1=0xe0;
		 break;
	 case 3: mask1=0xf0;
		 break;
	 case 4: mask1=0xf8;
		 break;
	 case 5: mask1=0xfc;
		 break;
	 case 6: mask1=0xfe;
		 break;
	 case 7: mask1=0xff;
		 break;
	   }

  /*IF THE LINE IS LESS THAN 8 PIXELS LONG*/
  if(dx == 0)
	 hsub1(address,mask1&mask2);
  /*IF THE LINE IS LESS THAN 16 PIXELS LONG*/
  if(dx==1)
	 hsub2(address,mask1,mask2);
  /*IF THE LINE IS GREATER THAN 16 PIXELS LONG*/
  if(dx>1)
	 hsub3(address,mask1,mask2,nbyte);

  bitmask(255);
  esetres(0);
  mapmask(15);
}

/*============================================================================
							  BITMASK
==============================================================================
	Sets the bitmask register to the byte 'val'.
============================================================================*/
void bitmask(char val)
{
 outp(0x3ce,8);
 outp(0x3cf,val);
 }



