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
#include "graph_3d.h"


/*============================================================================
==============================================================================
			MEMORY ALLOCATION ROUTINES
==============================================================================
============================================================================*/


/*============================================================================
				SHAPE_3D
==============================================================================
     FUNCTION TO ALLOCATE MEMORY TO A SHAPE STRUCTURE.
============================================================================*/
shape shape_3d()
{
 return (shape)malloc(sizeof(SHAPE));
}

/*============================================================================
				 MAT3D
==============================================================================
	THIS FUNCTION ALLOCATES MEMORY FOR A 4*3 MATRIX
============================================================================*/
matrix mat3d()
{
 return (matrix)malloc(sizeof(MATRIX));
}



/*============================================================================
==============================================================================
			 MATRIX MANIPULATION ROUTINES
==============================================================================
============================================================================*/


/*============================================================================
			      MROTX3D
==============================================================================
	THIS FUNCTION TAKES A MATRIX AS INPUT AND COMPOUNDS IT WITH A ROTATION
MATRIX ABOUT THE X-AXIS. THE DEGREE OF ROTATION IS ALSO DEFINED BY THE INPUTS
============================================================================*/
void mrotx3d(matrix transform,float sin_angle,float cos_angle)
{
 int i;
 float temp;

 for(i=0;i<4;i++)
    {
     temp=transform->entry[i][1]*cos_angle-transform->entry[i][2]*sin_angle;
     transform->entry[i][2]=transform->entry[i][1]*sin_angle+transform->entry[i][2]*cos_angle;
     transform->entry[i][1]=temp;
    }
}

/*============================================================================
				  MROTY3D
==============================================================================
	THIS FUNCTION TAKES A MATRIX AS INPUT AND COMPOUNDS IT WITH A ROTATION
MATRIX ABOUT THE Y-AXIS. THE DEGREE OF ROTATION IS ALSO DEFINED BY THE INPUTS
============================================================================*/
void mroty3d(matrix transform,float sin_angle,float cos_angle)
{
 int i;
 float temp;

  for(i=0;i<4;i++)
    {
      temp=transform->entry[i][0]*cos_angle-transform->entry[i][2]*sin_angle;
      transform->entry[i][2]=transform->entry[i][0]*sin_angle
			    +transform->entry[i][2]*cos_angle;
      transform->entry[i][0]=temp;
    }
}

/*============================================================================
				 MROTZ3D
==============================================================================
	THIS FUNCTION TAKES A MATRIX AS INPUT AND COMPOUNDS IT WITH A ROTATION
MATRIX ABOUT THE Z-AXIS. THE DEGREE OF ROTATION IS ALSO DEFINED BY THE INPUTS
============================================================================*/
void mrotz3d(matrix transform,float sin_angle,float cos_angle)
{
 int i;
 float temp,x,y,z;

 for(i=0;i<4;i++)
    {
      temp=transform->entry[i][0]*cos_angle-transform->entry[i][1]*sin_angle;
      transform->entry[i][1]=transform->entry[i][0]*sin_angle
			    +transform->entry[i][1]*cos_angle;
      transform->entry[i][0]=temp;
    }
}

/*============================================================================
			   INITIALISE_MATRIX
==============================================================================
     THIS FUNCTION INITIALISES A 4*3 MATRIX TO AN IDENTITY MATRIX
============================================================================*/
void initialise_matrix(matrix mat)
{
 int count,count2;

  for(count=0;count<4;count++)
    for(count2=0;count2<3;count2++)
       {
	mat->entry[count][count2]=0;
       }
}

/*============================================================================
			      MTRANSLATE_3D
==============================================================================
     THIS FUNCTION TAKES A MATRIX AS INPUT AND COMPOUNDS IT WITH A TRANSLATION
MATRIX.
============================================================================*/
void mtranslate_3d(matrix transmat,float xtrans,float ytrans,float ztrans)
{
  transmat->entry[3][0]=transmat->entry[3][0]+(float)xtrans;
  transmat->entry[3][1]=transmat->entry[3][1]+(float)ytrans;
  transmat->entry[3][2]=transmat->entry[3][2]+(float)ztrans;
}

/*============================================================================
				 SCALE
==============================================================================
	THIS FUNCTION TAKES A MATRIX AS INPUT AND COMPOUNDS IT WITH A SCALE
MATRIX.
============================================================================*/
void scale(matrix transmat,float scale)
{
 transmat->entry[0][0]=transmat->entry[0][0]*scale;
 transmat->entry[1][1]=transmat->entry[1][1]*scale;
 transmat->entry[2][2]=transmat->entry[2][2]*scale;
}


/*============================================================================
			     TRANSFORM_POINT
==============================================================================
	THIS FUNCTION TAKES A TRANSFORMATION MATRIX AS INPUT AND APPLIES IT
TO A POINT.
============================================================================*/
void transform_point(matrix transform,float *x,float *y, float *z)
{
  int i;
  float temp_points[3];

  for(i=0;i<3;i++)
     temp_points[i]=*x*transform->entry[0][i]+*y*transform->entry[1][i]
		   +*z*transform->entry[2][i]+transform->entry[3][i];
     *x=temp_points[0];
     *y=temp_points[1];
     *z=temp_points[2];
 }

/*============================================================================
==============================================================================
			     CLIPPING ROUTINES
==============================================================================
============================================================================*/

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
void clip_planes(float *x,float *y,float *z,float *x2,float *y2,float *z2)
{
  int outcode[2]={0,0};
  int *outcode1;
  int *outcode2;
  int count;
  float temp_x,temp_y,temp_z,t;

  outcode1=calculate_outcode(*x,*y,*z);  /*CALCULATE OUTCODES FOR END-POINTS*/
  outcode2=calculate_outcode(*x2,*y2,*z2);

  while((outcode1[0]!=0 && outcode1[1]!=0)||(outcode2[0]!=0 && outcode2[1]!=0))
   if((outcode1[0]==outcode2[0])||(outcode2[1]==outcode1[1])
      ||(outcode1[1]==outcode2[0]) || (outcode1[0]==outcode2[1]))
     {
      *z=-10000;
      *z2=-10000;
      return;       /*LINE NOT VISIBLE*/
     }
   else
     {
      for(count=0;count<2;count++)
	 {
	  if(outcode1[0]!=0 && outcode1[1]!=0)
	     outcode[count]=outcode1[count];
	  else
	     outcode[count]=outcode2[count];
	}
       if(outcode[0]==LEFT_CLIP)          /*CLIP AGAINST LEFT-CLIP PLANE*/
	  {
	   t=(*z+*x)/((*x-*x2)-(*z2,*z));
	   temp_z=t*(*z2-*z)+*z;
	   temp_x=-temp_z;
	   temp_y=t*(*y2-*y)+*y;
	  }

       if(outcode[0]==RIGHT_CLIP)         /*CLIP AGAINST RIGHT-CLIP PLANE*/
	  {
	   t=(*z-*x)/((*x2-*x)-(*z2-*z));
	   temp_z=t*(*z2-*z)+*z;
	   temp_x=temp_z;
	   temp_y=t*(*y2-*y)+*y;
	  }
					 /*CLIP AGAINST BOTTOM-CLIP PLANE*/
       if((outcode[0]==BOTTOM_CLIP)||(outcode[1]==BOTTOM_CLIP))
	  {
	    t=(*z+*y)/((*y-*y2)-(*z2-*z));
	    temp_z=t*(*z2-*z)+*z;
	    temp_x=t*(*x2-*x)+*x;
	    temp_y=-temp_z;
	  }
					/*CLIP AGAINST TOP-CLIP PLANE*/
       if((outcode[0]==TOP_CLIP)||(outcode[1]==TOP_CLIP))
	  {
	    t=(*z-*y)/((*y2-*y)-(*z2-*z));
	    temp_z=t*(*z2-*z)+*z;
	    temp_x=t*(*x2-*x)+*x;
	    temp_y=temp_z;
	  }

       if((outcode[0]==outcode1[0])&&(outcode[1]==outcode1[1]))
	 {
	   *x=temp_x;
	   *y=temp_y;
	   *z=temp_z;
	   free(outcode1);
	   /*CALCULATE NEW OUTCODES AND START AGAIN*/
	   outcode1=calculate_outcode(temp_x,temp_y,temp_z);
	 }
       else
	 {
	   *x2=temp_x;
	   *y2=temp_y;
	   *z2=temp_z;
	   free(outcode2);
	   /*CALCULATE NEW OUTCODES AND START AGAIN*/
	   outcode2=calculate_outcode(temp_x,temp_y,temp_z);
	 }
    }
    count=0;
}

/*============================================================================
			       Z_CLIP
==============================================================================
	THIS FUNCTION CLIPS A LINE AGAINST THE Z-PLANE, RETURNING THE NEW
CLIPPED END-POINTS.
============================================================================*/
void z_clip(float *x,float *y,float *z,float x2,float y2,float z2)
{
 float z_change;
 float front_z=20;   /*FRONT CLIPPING PLANE*/

 if ((*z<=front_z && z2>front_z)||(*z>=front_z && z2<front_z))
   {
     z_change=(front_z-*z)/(*z-z2);
     *x=(*x-x2)*z_change+*x;
     *y=(*y-y2)*z_change+*y;
     *z=front_z;
   }
 else
   if ((*z<front_z && z2<front_z))
     *x=*y=*z=-10000;             /*LINE NOT VISIBLE*/
}


/*============================================================================
==============================================================================
			   TRANSFORMATION ROUTINES
==============================================================================
============================================================================*/


/*=============================================================================
			     CREATE_VIEW
==============================================================================
	THIS FUNCTION APPLIES THE COMPOUND MATRIX CREATED IN TRANS_EYE
TO THE OBJECT ARRAY PASSED TO IT AS A PARAMETER.
============================================================================*/
void create_view(shape objects,matrix transform)
{
 int count1,count2;

 float x,y,z,x2,y2,z2;

 update_transform(objects);
 transform_projection_point(transform);

 /*EXTRAC ONE LINE AT A TIME (IE. TWO END-POINTS*/
 for(count1=0;count1<objects->no_of_lines;count1++)
     {
      x=objects->vert_array[objects->line_array[count1][0]][0];
      y=objects->vert_array[objects->line_array[count1][0]][1];
      z=objects->vert_array[objects->line_array[count1][0]][2];
      x2=objects->vert_array[objects->line_array[count1][1]][0];
      y2=objects->vert_array[objects->line_array[count1][1]][1];
      z2=objects->vert_array[objects->line_array[count1][1]][2];

      /*APPLY THE OBJECTS PARTICULAR TRANSFORM MATRIX TO THE END-POINTS*/
      transform_point(objects->shape_mat,&x,&y,&z);
      transform_point(objects->shape_mat,&x2,&y2,&z2);

      /*INVERT Z-AXES TO BRING FROM LEFT-HANDED TO RIGHT-HANDED SYSTEM*/
      z=-z;
      z2=-z2;

      /*APPLY THE WORLD TO EYE TRANSFORM MATRIX TO THE END-POINTS*/
      transform_point(transform,&x,&y,&z);
      transform_point(transform,&x2,&y2,&z2);

      /*A LITTLE MATHEMATICAL TRICKERY TO MAKE THE CLIPPING EASIER*/
      x=x*(5/(WINHEIGHT/2));
      x2=x2*(5/(WINHEIGHT/2));
      y=y*(5/(WINHEIGHT/2));
      y2=y2*(5/(WINHEIGHT/2));

      /*CLIP LINE AGAINST THE TOP, BOTTOM, LEFT AND RIGHT CLIPPING PLANES*/
      clip_planes(&x,&y,&z,&x2,&y2,&z2);

      /*IS THE LINE VISIBLE*/
      if(z!=-10000 && z2!=-10000)
	 {
	  /*IF SO, CLIP AGAINST THE Z-PLANE*/
	   if((z<z2))
	     z_clip(&x,&y,&z,x2,y2,z2);
	   else
	     z_clip(&x2,&y2,&z2,x,y,z);

	   /*IF IT IS STILL VISIBLE THEN DRAW IT*/
	   if(z!=-10000 && z2!=-10000)
	     convert_3d_to_2d(objects->color,x,y,z,x2,y2,z2);
	 }
    }
}

/*============================================================================
			    CONVERT_3D_TO_2D
==============================================================================
    THIS FUNCTION TAKES TWO 3-D LINE END-POINTS AS PARAMETERS AND MAPS THEM
INTO A 2-D LINE ON THE SCREEN.
============================================================================*/
void convert_3d_to_2d(int color,float x,float y,float z,float x2,float y2, float z2)
{
  float x_2d,y_2d,x2_2d,y2_2d,i;

  /*FIRST END-POINT*/
  x_2d=(x/z)*(SCREENWIDTH/WINWIDTH)+(SCREENWIDTH/2);
  y_2d=(y/z)*(SCREENHEIGHT/WINHEIGHT)+(SCREENHEIGHT/2);

  /*SECOND END-POINT*/
  x2_2d=(x2/z2)*(SCREENWIDTH/WINWIDTH)+(SCREENWIDTH/2);
  y2_2d=(y2/z2)*(SCREENHEIGHT/WINHEIGHT)+(SCREENHEIGHT/2);

  /*DRAW IN OBJECTS COLOUR*/
  setcolor(color);

  line(x_2d,y_2d,x2_2d,y2_2d);
}

/*============================================================================
			TRANSFORM_PROJECTION_POINT
==============================================================================
     THIS FUNCTION APPLIES THE TRANSFORMATION MATRIX TO THE CENTRE OF
PROJECTION THUS TRANSFORMING IT TO EYE CO-ORDINATES.
============================================================================*/
void transform_projection_point(matrix transform)
{
 xc=xpcntr;
 yc=ypcntr;
 zc=zpcntr;
 transform_point(transform,&xc,&yc,&zc);
}



/*============================================================================
			    UPDATE_TRANSFORM
==============================================================================
     THIS FUNCTION UPDATES THE TRANSFORM MATRIX ASSOCIATED WITH EACH OBJECT
FOR THE ROTATING AND MOVING OF THE OBJECT IN WORLD SPACE.
============================================================================*/
void update_transform(shape objects)
{
 /*SETUP THE TRANSFORM MATRIX AS AN IDENTITY MATRIX*/
 initialise_matrix(objects->shape_mat);
 objects->shape_mat->entry[0][0]=objects->shape_mat->entry[1][1]=objects->shape_mat->entry[2][2]=1;

 /*UPDATE THE CURRENT ROTATION ANGLE OF THE OBJECT*/
  objects->curr_y_angle+=objects->y_rot;
  objects->curr_z_angle+=objects->z_rot;

  /*CHECK IF THE OBJECT HAS MET THE BOUNDING CUBE IF SO REVERSE MOVEMENT
    DIRECTION AS APPROPRIATE*/
  if (objects->wx+objects->x_dir>=400 ||objects->wx+objects->x_dir<=-400)
      objects->x_dir=objects->x_dir*-1;
  if (objects->wy+objects->y_dir>=400 ||objects->wy+objects->y_dir<=-400)
      objects->y_dir=objects->y_dir*-1;
  if (objects->wz+objects->z_dir>=400 ||objects->wz+objects->z_dir<=-400)
      objects->z_dir=objects->z_dir*-1;

  /*UPDATE OBJECTS CURRENT WORLD POSITION*/
  objects->wx+=objects->x_dir;
  objects->wy+=objects->y_dir;
  objects->wz+=objects->z_dir;

  /*SCALE THE OBJECT ACCORDING TO ITS RANDOMLY CHOSEN SCALE FACTOR*/
  scale(objects->shape_mat,objects->scale_factor);

  /*ROTATE THE OBJECT ABOUT THE Z AXIS*/
  mrotz3d(objects->shape_mat,sin(objects->curr_y_angle/D_TO_R),cos(objects->curr_y_angle/D_TO_R));

  /*ROTATE THE OBJECT ABOUT THE Y AXIS*/
  mroty3d(objects->shape_mat,sin(objects->curr_y_angle/D_TO_R),cos(objects->curr_y_angle/D_TO_R));

  /*TRANSLATE OBJECT TO ITS WORLD POSITION*/
  mtranslate_3d(objects->shape_mat,objects->wx,objects->wy,objects->wz);

  }

/*============================================================================
	THIS FUNCTION INITIALISES ALL THE ATTRIBUTES OF A SHAPE
============================================================================*/
void init_shape(shape objects,float world_x,float world_y,float world_z,int scale,float verts[][3],int no_of_verts,float lines[][2],int no_of_lines)
{
 int count1,count2;
 float temp_verts[30][3];
 float temp_lines[30][2];

 /*COPY UNIT SHAPES VERTICES INTO OBJECT VERTEX STORE*/
 for(count1=0;count1<no_of_verts;count1++)
     for(count2=0;count2<3;count2++)
	 objects->vert_array[count1][count2]=verts[count1][count2];


 /*COPY UNIT SHAPES LINE DEFINITIONS INTO OBJECT LINE DEFINITIONS*/
 for(count1=0;count1<no_of_lines;count1++)
   for(count2=0;count2<2;count2++)
      objects->line_array[count1][count2]=lines[count1][count2];

 /*RECORD THE NO. OF VERTICES*/
 objects->no_of_verts=no_of_verts;

 /*RECORD THE NO. OF LINES*/
 objects->no_of_lines=no_of_lines;

 /*INITIALISE THE OBJECTS TRANSFORM MATRIX USED FOR ROTATIONS, SCALING ETC.*/
 initialise_matrix(objects->shape_mat);
 objects->shape_mat->entry[0][0]=objects->shape_mat->entry[1][1]=objects->shape_mat->entry[2][2]=1;

 /*SELECT A RANDOM DIRECTION AND SPEED FOR THE OBJECT TO MOVE ALONG*/
 objects->x_dir=(float)(random_select(100)-50)/5;
 objects->y_dir=(float)(random_select(100)-50)/5;
 objects->z_dir=(float)(random_select(100)-50)/5;

/*SELECT RANDOM DIRECTION FOR THE OBJECT TO ROTATE IN*/
 if((float)(random_select(10))>5)
   {
    objects->y_rot=-2;
    objects->z_rot=-2;
   }
 else
   {
    objects->y_rot=2;
    objects->z_rot=2;
   }

 /*INITIALISE THE CURRENT ROTATION ANGLES TO ZERO*/
 objects->curr_x_angle=objects->curr_y_angle=objects->curr_z_angle=0.0;

 /*RECORD OBJECTS WORLD LOCATION*/
 objects->wx=world_x;
 objects->wy=world_y;
 objects->wz=world_z;

 /*CHOOSE A RANDOM SIZE FOR THE OBJECT*/
 objects->scale_factor=(float)(((random_select(100))/10)+1);

 /*CHOOSE A RANDOM COLOR FOR THE OBJECT*/
 objects->color=random_select(14)+1;

}

/*============================================================================
			       TRAN_EYE
==============================================================================
       THIS FUNCTION CALCULATES THE COUMPOUND MATRIX NECESSARY TO TRANSLATE
FROM WORLD CO-ORDINATES TO EYE CO-ORDINATES.
============================================================================*/
matrix trans_eye()
{
 float rot;
 float rup,xup_vp,yup_vp;
 matrix transform;

 /*ALLOCATE MEMORY TO A TRANSFORM MATRIX*/
 transform=mat3d();

 /*INITIALISE MATRIX TO BE AN IDENTITY MATRIX*/
 initialise_matrix(transform);

 transform->entry[0][0]=transform->entry[1][1]=transform->entry[2][2]=1;

 /*TRANSLATE VIEWPOINT TO ORIGIN*/
 mtranslate_3d(transform,-(xr+dxn*view_distance),-(yr+dyn*view_distance),
	       -(zr+dzn*view_distance));

 /*CALCULATE THE COS OF THE ANGLE NEEDED TO ROTATE ABOUT THE Y-AXIS*/
 rot=sqrt(dyn*dyn+dzn*dzn);

 /*HANDILY ENOUGH THE COS AND SINE OF THE ANGLE TO ROTATE ABOUT THE AXIS
   CAN EASILY BE CALCULATED*/

 /*ROTATE ABOUT X-AXIS*/
 if (rot>0.001)
     mrotx3d(transform,-dyn/rot,-dzn/rot);

/*ROTATE ABOUT Y-AXIS*/
 mroty3d(transform,dxn,rot);

/*CALUCLATE ROTATIONS NECESSARY TO ALLIGN VIEW-UP VECTOR*/
 xup_vp=dxup*transform->entry[0][0]+dyup*transform->entry[1][0]
       +dzup*transform->entry[2][0];
 yup_vp=dxup*transform->entry[0][1]+dyup*transform->entry[1][1]
       +dzup*transform->entry[2][1];
 rup=sqrt(xup_vp*xup_vp+yup_vp*yup_vp);


 /*ROTATE AROUND Z-AXIS*/
 mrotz3d(transform,xup_vp/rup,yup_vp/rup);
 return transform;
}


/*============================================================================
==============================================================================
			    MISCELLANEOUS ROUTINES
==============================================================================
============================================================================*/


/*=============================================================================
			    SET_VIEW_REFERENCE_POINT
============================================================================*/
void set_view_reference_point(float x,float y,float z)
{
  xr=x;
  yr=y;
  zr=z;
}

/*============================================================================
			       SET_PERSPECTIVE
============================================================================*/
void set_perspective(float x,float y, float z)
{
  xpcntr=x;
  ypcntr=y;
  zpcntr=z;
}

/*============================================================================
			      RANDOM_SELECT
==============================================================================
	THIS FUNCTION RETURNS A RANDOM NUMBER IN THE RANGE DEFINED BY THE
INPUT PARAMETER.
============================================================================*/
int random_select(int range)
{
  return (rand()%range);
}


/*============================================================================
			ALLOCATE_SHAPE_MEM
==============================================================================
	THIS FUNCTION ALLOCATES MEMORY TO ALL THE SHAPE STRUCTURES AND
POINTERS TO SHAPE STRUCTURES THAT WILL BE USED.
============================================================================*/
shape *allocate_shape_mem(int no_of_shapes)
{
 int count;
 shape *shapes;

 /*ALLOCATE MEMORY TO POINTERS*/
 shapes=(shape*)malloc(no_of_shapes*sizeof(shape));

 /*ALLOCATE MEMORY TO ALL SHAPES*/
 for(count=0;count<no_of_shapes;count++)
   {
    shapes[count]=shape_3d();        /*ALLOCATE MEMORY TO 1 SHAPE*/
    shapes[count]->shape_mat=mat3d();/*ALLOCATE MEMORY TO TRANSFORM MATRIX*/
   }

 return shapes;
}

/*============================================================================
			      ALLOCATE_SHAPES
==============================================================================
    ALLOCATE DIFFERENT SHAPE ATTRIBUTES TO THE OBJECTS
============================================================================*/
void allocate_shapes(shape *shapes,int no_of_pyramids,int no_of_squares,int no_of_diamonds)
{
  int count,rand_x,rand_y;

  for(count=0;count<no_of_pyramids;count++)
      {
       /*SELECT A RANDOM LOCATION FOR EACH PYRAMID*/
       rand_x=random_select(200);
       rand_y=random_select(100);
       /*INITIALISE A PYRAMID*/
       init_shape(shapes[count],rand_x,rand_y,200.0,10,pyramid,5,pyramid_lines,8);
      }
  for(count=no_of_pyramids;count<no_of_pyramids+no_of_squares;count++)
     {
      /*SELECT A RANDOM LOCATION FOR EACH SQUARE*/
      rand_x=random_select(200);
      rand_y=random_select(100);
      /*INITIALISE A SQUARE*/
      init_shape(shapes[count],rand_x,rand_y,200.0,10,cube,8,cube_lines,12);
     }
  for(count=no_of_pyramids+no_of_squares;count<no_of_diamonds+no_of_pyramids+no_of_squares;count++)
      {
       /*SELECT A RANDOM LOCATION FOR EACH DIAMOND*/
       rand_x=random_select(200);
       rand_y=random_select(100);
       /*INITIALISE A DIAMOND*/
       init_shape(shapes[count],rand_x,rand_y,200.0,10,diamond,6,diamond_lines,12);
      }

}

/*============================================================================
			   BOUNDING_BOX
==============================================================================
	THIS FUNCTION DEFINES THE BOUNDING BOX IN WHICH ALL OTHER SHAPES WILL
BE CONTAINED.
============================================================================*/
void bounding_box(shape shapes)
{
  init_shape(shapes,100,0,200.0,10,cube,8,cube_lines,12);
  shapes->scale_factor=100;
  shapes->x_dir=0;
  shapes->y_dir=0;
  shapes->z_dir=0;
}

/*============================================================================
				  BANK
==============================================================================
	CHANGE THE VIEW-UP ANGLE.
============================================================================*/
void bank(float bank_angle)
{
 bank_angle=bank_angle/D_TO_R; /*CONVERT TO RADIANS*/
 set_view_up(sin(bank_angle)*dzn,cos(bank_angle)*sqrt(dxn*dxn+dzn*dzn),
	     -sin(bank_angle)*dxn);
}

/*============================================================================
				 SET_VIEW_UP
============================================================================*/
void set_view_up(float x,float y,float z)
{
 dxup=x;
 dyup=y;
 dzup=z;
}













