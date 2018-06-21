/*****************************************************************************
		THIS MODULE CONTAINS ALL MATHEMATATICAL AND OTHER FUNCTIONS PERTAINING
 TO COORDINATE GEOMETRY CALCULATIONS.
*****************************************************************************/
#include <stdio.h>
#include <graphics.h>
#include <alloc.h>
#include <stdlib.h>
#include <math.h>
#include "objects.h"
#include "geo.h"

float quickcos[360];
float quicksin[360];
float quicktan[360];

/*============================================================================
							   LINE_LENGTH
==============================================================================
	Calculate the length of a line.
============================================================================*/
float line_length(float x1,float y1,float x2,float y2)
{
  return sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
}

/*============================================================================
							INITIALISE_MATRIX
==============================================================================
	Initialises the entries in a matrix to zero.
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
	Compunds the input matrix with a translation matrix.
============================================================================*/
void mtranslate_3d(matrix transmat,float xtrans,float ytrans,float ztrans)
{
  transmat->entry[3][0]=transmat->entry[3][0]+(float)xtrans;
  transmat->entry[3][1]=transmat->entry[3][1]+(float)ytrans;
  transmat->entry[3][2]=transmat->entry[3][2]+(float)ztrans;
}

/*============================================================================
								MROTX3D
==============================================================================
	Compounds the input matrix with a rotation matrix about the x-axis.
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
	Compounds the input matrix with a rotation matrix about the y-axis.
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
	Compounds the input matrix with a rotation matrix about the z-axis.
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
								 SCALE
==============================================================================
	Compounds the input matrix with a scale matrix.
============================================================================*/
void scale(matrix transmat,float scale_x,float scale_y,float scale_z)
{
 transmat->entry[0][0]=transmat->entry[0][0]*scale_x;
 transmat->entry[1][1]=transmat->entry[1][1]*scale_y;
 transmat->entry[2][2]=transmat->entry[2][2]*scale_z;
}

/*============================================================================
						   CONVERT_3D_TO_2D
==============================================================================
		Performs a perspective projection of a line in 3-D space.
============================================================================*/
void convert_3d_to_2d(int count,float *x,float *y,float z,float *x2,float *y2, float z2)
{

  *x=(((*x))/(z))*639+370;
  *y=(((*y))/(z))*350+175;


  *x2=/*((x2*zc-xc*z2)/(zc-z2))*/(((*x2))/(z2))*639+370;
  *y2=/*((y2*zc-yc*z2)/(zc-z2))*/(((*y2))/(z2))*350+175;

  if((*x<0 && *x2<0)||(*x>639 && *x2>639)||(*y<0 && *y2<0)||(*y>200 && *y2>200))
	 return;
  line(*x,*y,*x2,*y2);
}

/*============================================================================
								  GET_SLOPE
==============================================================================
	Calculates the slope of a given line.
============================================================================*/
float get_slope(float x1,float y1,float x2,float y2)
{
  if(x2-x1!=0)
	 return((y2-y1)/(x2-x1));
  else
	 return 0;
}

/*============================================================================
							  ANGLE_TO_COORD
==============================================================================
	Converts an angle to a coordinate on a unit circle.
============================================================================*/
void angle_to_coord(float *x,float *y,float angle)
{
 float cos_angle,sin_angle;

 cos_angle=round_error(cos(angle/D_TO_R));
 sin_angle=round_error(sin(angle/D_TO_R));

 if(cos_angle>1 || cos_angle<-1)
	cos_angle=0;

 *x=(sin_angle);
 *y=(cos_angle)*0.67;   /*ALLOW FOR ASPECT RATIO*/
}

/*============================================================================
							  COORD_TO_ANGLE
==============================================================================
	Calulates the angle formed by joining two points.
============================================================================*/
float coord_to_angle(float x1,float y1,float x2,float y2)
{
  float slope;
  float angle;

  slope=get_slope(x1,y1,x2,y2);

  if(slope<0)    /*LINE IN 2nd OR 4th QUADRANT*/
	 {
	  angle=atan(slope)*D_TO_R;
	  if(x1<x2) return (270-angle);
	  else      return (90-angle);
	 }
  if(slope>0)    /*LINE IN 1st OR 3rd QUADRANT*/
	 {
	  angle=atan(slope)*D_TO_R;
	  if(x1<x2) return (270-angle);
	  else      return (90-angle);
	 }
  if(slope==0)
	 {
	  if(y1>y2) return 0;
	  if(y1<y2) return 180;
	  if(x1<x2) return 270;
	  else      return 90;
	 }
}

/*============================================================================
								ROUND_ERROR
==============================================================================
		Calculates round-off error and rounds of values accordingly.
============================================================================*/
float round_error(float value)
{
 if(fabs(value)<=UNDERFLOW)
	return 0;
 else return (float)value;
}

/*============================================================================
								SET_COS_TABLE
==============================================================================
	Sets up a table of cosines.
============================================================================*/
void set_cos_table()
{
 int i;

 for(i=0;i<360;i++)
	 quickcos[i]=round_error(cos(i/D_TO_R));
 quickcos[90]=quickcos[270]=0;
}

/*============================================================================
								SET_SIN_TABLE
==============================================================================
	Sets up a table of sines.
============================================================================*/
void set_sin_table()
{
 int i;

 for(i=0;i<360;i++)
	 quicksin[i]=round_error(sin(i/D_TO_R));
 quicksin[180]=0;
}

/*============================================================================
								SET_TAN_TABLE
==============================================================================
		Sets up a table of Tans
============================================================================*/
void set_tan_table()
{
 int i;

 for(i=0;i<360;i++)
	 quicktan[i]=round_error(tan(i/D_TO_R));
}

/*============================================================================
								QUICK_COS
==============================================================================
	Looks up cosine table.
============================================================================*/
float quick_cos(float angle)
{
  if(angle<1)
	 return(quickcos[-(int)angle]);
  else
  return quickcos[(int)angle];
}

/*============================================================================
								QUICK_SIN
==============================================================================
		Looks up sine table.
============================================================================*/
float quick_sin(float angle)
{
  if(angle<0)
	 return (-quicksin[-(int)angle]);
  else
	 return quicksin[(int)angle];
}

/*============================================================================
								QUICK_TAN
==============================================================================
		Looks up tan table.
============================================================================*/
float quick_tan(float angle)
{
  if(angle<0)
	 return -quicktan[-(int)angle];
  else
	 return quicktan[(int)angle];
}


/*============================================================================
								 MAX
==============================================================================
		Returns largest of two values.
============================================================================*/
float max(float value1, float value2)
{
   return ( (value1 > value2) ? value1 : value2);
}

/*============================================================================
								MIN
==============================================================================
		Returns smallest of two values.
============================================================================*/
float min(float value1, float value2)
{
   return ( (value1 < value2) ? value1 : value2);
}








