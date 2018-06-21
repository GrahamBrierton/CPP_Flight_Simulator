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

shape init_shape(shape,float,float,float,int,float[][3],int,float[][2],int);
void initialise_matrix(matrix);
void mtranslate_3d(matrix,float,float,float);
void bounding_box(shape);
shape shape_3d(void);
void clip(float*,float*,float*,float,float,float);
void transform_point(matrix,float*,float*,float*);
void set_perspective(float,float,float);
void set_view_reference_point(float,float,float);
void mroty3d(matrix,float,float);
void mrotz3d(matrix,float,float);
void transform_projection_point(matrix);
void convert_3d_to_2d(int,float,float,float,float,float,float);
matrix trans_eye();
matrix mat3d();
void mroty3d(matrix,float,float);
void mrotz3d(matrix,float,float);
void create_view(shape,matrix);
shape *allocate_shape_mem(int);
void allocate_shapes(shape *,int,int,int);
int  random_select(int);
void scale(matrix,float);

#define D_TO_R 57.29575

float xr=0.0,yr=0.0,zr=0.0;
float xc=0.0,yc=0.0,zc=0.0;
float xpcntr=0.0,ypcntr=0.0,zpcntr=0.0;
float dxn=0.0,dyn=0.0;
float dzn=-1.0;
float dxup=0.0,dzup=0.0;
float dyup=1.0;
float view_distance=0;

main()
{
  int driver=VGA;
  static int mode,i;
  shape *square1;
  int page=1,count,stime,read_key,turn_angle=0,speed=0;
  long ltime;
  float tempx=50;
  float tempy=10;
  float tempz=-50;
  int no_of_pyramids=0;
  int no_of_cubes=2;
  int no_of_diamonds=0;
  matrix transform_mat=mat3d();


  mode=VGAMED;
  initgraph(&driver,&mode,"g:\\c\\bc3.1\\bgi");

  /*SEED RANDOM NUMBER GENERATOR*/
  ltime=time(NULL);
  stime=(unsigned int) ltime/2;
  srand(stime);

/*  no_of_pyramids=random_select(5);
  no_of_cubes=random_select(5);*/

  square1=allocate_shape_mem(no_of_pyramids+no_of_cubes+no_of_diamonds+1);

  allocate_shapes(square1,no_of_pyramids,no_of_cubes,no_of_diamonds);

  bounding_box(square1[no_of_pyramids+no_of_cubes+no_of_diamonds]);



  while(read_key!=7181)
  {
    if(bioskey(1)!=0)
      read_key=bioskey(0);
      else read_key=0;
   if(read_key!=0)
     {
    switch(read_key)
      {
	case(UP) :speed++;
		  break;
	case(DOWN) :speed--;
		    break;
	case(LEFT): turn_angle++;
		    break;
	case(RIGHT): turn_angle--;
		    break;
      }
     }
     tempz+=speed;
	setactivepage(page&1);
	setviewport(0,0,639,350,1);
	clearviewport();
	set_view_reference_point(tempx,tempy,tempz);
	set_perspective(tempx-5*dxn,tempy-5*dyn,tempz-5*dzn);
	transform_mat=trans_eye();
	for(count=0;count<no_of_pyramids+no_of_cubes+no_of_diamonds;count++)
	   create_view(square1[count],transform_mat);
	free(transform_mat);
	setvisualpage(page&1);
	page++;

  }



}

void convert_3d_to_2d(int color,float x,float y,float z,float x2,float y2, float z2)
{
  float x_2d,y_2d,x2_2d,y2_2d,i;

  x_2d=((x*zc-xc*z)/(zc-z))*64+370;
  y_2d=((y*zc-yc*z)/(zc-z))*42+200;

  x2_2d=((x2*zc-xc*z2)/(zc-z2))*64+370;
  y2_2d=((y2*zc-yc*z2)/(zc-z2))*42+200;

  setcolor(color);
  line(x_2d,y_2d,x2_2d,y2_2d);
/*  if (count==0)
   {  putpixel(x_2d,y_2d,8);
     moveto(x_2d,y_2d);}
  else
     lineto(x_2d,y_2d);*/
}


void transform_projection_point(matrix transform)
{
 xc=xpcntr;
 yc=ypcntr;
 zc=zpcntr;
 transform_point(transform,&xc,&yc,&zc);
}


shape shape_3d()
{
 return (shape)malloc(sizeof(SHAPE));
}


void update_transform(shape shape1)
{
 initialise_matrix(shape1->shape_mat);
 shape1->shape_mat->entry[0][0]=shape1->shape_mat->entry[1][1]=shape1->shape_mat->entry[2][2]=1;


  shape1->curr_y_angle+=2;/*shape1->y_rot;*/
  shape1->curr_z_angle+=2;/*shape1->z_rot;*/
  if (shape1->wx+shape1->x_dir>1000 ||shape1->wx+shape1->x_dir<-1000)
      shape1->x_dir=shape1->x_dir*-1;
  if (shape1->wy+shape1->y_dir>1000 ||shape1->wy+shape1->y_dir<-1000)
      shape1->y_dir=shape1->y_dir*-1;

  shape1->wx+=shape1->x_dir;
  shape1->wy+=shape1->y_dir;
  shape1->wz+=shape1->z_dir;
  scale(shape1->shape_mat,shape1->scale_factor);
  mrotz3d(shape1->shape_mat,sin(shape1->curr_y_angle/D_TO_R),cos(shape1->curr_y_angle/D_TO_R));
  mroty3d(shape1->shape_mat,sin(shape1->curr_y_angle/D_TO_R),cos(shape1->curr_y_angle/D_TO_R));
  mtranslate_3d(shape1->shape_mat,shape1->wx,shape1->wy,shape1->wz);

  }

matrix mat3d()
{
 return (matrix)malloc(sizeof(MATRIX));
}

shape init_shape(shape shape1,float world_x,float world_y,float world_z,int scale,float verts[][3],int no_of_verts,float lines[][2],int no_of_lines)
{
 int count1,count2;
 float temp_verts[30][3];
 float temp_lines[30][2];

 for(count1=0;count1<no_of_verts;count1++)
     for(count2=0;count2<3;count2++)
	 shape1->vert_array[count1]->co_ord[count2]=temp_verts[count1][count2];

 for(count1=0;count1<no_of_lines;count1++)
   for(count2=0;count2<2;count2++)
      shape1->line_array[count1]->vert[count2]=lines[count1][count2];

 shape1->no_of_verts=no_of_verts;
 shape1->no_of_lines=no_of_lines;
 initialise_matrix(shape1->shape_mat);
 shape1->shape_mat->entry[0][0]=shape1->shape_mat->entry[1][1]=shape1->shape_mat->entry[2][2]=1;
 mtranslate_3d(shape1->shape_mat,world_x,world_y,world_z);
 shape1->x_dir=(float)(random_select(100)-50)/50;
 shape1->y_dir=(float)(random_select(100)-50)/50;
 shape1->z_dir=(float)(random_select(100)-50)/50;
 shape1->x_rot=(float)(random_select(2));
 shape1->y_rot=2;/*(float)(random_select(2));*/
 shape1->z_rot=2;
 shape1->curr_x_angle=shape1->curr_y_angle=shape1->curr_z_angle=0.0;
 shape1->wx=world_x;
 shape1->wy=world_y;
 shape1->wz=world_z;
 shape1->scale_factor=1;/*(float)(random_select(40))/10;*/
 shape1->color=random_select(15);
 return (shape1);

}

void initialise_matrix(matrix mat)
{
 int count,count2;

  for(count=0;count<4;count++)
    for(count2=0;count2<3;count2++)
       {
	mat->entry[count][count2]=0;
       }
}

void mtranslate_3d(matrix transmat,float xtrans,float ytrans,float ztrans)
{
  transmat->entry[3][0]=transmat->entry[3][0]+(float)xtrans;
  transmat->entry[3][1]=transmat->entry[3][1]+(float)ytrans;
  transmat->entry[3][2]=transmat->entry[3][2]+(float)ztrans;
}

void scale(matrix transmat,float scale)
{
 transmat->entry[0][0]=transmat->entry[0][0]*scale;
 transmat->entry[1][1]=transmat->entry[1][1]*scale;
 transmat->entry[2][2]=transmat->entry[2][2]*scale;
}



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

matrix trans_eye()
{
 float rot;
 float rup,xup_vp,yup_vp;
 matrix transform;

 transform=mat3d();

 initialise_matrix(transform);

 transform->entry[0][0]=transform->entry[1][1]=transform->entry[2][2]=1;

 mtranslate_3d(transform,-(xr+dxn*view_distance),-(yr+dyn*view_distance),
	       -(zr+dzn*view_distance));
 rot=sqrt(dyn*dyn+dzn*dzn);
 mroty3d(transform,dxn,rot);
 xup_vp=dxup*transform->entry[0][0]+dyup*transform->entry[1][0]
       +dzup*transform->entry[2][0];
 yup_vp=dxup*transform->entry[0][1]+dyup*transform->entry[1][1]
       +dzup*transform->entry[2][1];
 rup=sqrt(xup_vp*xup_vp+yup_vp*yup_vp);
 mrotz3d(transform,xup_vp/rup,yup_vp/rup);
 return transform;
}

void create_view(shape shape1,matrix transform)
{
 int count1,count2;

 float x,y,z,x2,y2,z2;

 update_transform(shape1);
 transform_projection_point(transform);
 for(count1=0;count1<shape1->no_of_lines;count1++)
     {
      x=shape1->vert_array[shape1->line_array[count1]->vert[0]]->co_ord[0];
      y=shape1->vert_array[shape1->line_array[count1]->vert[0]]->co_ord[1];
      z=shape1->vert_array[shape1->line_array[count1]->vert[0]]->co_ord[2];
      x2=shape1->vert_array[shape1->line_array[count1]->vert[1]]->co_ord[0];
      y2=shape1->vert_array[shape1->line_array[count1]->vert[1]]->co_ord[1];
      z2=shape1->vert_array[shape1->line_array[count1]->vert[1]]->co_ord[2];

      transform_point(shape1->shape_mat,&x,&y,&z);
      transform_point(shape1->shape_mat,&x2,&y2,&z2);

      transform_point(transform,&x,&y,&z);
      transform_point(transform,&x2,&y2,&z2);

/*      if((z<z2))
	clip(&x,&y,&z,x2,y2,z2);
      else
	clip(&x2,&y2,&z2,x,y,z);
      if(z!=-1000 && z2!=-1000)*/
      convert_3d_to_2d(shape1->color,x,y,z,x2,y2,z2);
    }
}




void set_view_reference_point(float x,float y,float z)
{
  xr=x;
  yr=y;
  zr=z;
}

void set_perspective(float x,float y, float z)
{
  xpcntr=x;
  ypcntr=y;
  zpcntr=z;
}

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

int random_select(int range)
{
/*  randomize();*/
  return (rand()%range);
}

void clip(float *x,float *y,float *z,float x2,float y2,float z2)
{
 float z_change;
 float front_z=1;

 if ((*z<=front_z && z2>front_z)||(*z>=front_z && z2<front_z))
   {
     z_change=(front_z-*z)/(*z-z2);
     *x=(*x-x2)*z_change+*x;
     *y=(*y-y2)*z_change+*y;
     *z=front_z;
   }
 else
   if ((*z<=front_z && z2<=front_z))
     *x=*y=*z=-1000;
}

shape *allocate_shape_mem(int no_of_shapes)
{
 int count;
 shape *shapes;

 for(count=0;count<no_of_shapes;count++)
   {
    shapes[count]=shape_3d();
    shapes[count]->vert_array=(point*)(calloc(30,sizeof(VERTEX)));
    shapes[count]->line_array=(line_def*)(calloc(30,sizeof(EDGE)));
    shapes[count]->shape_mat=mat3d();
   }

 return shapes;
}

void allocate_shapes(shape *shapes,int no_of_pyramids,int no_of_squares,int no_of_diamonds)
{
  int count,rand_x,rand_y;
  shape temp;

  for(count=0;count<no_of_pyramids;count++)
      {
       rand_x=random_select(200);
       rand_y=random_select(100);
       init_shape(shapes[count],rand_x,rand_y,200.0,10,pyramid,5,pyramid_lines,8);
      }
  for(count=no_of_pyramids;count<no_of_pyramids+no_of_squares;count++)
     {
      rand_x=random_select(200);
      rand_y=random_select(100);
      temp=shapes[count];
      shapes[count]=init_shape(shapes[count],rand_x,rand_y,200.0,10,cube,8,cube_lines,12);
     }

  for(count=no_of_pyramids+no_of_squares;count<no_of_diamonds+no_of_pyramids+no_of_squares;count++)
      {
       rand_x=random_select(200);
       rand_y=random_select(100);
       init_shape(shapes[count],rand_x,rand_y,200.0,10,diamond,6,diamond_lines,12);
      }

}

void bounding_box(shape shapes)
{
  init_shape(shapes,100,0,200.0,10,cube,8,cube_lines,12);
  shapes->scale_factor=100;
  shapes->x_dir=0;
  shapes->y_dir=0;
  shapes->z_dir=0;
}












