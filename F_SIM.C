#include <stdio.h>
#include <string.h>
#include <graphics.h>
#include <conio.h>
#include <bios.h>
#include <alloc.h>
#include <stdlib.h>
#include <math.h>
#include <io.h>
#include <fcntl.h>
#include <ctype.h>
#include <dos.h>
#include "objects.h"
#include "mem.h"

#define WINWIDTH 1
#define WINHEIGHT 1
#define SCREENWIDTH 640
#define SCREENHEIGHT 200
#define LEFT_CLIP 1
#define RIGHT_CLIP 2
#define BOTTOM_CLIP 3
#define TOP_CLIP 4
#define RETURN 7181
#define GRAVITY 32
#define PLANE_WEIGHT 8000
#define WING_AREA 340
#define CLIPPED 1
#define NOT_CLIPPED 0
#define NEAR 0
#define FAR  1
#define UP     18432
#define LEFT   19200
#define RIGHT  19712
#define DOWN   20480
#define AILERON_LEFT 7777
#define AILERON_RIGHT 8051
#define ACCELERATE 13358
#define DECELERATE 13100
#define D_TO_R 57.29575
#define SOLID 1
#define TRUE 1
#define FALSE 0
#define FRONT_VIEW 15104
#define LEFT_VIEW 15360
#define RIGHT_VIEW 15616
#define REAR_VIEW 15872
#define ON 1
#define OFF 0
#define GEAR 8807
#define MAXRPM 4000
#define EXTENT_FILL 3
#define UNDERFLOW 0.001
#define NAV1 1
#define NAV2 2
#define CHANGE_NAV1 16128
#define CHANGE_NAV2 16384
#define FLAPS_UP 3389
#define FLAPS_DOWN 3117
#define CHANGE_OMI_100 561
#define CHANGE_OMI_10 818
#define CHANGE_OMI_1 1075
#define MAXFUEL 4000
#define RUDDER_RIGHT 11640
#define RUDDER_LEFT 11386
#define BRAKES 12386
#define BRAKE_DRAG 1500
#define CUT_ENGINE 11875
#define ESCAPE 1
#define MENU 17408
#define WINTER 1
#define SUMMER 2
#define SPRING 3
#define AUTUMN 4

matrix trans_eye(plane);
void save_screen(void far *buf[4]);
void restore_screen(void far *buf[4]);
void initialise_matrix(matrix);
void update_orientation(plane,float);
void mtranslate_3d(matrix,float,float,float);
void load_world(int*,int*,int*,int*,int*,int*,FILE*);
vertices *load_verts(vertices*,int,FILE *);
facelist *load_faces(facelist*,int,FILE *);
shape *load_shapes(shape*,int,FILE *);
extent*load_extents(extent*,int,FILE*);
airport *load_airports(airport*,int,FILE*);
void load_beacons(nav,int,FILE*);
void mrotx3d(matrix,float,float);
void mroty3d(matrix,float,float);
void mrotz3d(matrix,float,float);
void transform_point(matrix,float*,float*,float*);
int  clip(float*,float*,float*,float,float,float);
void create_view(matrix,vertices*,facelist*,shape*,extent*,int,int);
void convert_3d_to_2d(int,float*,float*,float,float*,float*,float);
void set_perspective(float,float,float);
void set_view_reference_point(float,float,float);
void transform_projection_point(matrix);
void bank_plane(float,plane);
void move_plane(plane,nav);
void set_view_up(float,float,float,plane);
void draw_instruments(void);
void draw_sky(void);
void update_transform(shape);
void scale(matrix,float,float,float);
void create_horizon(matrix,float,nav,int);
void initialize_nav(nav);
void initialize_environment(environment);
void initialize_thrust_required(plane);
void update_instruments(nav,plane);
void update_altimeter(float);
void test_extents(extent*,int,matrix);
int clip_planes(float*,float*,float*,float*,float*,float*,int,int);
void angle_to_coord(float*,float*,float);
void update_speed(plane,nav);
void update_lift(plane,nav);
void update_fuel_flow(plane);
void update_weight_change(float);
void update_dive_angle(plane,nav);
void add_xval(scanline,float);
void mypoly(float *,int,int,int);
void draw_scanlines(scanline,int);
void delete_scanlines(scanline);
void update_artificial_horizon(float);
void update_rpm(float);
void update_time();
void update_speedometer(float);
void update_compass(float);
void update_flaps(int);
void change_flaps(plane,int);
void change_omi_radial(nav,int);
void update_fuel(float);
void polyclip(float*,int*,int*,int);
void line_intersect(float,float,float,float,float*,float*,int*,int);
int  inside(float,float,int,int*);
extern  hsub1(int,char);
extern  hsub2(int,char,char);
extern  hsub3(int,char,char,int);
extern void menu_bar(nav,plane,airport*,int);
extern void displaymap(char*,int,int);
void update_climb_angle(float);
void esetres(char);
void crash(int,int,int,int,int);
void set_new_palette(void);
void check_for_crash(plane,nav);
void setres(char);
void bitmask(char);
void mapmask(char);
void update_climb_rate(float,float);
void hline(int,int,int,int,int);
void update_gear(int);
void move_gear(plane);
void set_cos_table(void);
void set_sin_table(void);
void set_tan_table(void);
void update_navigation(nav);
void update_bank_recorder(float);
void update_climb_recorder(float);
void change_nav(nav,int);
void change_brakes(plane);
void update_flaps(int);
void update_dme(nav);
void update_omi(float,nav);
void update_brake(int);
int *calculate_outcode(float,float ,float);
int invisible(float,float);
intersect new_intersect(void);
scanline new_scanline(void);
int test(float,float,float);
float quick_tan(float);
float quick_cos(float);
float quick_sin(float);
float get_slope(float,float,float,float);
float coord_to_angle(float,float,float,float);
float round_error(float);
float line_length(float,float,float,float);

FILE *infile;
float xr=0.0,yr=0.0,zr=0.0;
float xc=0.0,yc=0.0,zc=0.0;
float xpcntr=0.0,ypcntr=0.0,zpcntr=0.0;
float view_distance=0.0;
float front_z=20.0;
int sky[]={0,0,639,0,639,160,0,160};
float MINDRAG=1.7;
float L_D=200;
float P_DRAG=400;
float quickcos[360];
float quicksin[360];
float quicktan[360];
int maxx;
int maxy;
int note=0;


float max (float value1, float value2);
float min (float value1,float value2);

float max(float value1, float value2)
{
   return ( (value1 > value2) ? value1 : value2);
}

float min(float value1, float value2)
{
   return ( (value1 < value2) ? value1 : value2);
}

void main()
{
  vertices *vertex_list;
  facelist *face_list;
  shape  *shape_list;
  extent *extent_list;
  airport *airport_list;
  matrix transform_mat;
  int driver=VGA;
  static int mode;
  int i,menu_choice;
  void far *ptr[4];
  char *view_text=(char*)malloc(30);

  plane plane_details=allocate_plane_mem();
  nav nav_details=allocate_nav_mem();
  environment environ_stats=allocate_environment_mem();

  initialize_plane(plane_details);
  initialize_nav(nav_details);
  initialize_thrust_required(plane_details);

  float temp_rot=0.0;
  int page=0;
  int read_key;
  int no_of_verts,no_of_faces,no_of_shapes,no_of_extents,no_of_beacons,no_of_airports;
  char *temp_str=(char*)(malloc(80));
  struct viewporttype info;
  int view_angle=0;
  int up_down;

  strcpy(view_text,"");
  set_sin_table();
  set_cos_table();
  set_tan_table();

  mode=VGAMED;
  initgraph(&driver,&mode,"g:\\c\\bc3.1\\bgi");


  maxx=getmaxx();
  maxy=getmaxy();

  setpalette(LIGHTGREEN,BLACK);
  setactivepage(1);
  draw_instruments();
/*  save_screen(ptr);*/
  setactivepage(0);
 /* restore_screen(ptr);*/
  draw_instruments();
  setpalette(0,GREEN);

  infile=fopen("world.dat","r");
  load_world(&no_of_verts,&no_of_faces,&no_of_shapes,&no_of_extents,&no_of_beacons,&no_of_airports,infile);
  load_beacons(nav_details,no_of_beacons,infile);
  airport_list=allocate_airport_mem(no_of_airports);
  airport_list=load_airports(airport_list,no_of_airports,infile);
  vertex_list=allocate_vert_mem(no_of_verts);
  vertex_list=load_verts(vertex_list,no_of_verts,infile);
  face_list=allocate_face_mem(no_of_faces);
  face_list=load_faces(face_list,no_of_faces,infile);
  shape_list=allocate_shape_mem(no_of_shapes);
  shape_list=load_shapes(shape_list,no_of_shapes,infile);
  extent_list=allocate_extent_mem(no_of_extents);
  extent_list=load_extents(extent_list,no_of_extents,infile);
  fclose(infile);


  update_orientation(plane_details,view_angle);

  while(read_key!=7181)
  {
   if(bioskey(1)!=0)
	  read_key=bioskey(0);
	  else read_key=0;
   if(read_key!=0)
	 {
	switch(read_key)
	  {
	case(ACCELERATE) : plane_details->thrust+=200;
			   if(plane_details->thrust>MAXRPM)
				  plane_details->thrust=MAXRPM;
			   break;
	case(DECELERATE): plane_details->thrust-=200;
			  if(plane_details->thrust<0)
				 plane_details->thrust=0;
			  break;

	case(UP) :plane_details->dive_angle+=0.2;
		  update_dive_angle(plane_details,nav_details);
		  break;
	case(DOWN) :plane_details->dive_angle-=0.2;
			update_dive_angle(plane_details,nav_details);
			break;
	case(LEFT):if(nav_details->alt>0 && plane_details->speed!=0)
			  {
			plane_details->bank_angle--;
			   bank_plane(plane_details->bank_angle,plane_details);
			temp_rot=(1091*quick_tan(plane_details->bank_angle))/(plane_details->speed*0.5925);
			   }
		   break;
	case(RIGHT):if(nav_details->alt>0 && plane_details->speed!=0)
			{
		   plane_details->bank_angle++;
		   bank_plane(plane_details->bank_angle,plane_details);
		   temp_rot=(1091*quick_tan(plane_details->bank_angle))/(plane_details->speed*0.5925);
			}
		   break;
	case(AILERON_LEFT):if(nav_details->alt>0 && plane_details->speed!=0)
			  {
			   plane_details->bank_angle--;
			   bank_plane(plane_details->bank_angle,plane_details);
			   }
		   break;
	case(AILERON_RIGHT):if(nav_details->alt>0 && plane_details->speed!=0)
			{
		   plane_details->bank_angle++;
		   bank_plane(plane_details->bank_angle,plane_details);
			}
		   break;

	case(RUDDER_RIGHT) : plane_details->rudder=min(plane_details->rudder+1,30);
						 if(plane_details->speed!=0)
						 temp_rot=(1091*quick_tan(plane_details->rudder))/(plane_details->speed*0.5925);
			  break;
	case(RUDDER_LEFT) :plane_details->rudder=max(plane_details->rudder-1,-25);
					   if(plane_details->speed!=0)
					   temp_rot=(1091*quick_tan(plane_details->rudder))/(plane_details->speed*0.5925);
			  break;
	case(FRONT_VIEW): view_angle=0;
					  strcpy(view_text,"");
			  break;
	case(LEFT_VIEW) : view_angle=90;
					  strcpy(view_text,"LEFT");
			  break;
	case(RIGHT_VIEW): view_angle=-90;
					  strcpy(view_text,"RIGHT");
			  break;
	case(REAR_VIEW):  view_angle=180;
					  strcpy(view_text,"REAR");
			  break;
	case(GEAR)     :  move_gear(plane_details);
			  break;
	case(BRAKES)   :  change_brakes(plane_details);
			  break;
	case(CHANGE_NAV1):change_nav(nav_details,NAV1);
			  break;
	case(CHANGE_NAV2):change_nav(nav_details,NAV2);
			  break;
	case(FLAPS_DOWN) :change_flaps(plane_details,DOWN);
			  break;
	case(FLAPS_UP)   :change_flaps(plane_details,UP);
			  break;
	case(CHANGE_OMI_100) :change_omi_radial(nav_details,100);
			  break;
	case(CHANGE_OMI_10) :change_omi_radial(nav_details,10);
			  break;
	case(CHANGE_OMI_1) :change_omi_radial(nav_details,1);
			  break;
	case(CUT_ENGINE)   :plane_details->thrust=0;
			  break;
	case(MENU)         :menu_bar(nav_details,plane_details,airport_list,no_of_airports);
			  break;
	  }
	 }
  plane_details->rot+=temp_rot/2;
  if(plane_details->rot==0)
	 plane_details->rot=0;
  if(plane_details->rot>=360)
	 plane_details->rot-=360;
  if(plane_details->rot<0)
	 plane_details->rot+=360;


  update_orientation(plane_details,view_angle);

  move_plane(plane_details,nav_details);

  if(nav_details->alt>0)
	 bank_plane(plane_details->bank_angle,plane_details);


	setactivepage(page&1);
	setviewport(0,0,639,349,1);

	update_instruments(nav_details,plane_details);

	setviewport(0,0,639,200,1);
	clearviewport();
	set_view_reference_point(nav_details->ew,plane_details->view_alt,nav_details->ns);
	set_perspective((nav_details->ew-5)*plane_details->view_x,(plane_details->view_alt-5)*plane_details->view_y,(nav_details->ns-5)*plane_details->view_z);
	transform_mat=trans_eye(plane_details);
	create_horizon(transform_mat,plane_details->rot,nav_details,view_angle);
	create_view(transform_mat,vertex_list,face_list,shape_list,extent_list,no_of_shapes,no_of_extents);
	setcolor(RED);

	outtextxy(305,10,view_text);

	free(transform_mat);
	setvisualpage(page&1);
	page++;
	check_for_crash(plane_details,nav_details);
  }
nosound();
page++;
}

void set_new_palette()
{
 int count,count2,i;

 for(i=0;i<16;i++)
 setpalette(i,20+i);
}

/*============================================================================
								CHECK_FOR_CRASH
==============================================================================
============================================================================*/
void check_for_crash(plane plane_details,nav nav_details)
{
 if((plane_details->gear==UP && nav_details->alt==0) ||(nav_details->alt<10 && plane_details->bank_angle>1)
	||(nav_details->alt==0 && plane_details->speed>400))
   {
	crash(320,100,15,1,1);
	while(!kbhit());    /*WAIT FOR KEYSTROKE*/
	initialize_nav(nav_details);
	initialize_plane(plane_details);
   }
}
/*============================================================================
								CRASH
==============================================================================
   Generates a random 'shattering' of the cockpit windscreen.
============================================================================*/
void crash(int x,int y,int no_of_branches,int x_dir,int y_dir)
{
  int i,r_number,x_rand,y_rand,x2,y2;

  setcolor(WHITE);

  for(i=0;i<no_of_branches;i++)
	{
	 if(no_of_branches==1)
	  {	x_rand=(rand()%90)+10;
		y_rand=(rand()%20)+10;
	  }
	 else
	 if(i<no_of_branches/2)
	  {	x_rand=(rand()%90)+10;
		y_rand=(rand()%30)+10;
	  }
	 else
	  {	y_rand=(rand()%50)+10;
		x_rand=(rand()%50)+10;
	  }
	 if(i>0)
	 {
	 x_dir=rand()%2;
	 y_dir=rand()%2;
	 }
	 if(x_dir==1)
		x2=x+x_rand;
	 else x2=x-x_rand;
	 if(y_dir==1)
		y2=y+y_rand;
	 else y2=y-y_rand;

	 line(x,y,x2,y2);
	 if(x2>639 || x2<0 || y2>200 || y2<0)
		 return;

	 r_number=(rand()%10);
	 if(r_number>=8)
		r_number=2;
	 else
		r_number=1;
	 crash(x2,y2,r_number,x_dir,y_dir);
   }

}
/*============================================================================
								UPDATE_ORIENTATION
==============================================================================
============================================================================*/
void update_orientation(plane plane_details,float view_angle)
{
  float up_down;

  /*UPDATE FOR DIVE ANGLE*/
  plane_details->dyn=round_error(sin(plane_details->dive_angle/D_TO_R));
  plane_details->dzn=round_error(-cos(plane_details->dive_angle/D_TO_R));

  up_down=round_error(cos(view_angle/D_TO_R));

  plane_details->view_y=round_error(sin(plane_details->dive_angle*up_down/D_TO_R));
  plane_details->view_z=round_error(-cos(plane_details->dive_angle*up_down/D_TO_R));

  /*UPDATE FOR BANK ANGLE*/
  plane_details->dxn=round_error(sin(plane_details->rot/D_TO_R));
  plane_details->dzn=round_error(-cos(plane_details->rot/D_TO_R));

  plane_details->view_x=round_error(sin((view_angle+plane_details->rot)/D_TO_R));
  plane_details->view_z=round_error(-cos((view_angle+plane_details->rot)/D_TO_R));
}

/*============================================================================
								SAVE_SCREEN
==============================================================================
============================================================================*/
void save_screen(void far *buf[4])
{
   unsigned size;
   int ystart=0, yend, yincr, block;

   yincr = (maxy+1) / 4;
   yend = yincr;
   size = imagesize(0, ystart, maxx, yend);
/* get byte size of image */

   for (block=0; block<=3; block++)
   {
	  if ((buf[block] = farmalloc(size)) == NULL)
	  {
		 closegraph();
		 printf("Error: not enough heap space in save_screen().\n");
		 exit(1);
	  }

	  getimage(0, ystart, maxx, yend, buf[block]);
	  ystart = yend + 1;
	  yend += yincr + 1;
   }
}

/*============================================================================
								RESTORE_SCREEN
==============================================================================
============================================================================*/
void restore_screen(void far *buf[4])
{
   int ystart=0, yend, yincr, block;

   yincr = (maxy+1) / 4;
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
						INITIALIZE_THRUST_REQUIRED
==============================================================================
============================================================================*/
void initialize_thrust_required(plane plane_details)
{
 L_D+=plane_details->fuel/100;
 MINDRAG+=plane_details->fuel/20000;
}

/*============================================================================
								ROUND_ERROR
==============================================================================
============================================================================*/
float round_error(float value)
{
 if(fabs(value)<=UNDERFLOW)
	return 0;
 else return (float)value;
}


/*============================================================================
							CREATE_HORIZON
==============================================================================
============================================================================*/
void create_horizon(matrix transform,float rot,nav nav_details,int view_angle)
{
 float hx1,hy1,hz1,hx2,hy2,hz2,hx3,hy3,hz3,hx4,hy4,hz4;
 int neg;

 hy1=hy2=0.0;
 hy3=hy4=nav_details->alt+250000;

 if(nav_details->ew<0)
	neg=-1;
 else
	neg=1;

 /*PROJECT THE HORIZON IN FRONT OF THE PLANE*/
 hx2=hx3=nav_details->ew+(250000*round_error(sin((rot+view_angle-45.0)/D_TO_R)));
 hx1=hx4=nav_details->ew+(250000*round_error(sin((rot+view_angle+45.0)/D_TO_R)));

 if(nav_details->ns<0)
	neg=-1;
 else
	neg=1;
 hz1=hz4=nav_details->ns*neg+(250000*-round_error(cos((rot+view_angle+45.0)/D_TO_R)));
 hz2=hz3=nav_details->ns*neg+(250000*-round_error(cos((rot+view_angle-45.0)/D_TO_R)));

 hz1=-hz1;
 hz2=-hz2;
 hz3=-hz3;
 hz4=-hz4;

 /*TRANSFORM TO WORLD COORDINATES*/
 transform_point(transform,&hx1,&hy1,&hz1);
 transform_point(transform,&hx2,&hy2,&hz2);
 transform_point(transform,&hx3,&hy3,&hz3);
 transform_point(transform,&hx4,&hy4,&hz4);


 hz1=-hz1;
 hz2=-hz2;
 hz3=-hz3;
 hz4=-hz4;

 hx1=hx1*2;
 hy1=hy1*2;
 hx2=hx2*2;
 hy2=hy2*2;
 hx3=hx3*2;
 hy3=hy3*2;
 hx4=hx4*2;
 hy4=hy4*2;

 /*PROJECT IMAGE OF SKY FROM 3_D TO 2_D*/
  sky[6]=(hx1/hz1)*639+370;
  sky[7]=(hy1/hz1)*350+175;
  sky[4]=(hx2/hz2)*639+370;
  sky[5]=(hy2/hz2)*350+175;

  sky[0]=(hx4/hz4)*639+370;
  sky[1]=(hy4/hz4)*350+175;
  sky[2]=(hx3/hz3)*639+370;
  sky[3]=(hy3/hz3)*350+175;

  draw_sky();
  setcolor(LIGHTBLUE);
  /*DRAW HORIZON LINE*/
  convert_3d_to_2d(0,&hx1,&hy1,hz1,&hx2,&hy2,hz2);
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
								LINE_LENGTH
==============================================================================
	Calculate the length of a line.
============================================================================*/
float line_length(float x1,float y1,float x2,float y2)
{
  return sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
}

/*============================================================================
								LOAD_BEACONS
==============================================================================
		Loads beacon coordinates from scenery file.
============================================================================*/
void load_beacons(nav nav_details,int no_of_beacons,FILE *infile)
{
 int count;
 long read_info;
 nav_details->no_of_beacons=no_of_beacons;

 for(count=0;count<no_of_beacons*2;count++)
   {
	fscanf(infile,"%I",&read_info);
	nav_details->beacon_coords[count]=read_info;
   }
}

/*============================================================================
								LOAD_AIRPORTS
==============================================================================
		Loads airport details into array of airport structures from scenery
file.
============================================================================*/
airport *load_airports(airport *airports,int no_of_airports,FILE *infile)
{
 long read_info;
 int count;

 for(count=0;count<no_of_airports;count++)
   {
	 airports[count]->airport_name=(char*)malloc(30);
	 fscanf(infile,"%s",airports[count]->airport_name);
	 fscanf(infile,"%I",&read_info);
	 airports[count]->ew_coord=read_info;
	 fscanf(infile,"%I",&read_info);
	 airports[count]->ns_coord=read_info;
	}
 return airports;
}

/*============================================================================
								LOAD_VERTS
==============================================================================
		Loads vertices dat into array of vertex structures from scenery file.
============================================================================*/
vertices *load_verts(vertices *vertex_list,int no_of_verts,FILE *infile)
{
 int read_info,count1,count2;
 float *floatptr;

 for(count1=0;count1<no_of_verts;count1++)
	{
	 floatptr=(float*)vertex_list[count1];
	 for(count2=0;count2<3;count2++)
	   {
		  fscanf(infile,"%d",&read_info);
		  *floatptr++=read_info;
	   }
	}
 return vertex_list;
}

/*============================================================================
								LOAD_EXTENTS
==============================================================================
		Loads extent data from scenery file into array of extent structures.
============================================================================*/
extent *load_extents(extent *extent_list,int no_of_extents,FILE *infile)
{
 int count1,count2;
 long read_info;
 float *floatptr;

 for(count1=0;count1<no_of_extents;count1++)
	{
	 floatptr=(float*)extent_list[count1];
	 for(count2=0;count2<10;count2++)
	   {
		  fscanf(infile,"%I",&read_info);
		  *floatptr++=read_info;
	   }
	}
 return extent_list;
}

/*============================================================================
								LOAD_FACES
==============================================================================
		Loads polygon data into array of polygon structures from scenery file.
============================================================================*/
facelist *load_faces(facelist *face_list,int no_of_faces,FILE *infile)
{
 int count1,count2,read_info;
 float *charptr;


 for(count1=0;count1<no_of_faces;count1++)
	{
	 charptr=(float*)face_list[count1];
	   for(count2=0;count2<34;count2++)
	   {
		fscanf(infile,"%d",&read_info);
		*charptr++=read_info;
	   }
   }
 return face_list;
}

/*============================================================================
								LOAD_SHAPES
==============================================================================
		Loads shape data from scenery file into array of shape structures.
============================================================================*/
shape *load_shapes(shape *shape_list,int no_of_shapes, FILE *infile)
{
  int count1,count2;
  long read_info;
  float *floatptr;
   for(count1=0;count1<no_of_shapes;count1++)
	 {
	  floatptr=(float*)shape_list[count1];
	  for(count2=0;count2<21;count2++)
	   {
		  fscanf(infile,"%I",&read_info);
		  *floatptr++=(float)read_info;
	   }
		update_transform(shape_list[count1]);

	 }
  return shape_list;
}



/*============================================================================
								LOAD_WORLD
==============================================================================
		Loads data pertaining to the data requirements for loading the
scenery file.
============================================================================*/
void load_world(int *no_of_verts,int *no_of_faces,int *no_of_shapes,int *no_of_extents,int *no_of_beacons,int *no_of_airports,FILE *infile)
{
 fscanf(infile,"%d",no_of_verts);
 fscanf(infile,"%d",no_of_faces);
 fscanf(infile,"%d",no_of_shapes);
 fscanf(infile,"%d",no_of_extents);
 fscanf(infile,"%d",no_of_beacons);
 fscanf(infile,"%d",no_of_airports);
}

/*============================================================================
								TRANS_EYE
==============================================================================
		Generates the transformation matrix for converting from world to eye
coordinate systems.
============================================================================*/
matrix trans_eye(plane plane_details)
{
 float rot;
 float rup,xup_vp,yup_vp;
 matrix transform;

 transform=mat3d();

 initialise_matrix(transform);

 transform->entry[0][0]=transform->entry[1][1]=transform->entry[2][2]=1;

 mtranslate_3d( transform
		   ,-(xr+plane_details->view_x*view_distance)
		   ,-(yr+plane_details->view_y*view_distance)
		   ,-(zr+plane_details->view_z*view_distance)
		  );

 rot=sqrt( plane_details->view_y*plane_details->view_y
	  +plane_details->view_z*plane_details->view_z);

 if (rot>0.0)
	 mrotx3d(transform,-plane_details->view_y/rot,-plane_details->view_z/rot);

 mroty3d(transform,plane_details->view_x,rot);

 xup_vp= plane_details->dxup*transform->entry[0][0]
	+plane_details->dyup*transform->entry[1][0]
	+plane_details->dzup*transform->entry[2][0];
 yup_vp= plane_details->dxup*transform->entry[0][1]
	+plane_details->dyup*transform->entry[1][1]
	+plane_details->dzup*transform->entry[2][1];

 rup=sqrt(xup_vp*xup_vp+yup_vp*yup_vp);
 mrotz3d(transform,xup_vp/rup,yup_vp/rup);
 return transform;
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
============================================================================*/
void scale(matrix transmat,float scale_x,float scale_y,float scale_z)
{
 transmat->entry[0][0]=transmat->entry[0][0]*scale_x;
 transmat->entry[1][1]=transmat->entry[1][1]*scale_y;
 transmat->entry[2][2]=transmat->entry[2][2]*scale_z;
}



/*============================================================================
								CREATE_VIEW
==============================================================================
============================================================================*/
void create_view(matrix transform,vertices *vertex_list, facelist *polygon_list,shape *shape_list,extent *extent_list,int no_of_shapes,int no_of_extents)
{
 int i,count,shape_count;
 float x,y,z,x2,y2,z2;
 float poly_array[100];
 int poly_int_array[100];
 float clipped_line[4];
 int clipcount=0;
 int poly_count;
 int line_status=0;
 int end_clipped;
 int plane_clipped=FALSE;
 int right[4]={639,0,639,200};
 int left[4]={0,0,0,200};
 int top[4]={0,0,639,0};
 int bottom[4]={0,200,639,200};
 transform_projection_point(transform);

 test_extents(extent_list,no_of_extents,transform);

 for(shape_count=0;shape_count<no_of_shapes;shape_count++)
 { if(extent_list[shape_list[shape_count]->extent_no]->clip!=CLIPPED)
 for(i=shape_list[shape_count]->start_poly;i<=shape_list[shape_count]->end_poly;i++)
	{
	 poly_count=0;
	 clipcount=1;
	 for(count=0;count<polygon_list[i]->no_of_verts-1;count+=1)
	 {
	  x=vertex_list[polygon_list[i]->verts[count]]->x;
	  y=vertex_list[polygon_list[i]->verts[count]]->y;
	  z=vertex_list[polygon_list[i]->verts[count]]->z;

	  x2=vertex_list[polygon_list[i]->verts[count+1]]->x;
	  y2=vertex_list[polygon_list[i]->verts[count+1]]->y;
	  z2=vertex_list[polygon_list[i]->verts[count+1]]->z;


	  transform_point(shape_list[shape_count]->shape_mat,&x,&y,&z);
	  transform_point(shape_list[shape_count]->shape_mat,&x2,&y2,&z2);

	  z=-z;
	  z2=-z2;

	  transform_point(transform,&x,&y,&z);
	  transform_point(transform,&x2,&y2,&z2);
	  if(!invisible(z,shape_list[shape_count]->detail_level))
	 {
	 if((z<=z2))/*&& dzn<0)||(z>z2 && dzn>0))*/
	   {
	line_status=clip(&x,&y,&z,x2,y2,z2);
	end_clipped=NEAR;
	   }

	 else
	   {
	line_status=clip(&x2,&y2,&z2,x,y,z);
	end_clipped=FAR;
	   }
	}
	else
	  break;

	  if(z!=-1000 && z2!=-1000)
	{
		x=x*2;
		y=y*2;
		x2=x2*2;
		y2=y2*2;
		clip_planes(&x,&y,&z,&x2,&y2,&z2,shape_list[shape_count]->color,shape_list[shape_count]->fill);

		if (line_status==CLIPPED)
		   {
		 if(end_clipped==NEAR)
		   {
			clipped_line[clipcount*2]=x;
			clipped_line[clipcount*2+1]=y;
			clipcount--;
		   }
		 else
		   {
			clipped_line[clipcount*2]=x2;
			clipped_line[clipcount*2+1]=y2;
			clipcount--;
		   }
		 line_status=NOT_CLIPPED;
		   }
		if(z!=-9999)
		   {
		poly_array[poly_count]=x;
		poly_array[poly_count+1]=y;
		poly_array[poly_count+2]=x2;
		poly_array[poly_count+3]=y2;

		poly_count+=4;
		   }

		if(clipcount==-1&&z!=-9999)
		  {
		poly_array[poly_count]=clipped_line[0];
		poly_array[poly_count+1]=clipped_line[1];
		poly_array[poly_count+2]=clipped_line[2];
		poly_array[poly_count+3]=clipped_line[3];

		poly_count+=4;
		clipcount=1;
		  }



/*	    convert_3d_to_2d(count,&x,&y,-z,&x2,&y2,-z2);*/

	}
	   }
	 poly_array[poly_count]=-9999;

	 if(shape_list[shape_count]->fill==SOLID && poly_array[0]!=-9999)
		{
		 setfillstyle(SOLID_FILL,shape_list[shape_count]->color);
		 polyclip(poly_array,&poly_count,right,RIGHT_CLIP);
		 polyclip(poly_array,&poly_count,left,LEFT_CLIP);
		 polyclip(poly_array,&poly_count,top,TOP_CLIP);
		 polyclip(poly_array,&poly_count,bottom,BOTTOM_CLIP);
		 if(poly_count>0)
		mypoly(poly_array,poly_count/2,shape_list[shape_count]->color,shape_count);
	   }
	}
  }
}

/*============================================================================
								INVISIBLE
==============================================================================
============================================================================*/
int invisible(float z,float range)
{
 switch((int)range){
			   case 1: if(z>8000) return TRUE;
					   else return FALSE;
			   case 2: if(z>20000) return TRUE;
					   else return FALSE;
			   case 3: if(z>50000) return TRUE;
					   else return FALSE;
			   case 4: if(z>100000) return TRUE;
					   else return FALSE;
			   default: return FALSE;
			  }
}


/*============================================================================
								TEST_EXTENTS
==============================================================================
============================================================================*/
void test_extents(extent *extent_list,int no_of_extents,matrix transform)
{
 int extent_count,line_count;
 int clip_count=0;
 float x,y,z,x2,y2,z2;

 for(extent_count=0;extent_count<no_of_extents;extent_count++)
  {
	for(line_count=0;line_count<4;line_count++)
	   {
	x=extent_list[extent_count]->coord[line_count][0];
	z=extent_list[extent_count]->coord[line_count][1];
	x2=extent_list[extent_count]->coord[line_count+1][0];
	z2=extent_list[extent_count]->coord[line_count+1][1];

	y=y2=0.0;

	  z=-z;
	  z2=-z2;

	  transform_point(transform,&x,&y,&z);
	  transform_point(transform,&x2,&y2,&z2);


	 if((z<=z2))/*&& dzn<0)||(z>z2 && dzn>0))*/
	   {
	clip(&x,&y,&z,x2,y2,z2);
	   }
	 else
	   {
	clip(&x2,&y2,&z2,x,y,z);
	   }

	  if(z==-1000 || z2==-1000)
	 clip_count++;
	  else
	{
		x=x*2;
		y=y*2;
		x2=x2*2;
		y2=y2*2;
		clip_planes(&x,&y,&z,&x2,&y2,&z2,0,EXTENT_FILL);
		if(z==-9999)
		   clip_count++;
	}

	}

  if(clip_count==4)
	 extent_list[extent_count]->clip=CLIPPED;
  else
	 extent_list[extent_count]->clip=NOT_CLIPPED;
  clip_count=0;
 }
}
/*============================================================================
								TRANSFORM_POINT
==============================================================================
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
								CLIP
==============================================================================
============================================================================*/
int clip(float *x,float *y,float *z,float x2,float y2,float z2)
{
 float z_change;

 if ((*z<=front_z && z2>front_z)||(*z>=front_z && z2<front_z))
   {
	 z_change=(front_z-*z)/(*z-z2);
	 *x=(*x-x2)*(z_change)+*x;
	 *y=(*y-y2)*(z_change)+*y;

	 if (z2>=front_z) *z=front_z;
	 return CLIPPED;
   }
 else
   if ((*z<=front_z && z2<=front_z))
	*z=-1000;
 return 0;

}

/*============================================================================
								TEST
==============================================================================
============================================================================*/
int test(float x,float y, float z)
{
 return(2*x>=z || 2*x<=-z || y>=z || y<=-z);
}

/*============================================================================
								CONVERT_3D_TO_2D
==============================================================================
============================================================================*/
void convert_3d_to_2d(int count,float *x,float *y,float z,float *x2,float *y2, float z2)
{

  *x=/*((x*zc-xc*z)/(zc-z))*/(((*x))/(z))*639+370;
  *y=/*((y*zc-yc*z)/(zc-z))*/(((*y))/(z))*350+175;


  *x2=/*((x2*zc-xc*z2)/(zc-z2))*/(((*x2))/(z2))*639+370;
  *y2=/*((y2*zc-yc*z2)/(zc-z2))*/(((*y2))/(z2))*350+175;

  if((*x<0 && *x2<0)||(*x>639 && *x2>639)||(*y<0 && *y2<0)||(*y>200 && *y2>200))
	 return;
  line(*x,*y,*x2,*y2);
}

/*============================================================================
						SET_VIEW_REFERENCE_POINT
==============================================================================
============================================================================*/
void set_view_reference_point(float x,float y,float z)
{
  xr=x;
  yr=y;
  zr=z;
}

/*============================================================================
							SET_PERSPECTIVE
==============================================================================
============================================================================*/
void set_perspective(float x,float y, float z)
{
  xpcntr=x;
  ypcntr=y;
  zpcntr=z;
}

/*============================================================================
						TRANSFORM_PROJECTION_POINT
==============================================================================
============================================================================*/
void transform_projection_point(matrix transform)
{
 xc=xpcntr;
 yc=ypcntr;
 zc=zpcntr;
 transform_point(transform,&xc,&yc,&zc);
}

/*============================================================================
								BANK_PLANE
==============================================================================
============================================================================*/
void bank_plane(float bank_angle,plane plane_details)
{

 if(bank_angle<0)
 bank_angle=bank_angle;

 set_view_up( quick_sin(bank_angle)*plane_details->dzn
		 ,quick_cos(bank_angle)
		  *sqrt( plane_details->dxn*plane_details->dxn
			+plane_details->dzn*plane_details->dzn),
		 quick_sin(bank_angle)*plane_details->dxn,
		 plane_details
		);
 if(plane_details->bank_angle>=360)
	plane_details->bank_angle=0;
 else
 if(plane_details->bank_angle<=-360)
	plane_details->bank_angle=0;
 update_artificial_horizon(plane_details->bank_angle);
}


/*============================================================================
								SET_VIEW_UP
==============================================================================
============================================================================*/
void set_view_up(float x,float y,float z,plane plane_details)
{
 plane_details->dxup=x;
 plane_details->dyup=y;
 plane_details->dzup=z;
}

/*============================================================================
								DRAW_INSTRUMENTS
==============================================================================
============================================================================*/
void draw_instruments()
{
  displaymap("test.bmp",0,200);
}

/*============================================================================
								DRAW_SKY
==============================================================================
============================================================================*/
void draw_sky()
{
 setfillstyle(SOLID_FILL,BLUE);
 setcolor(BLUE);
 fillpoly(4,sky);
 setcolor(WHITE);
};

/*============================================================================
								MOVE_PLANE
==============================================================================
============================================================================*/
void move_plane(plane plane_details,nav nav_details)
{
  int actual_speed;
  static int alt_speed;

  update_fuel_flow(plane_details);
  update_lift(plane_details,nav_details);
  update_dive_angle(plane_details,nav_details);
  update_speed(plane_details,nav_details);


  actual_speed=plane_details->speed*0.5792;

  nav_details->ew=nav_details->ew+actual_speed*-plane_details->dxn;
  nav_details->ns=nav_details->ns+actual_speed*plane_details->dzn;
  nav_details->alt=nav_details->alt+actual_speed*plane_details->dyn;

  /*CHECK IF EXTREMETIES OF WORLD HAVE BEEN BREACHED*/
  if(nav_details->ew>700000) nav_details->ew=10;
  if(nav_details->ew<10) nav_details->ew=700000;
  if(nav_details->ns<-700000) nav_details->ns=-10;
  if(nav_details->ns>-10) nav_details->ns=-700000;


  if(nav_details->alt<0)
	{
	 nav_details->alt=0;
	 plane_details->dive_angle=0;
	}
 plane_details->view_alt=nav_details->alt+10;
}

/*============================================================================
								UPDATE_SPEED
==============================================================================
============================================================================*/
void update_speed(plane plane_details,nav nav_details)
{
  float drag=0;
  float available_thrust;
  float plane_downforce;
  float gravity_drag;
  float total_weight;
  float flap_drag;
  float gear_drag,config_drag;
  float drag_param1,drag_param2;
  float actual_speed=plane_details->speed*0.5925;
  float weight_drag;

  weight_drag=plane_details->fuel/10000;
  flap_drag=(plane_details->flaps-1)*0.06;

  if(plane_details->gear==DOWN)
	 gear_drag=0.1;
  else
	 gear_drag=0;

  config_drag=gear_drag+flap_drag+weight_drag;

  total_weight=PLANE_WEIGHT+plane_details->fuel;

  plane_downforce=(total_weight-plane_details->lift);


  if(plane_details->speed!=0 && nav_details->alt>0)
	 {
	  drag_param1=(L_D/(-actual_speed))+config_drag;
	  drag_param2=(-actual_speed/(L_D))+config_drag;

/*	drag=500*(MINDRAG*(((((L_D/(plane_details->speed*0.5925))-config_drag)*((L_D/(plane_details->speed*0.5925))-config_drag))
			   +((((plane_details->speed*0.5925)/L_D)-config_drag)*(((plane_details->speed*0.5925)/L_D)-config_drag)))));*/

	  drag=500*(MINDRAG*((drag_param1*drag_param1)+(drag_param2*drag_param2)));

	  drag=drag+((1/quick_cos(plane_details->bank_angle/D_TO_R))
		  *(1/quick_cos(plane_details->bank_angle/D_TO_R)-1)*drag);
	 }

  drag=drag+plane_details->aoa_drag;
  if(plane_details->speed!=0 && nav_details->alt==0)
	 drag=P_DRAG*(plane_details->speed*0.5925/L_D)*(plane_details->speed*0.5925/L_D);
  if(nav_details->alt==0.0)
	   drag=drag+(total_weight/GRAVITY)*0.8+fabs(plane_details->acceleration)*(GRAVITY/total_weight)+BRAKE_DRAG*plane_details->brakes;
	/* drag=0.8*(plane_downforce/GRAVITY)*(plane_details->acceleration);*/


  if(drag>10000)
	 drag=10000;

  plane_details->available_thrust=plane_details->thrust-((total_weight*-plane_details->dyn)+drag);

  drag=drag+plane_details->aoa_drag;
  plane_details->drag=drag;

  gravity_drag=(GRAVITY*plane_details->dyn);


  plane_details->acceleration=((plane_details->available_thrust*GRAVITY)/(total_weight))+gravity_drag;


  plane_details->speed+=-plane_details->acceleration/2;

   if(plane_details->speed>0)
	 plane_details->speed=0;

   sound(plane_details->speed/3);
}

/*============================================================================
								UPDATE_LIFT
==============================================================================
============================================================================*/
void update_lift(plane plane_details,nav nav_details)
{
  float lift_coeff;

  lift_coeff=-plane_details->dive_angle/10;

  if(lift_coeff>0.0)
  plane_details->lift=lift_coeff*WING_AREA*((plane_details->speed*0.5925*plane_details->speed*0.5925)/295);
}

/*============================================================================
								UPDATE_DIVE_ANGLE
==============================================================================
============================================================================*/
void update_dive_angle(plane plane_details,nav nav_details)
{
 float dive_angle;
 float plane_angle;
 float total_weight;

 total_weight=plane_details->fuel+PLANE_WEIGHT;

 plane_angle=(plane_details->thrust-plane_details->drag)/total_weight;

 if(plane_angle>=-1 && plane_angle<=1)
 {
 dive_angle=round_error(asin(plane_angle));
 if(-dive_angle>plane_details->dive_angle);
  {
	plane_details->aoa_drag=plane_details->drag-(plane_details->thrust-(total_weight*plane_angle));
	if(-plane_details->speed<sqrt(295*total_weight/(1.3*WING_AREA))+20 && nav_details->alt>0)
		  plane_details->dive_angle=-dive_angle;
  }
 }
 if(nav_details->alt==0.0 && plane_details->dive_angle>0)
	plane_details->dive_angle=0;
}

/*============================================================================
								UPDATE_WEIGHT_CHANGES
==============================================================================
============================================================================*/
void update_weight_changes(plane plane_details,float weight_change)
{
 L_D-=weight_change/100;

 MINDRAG-=weight_change/10000;
}


/*============================================================================
								NEW_SCANLINE
==============================================================================
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
============================================================================*/
intersect new_intersect()
{
  intersect temp;

  temp=(intersect)malloc(sizeof(INTERSECT));

  temp->next_intersect=NULL;

  return temp;
}



/*============================================================================
								UPDATE_TRANSFORM
==============================================================================
============================================================================*/
void update_transform(shape shape1)
{
 initialise_matrix(shape1->shape_mat);
 shape1->shape_mat->entry[0][0]=shape1->shape_mat->entry[1][1]=shape1->shape_mat->entry[2][2]=1;
  scale(shape1->shape_mat,shape1->scale_x,shape1->scale_y,shape1->scale_z);
/*  mrotz3d(shape1->shape_mat,sin(shape1->curr_y_angle/D_TO_R),cos(shape1->curr_y_angle/D_TO_R));*/
  if(shape1->curr_y_angle!=0)
	 mroty3d(shape1->shape_mat,sin(shape1->curr_y_angle/D_TO_R),cos(shape1->curr_y_angle/D_TO_R));
  mtranslate_3d(shape1->shape_mat,shape1->wx,shape1->wy,shape1->wz);
}

/*============================================================================
								INITIALIZE_ENVIRONMENT
==============================================================================
============================================================================*/
void initialize_environment(environment environ)
{
 environ->season=WINTER;
}
/*============================================================================
								INITIALIZE_PLANE
==============================================================================
============================================================================*/
void initialize_plane(plane plane_details)
{
   plane_details->dxn=plane_details->dyn=0.0;
   plane_details->dzn=-1;
   plane_details->view_x=plane_details->view_y=0.0;
   plane_details->view_z=-1;
   plane_details->dxup=plane_details->dzup=0.0;
   plane_details->dyup=1.0;
   plane_details->dive_angle=0.0;
   plane_details->rot=180.0;
   plane_details->speed=0;
   plane_details->bank_angle=0.0;
   plane_details->thrust=0.0;
   plane_details->acceleration=0.0;
   plane_details->lift=0.0;
   plane_details->view_alt=10;
   plane_details->gear=DOWN;
   plane_details->flaps=1;
   plane_details->aoa_drag=0.0;
   plane_details->fuel=4000;
   plane_details->rudder=0;
   plane_details->drag=0;
   plane_details->available_thrust=0;
   plane_details->brakes=OFF;
}

/*============================================================================
								INITIALIZE_NAV
==============================================================================
============================================================================*/
void initialize_nav(nav nav_details)
{
  nav_details->ns=-150200.0;
  nav_details->alt=0.0;
  nav_details->ew=200100.0;
  nav_details->nav1=0;
  nav_details->nav2=1;
  nav_details->omi_radial=180;
}

/*============================================================================
								CHANGE_BRAKES
==============================================================================
============================================================================*/
void change_brakes(plane plane_details)
{
 if(plane_details->brakes==ON)
	plane_details->brakes=OFF;
 else
	plane_details->brakes=ON;
}

/*============================================================================
								MOVE_GEAR
==============================================================================
============================================================================*/
void move_gear(plane plane_details)
{
 if(plane_details->gear==UP)
	{
	 plane_details->gear=DOWN;
	 MINDRAG+=1;
	 L_D-=25;
	}
 else
	{
	 plane_details->gear=UP;
	 MINDRAG-=1;
	 L_D+=25;
	}
}

/*============================================================================
								CHANGE_NAV
==============================================================================
============================================================================*/
void change_nav(nav nav_details,int nav_num)
{
 if(nav_num==NAV1)
	nav_details->nav1++;
 else
	nav_details->nav2++;
 if(nav_details->nav1>=nav_details->no_of_beacons)
	nav_details->nav1=0;
 if(nav_details->nav2>=nav_details->no_of_beacons)
	nav_details->nav2=0;

}

/*============================================================================
								CHANGE_FLAPS
==============================================================================
============================================================================*/
void change_flaps(plane plane_details,int direction)
{
 switch(direction)
	 {
	  case UP  :  plane_details->flaps++;
				  if(plane_details->flaps>4)
					 plane_details->flaps=4;
				  else
					{ L_D-=10;
					  MINDRAG+=0.3;
					  P_DRAG+=300;
					}
				  break;
	  case DOWN:  plane_details->flaps--;
				  if(plane_details->flaps<1)
					 plane_details->flaps=1;
				  else
					{ L_D+=10;
					  MINDRAG-=0.3;
					  P_DRAG-=300;
					}
				  break;
	 }
}
/*============================================================================
							CHANGE_OMI_RADIAL
==============================================================================
============================================================================*/
void change_omi_radial(nav nav_details,int change_rate)
{
 nav_details->omi_radial=fmod(nav_details->omi_radial+change_rate,360);
}

/*============================================================================
								UPDATE_FUEL_FLOW
==============================================================================
============================================================================*/
void update_fuel_flow(plane plane_details)
{
  float fuel_used;
  fuel_used=plane_details->thrust/7200;
  plane_details->fuel-=fuel_used;
  if(plane_details->fuel<0)
	{
	 plane_details->fuel=0;
	 plane_details->thrust=0;
	}
  update_weight_changes(plane_details,fuel_used);
}


/*============================================================================
								UPDATE_INSTRUMENTS
==============================================================================
============================================================================*/
void update_instruments(nav nav_details,plane plane_details)
{
 update_altimeter(nav_details->alt);
 update_artificial_horizon(plane_details->bank_angle);
 update_bank_recorder(plane_details->bank_angle);
 update_speedometer(plane_details->speed);
 update_compass(plane_details->rot);
 update_rpm(plane_details->thrust);
 update_climb_angle(plane_details->dive_angle);
 update_climb_recorder(plane_details->dive_angle);
 update_climb_rate(plane_details->speed,plane_details->dyn);
 update_navigation(nav_details);
 update_flaps(plane_details->flaps);
 update_time();
 update_gear(plane_details->gear);
 update_dme(nav_details);
 update_fuel(plane_details->fuel);
 update_brake(plane_details->brakes);
}

/*============================================================================
								UPDATE_BRAKES
==============================================================================
============================================================================*/
void update_brake(int brake)
{
 if(brake==ON)
	{
	  setcolor(RED);
	  setfillstyle(SOLID_FILL,RED);
	  pieslice(607,312,0,360,2);
	 }
 else
   {
	 setcolor(LIGHTGREEN);
	 setfillstyle(SOLID_FILL,LIGHTGREEN);
	 pieslice(607,312,0,360,2);
   }
}


/*============================================================================
								UPDATE_GEAR
==============================================================================
============================================================================*/
void update_gear(int gear)
{

 if(gear==DOWN)
   {
	 setcolor(RED);
	 setfillstyle(SOLID_FILL,RED);
	 pieslice(607,292,0,360,2);
   }
 else
   {
	 setcolor(LIGHTGREEN);
	 setfillstyle(SOLID_FILL,LIGHTGREEN);
	 pieslice(607,292,0,360,2);
   }
}

/*============================================================================
								UPDATE_ALTIMETER
==============================================================================
============================================================================*/
void update_altimeter(float curr_alt)
{
 float angle,x,y;

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);

 pieslice(189,242,0,360,26);


 setcolor(RED);


 angle=36*(curr_alt/100);

 angle_to_coord(&x,&y,angle);
 x=x*25;
 y=y*-25;

 setcolor(WHITE);
 line(189,242,189+x,242+y);

 angle=36*(curr_alt/1000);
 angle_to_coord(&x,&y,angle);
 x=x*15;
 y=y*-15;
 line(189,242,189+x,242+y);
}

/*============================================================================
								UPDATE_FUEL
==============================================================================
============================================================================*/
void update_fuel(float fuel)
{
 float fuel_percentage;

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);

 bar(482,293,567,296);

 setcolor(RED);
 setfillstyle(SOLID_FILL,RED);
 fuel_percentage=(fuel/MAXFUEL)*85;

 bar(482,293,482+fuel_percentage,296);
}


/*============================================================================
								UPDATE_FLAPS
==============================================================================
============================================================================*/
void update_flaps(int flap_pos)
{
 setcolor(LIGHTGREEN);

 if(flap_pos==4)
	setcolor(WHITE);
 line(620,268,620,258);
 setcolor(LIGHTGREEN);
 if(flap_pos==3)
	setcolor(WHITE);
 line(620,268,614,260);
 setcolor(LIGHTGREEN);
 if(flap_pos==2)
	setcolor(WHITE);
 line(620,268,607,263);
 setcolor(LIGHTGREEN);
 if(flap_pos==1)
	setcolor(WHITE);
 line(620,268,605,268);
}

/*============================================================================
								UPDATE_CLIMB_ANGLE
==============================================================================
============================================================================*/
void update_climb_angle(float angle)
{
 float y_coord;

  setfillstyle(SOLID_FILL,LIGHTGREEN);
  setcolor(LIGHTGREEN);

  bar(68,287,110,337);

  if(angle<-13) angle=-13;
  if(angle>13) angle=13;


  y_coord=311+angle*2;

  setcolor(WHITE);
  line(69,y_coord,109,y_coord);
}

/*============================================================================
								UPDATE_CLIMB_RATE
==============================================================================
============================================================================*/
void update_climb_rate(float speed,float climb_vector)
{
 float angle,x,y;

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);

 pieslice(289,240,0,360,21);

 angle=45*((speed*0.5925)*climb_vector*0.1)-90;

 if(angle>180) angle=180;
 if(angle<-180) angle=-180;

 angle_to_coord(&x,&y,angle);
 x=x*20;
 y=y*-20;

 setcolor(WHITE);
 line(289,240,289+x,240+y);
}
/*============================================================================
								UPDATE_DME
==============================================================================
============================================================================*/
void update_dme(nav nav_details)
{
 float distance;
 char *dme_text=(char *)malloc(30*sizeof(char));

 setfillstyle(SOLID_FILL,LIGHTGREEN);
 setcolor(LIGHTGREEN);

 bar(584,225,615,237);

 distance=line_length(nav_details->beacon_coords[nav_details->nav1*2],nav_details->beacon_coords[nav_details->nav1*2+1],nav_details->ew,nav_details->ns);
 distance=distance/5292;
 setcolor(RED);
 sprintf(dme_text,"%d",(int)distance);
 outtextxy(586,227,dme_text);

 free(dme_text);
}

/*============================================================================
								UPDATE_NAVIGATION
==============================================================================
============================================================================*/
void update_navigation(nav nav_details)
{
 float angle;
 char *nav_text=(char*)malloc(30*sizeof(char));

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);
 bar(475,225,507,237);
 bar(475,247,507,259);
 bar(530,224,537,232);
 bar(531,249,537,256);

 angle=coord_to_angle(nav_details->ew,-nav_details->ns,nav_details->beacon_coords[nav_details->nav1*2],nav_details->beacon_coords[nav_details->nav1*2+1]);

 update_omi(angle,nav_details);

 setcolor(RED);
 sprintf(nav_text,"%d",(int)angle);
 outtextxy(477,227,nav_text);

 angle=coord_to_angle(nav_details->ew,-nav_details->ns,nav_details->beacon_coords[nav_details->nav2*2],nav_details->beacon_coords[nav_details->nav2*2+1]);
 sprintf(nav_text,"%d",(int)angle);
 outtextxy(477,250,nav_text);

 sprintf(nav_text,"%d",(int)nav_details->nav1+1);
 outtextxy(530,225,nav_text);
 sprintf(nav_text,"%d",(int)nav_details->nav2+1);
 outtextxy(531,250,nav_text);

 free(nav_text);
}

/*============================================================================
								UPDATE_OMI
==============================================================================
============================================================================*/
void update_omi(float angle,nav nav_details)
{
 char *radial_text=(char*)malloc(30*sizeof(char));

 bar(368,288,398,294);
 bar(368,331,397,337);
 bar(355,300,409,325);

 angle=(nav_details->omi_radial-angle)*25;
 if(angle>25) angle=25;
 if(angle<-25) angle=-25;
 setcolor(BLUE);
 line(383+angle,300,383+angle,325);

 setfillstyle(SOLID_FILL,WHITE);
 setcolor(WHITE);

 bar(382,312,384,314);

 setcolor(RED);
 sprintf(radial_text,"%d",(int)nav_details->omi_radial);
 outtextxy(370,288,radial_text);

 setcolor(BLUE);
 sprintf(radial_text,"%d",((int)nav_details->omi_radial+180)%360);
 outtextxy(370,331,radial_text);

 free(radial_text);

}

/*============================================================================
							UPDATE_BANK_RECORDER
==============================================================================
============================================================================*/
void update_bank_recorder(float bank_angle)
{
  int xval;

  setcolor(LIGHTGREEN);
  setfillstyle(SOLID_FILL,LIGHTGREEN);
  bar(355,205,410,207);
  bar(355,270,410,272);

  xval=bank_angle;
  if(xval>27) xval=27;
  if(xval<-27) xval=-27;
  setcolor(WHITE);
  line(382+xval,205,382+xval,207);
  line(382+xval,272,382+xval,270);
}


/*============================================================================
						UPDATE_CLIMB_RECORDER
==============================================================================
============================================================================*/
void update_climb_recorder(float angle)
{
  int yval;

  setcolor(LIGHTGREEN);
  setfillstyle(SOLID_FILL,LIGHTGREEN);
  bar(379,216,381,261);

  if(angle>23) angle=23;
  if(angle<-23) angle=-23;
  yval=angle+238;

  setcolor(WHITE);

  line(379,yval,381,yval);
}



/*============================================================================
							UPDATE_ARTIFICIAL_HORIZON
==============================================================================
============================================================================*/
void update_artificial_horizon(float bank_angle)
{
 float angle;

 setcolor(BLUE);
 setfillstyle(SOLID_FILL,BLUE);

 pieslice(190,312,0,360,30);

 angle=bank_angle+180;


 setfillstyle(SOLID_FILL,BLACK);
 setcolor(BLACK);

 if(bank_angle>=0.0)
   if(bank_angle<180)
   { pieslice(190,312,angle,360,30);
     pieslice(190,312,0,angle-180,30);
	 }
   else
   {
	pieslice(190,312,angle,angle-180,30);
   }

 else
  if(bank_angle<-180)
   {
	pieslice(190,312,0,360+bank_angle,30);
	pieslice(190,312,360+angle,360,30);
   }
   else
	pieslice(190,312,angle,angle+180,30);

  setcolor(WHITE);
  line(179,312,201,312);
}


/*============================================================================
2								UPDATE_RPM
==============================================================================
============================================================================*/
void update_rpm(float rpm)
{
 float rpm_percentage;

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);
 bar(484,325,560,333);

 rpm_percentage=(rpm/MAXRPM)*75;
 setcolor(RED);
 setfillstyle(SOLID_FILL,RED);
 bar(485,328,485+rpm_percentage,332);
}
/*============================================================================
								UPDATE_TIME
==============================================================================
============================================================================*/

void update_time()
{
 struct time t;
 char *time_text=(char*)malloc(30*sizeof(char));

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);
 bar(475,270,540,282);

 gettime(&t);

 setcolor(RED);
 sprintf(time_text,"%2d:%02d:%02d",t.ti_hour, t.ti_min, t.ti_sec);
 outtextxy(475,273,time_text);
 free(time_text);
}
/*============================================================================
								 UPDATE_SPEEDOMETER
==============================================================================
============================================================================*/
void update_speedometer(float speed)
{
 float angle,x,y;

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);

 pieslice(89,240,0,360,16);

 angle=0.6*((-speed)*0.5925);

 if(angle>359) angle=359;

 angle_to_coord(&x,&y,angle);
 x=x*15;
 y=y*-15;

 setcolor(WHITE);
 line(89,240,89+x,240+y);
}

/*============================================================================
								UPDATE_COMPASS
==============================================================================
============================================================================*/
void update_compass(float direction)
{
 char *direction_text=(char*)malloc(30*sizeof(char));

 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGREEN);
 bar(280,290,305,300);

 sprintf(direction_text,"%d",(int)direction);
 setcolor(RED);
 outtextxy(280,290,direction_text);

 free(direction_text);
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
							ANGLE_TO_COORD
==============================================================================
============================================================================*/
void angle_to_coord(float *x,float *y,float angle)
{
 float cos_angle,sin_angle;

 cos_angle=round_error(cos(angle/D_TO_R));
 sin_angle=round_error(sin(angle/D_TO_R));

 if(cos_angle>1 || cos_angle<-1)
	cos_angle=0;

 *x=(sin_angle);
 *y=(cos_angle)*0.67;
}

/*============================================================================
								COORD_TO_ANGLE
==============================================================================
============================================================================*/
float coord_to_angle(float x1,float y1,float x2,float y2)
{
  float slope;
  float angle;

  slope=get_slope(x1,y1,x2,y2);

  if(slope<0)
	 {
	  angle=atan(slope)*D_TO_R;
	  if(x1<x2) return (270-angle);
	  else      return (90-angle);
	 }
  if(slope>0)
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
							   GET_SLOPE
==============================================================================
============================================================================*/
float get_slope(float x1,float y1,float x2,float y2)
{
  if(x2-x1!=0)
	 return((y2-y1)/(x2-x1));
  else
	 return 0;
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


 for(i=0;i<no_of_verts*2;i+=2)
    {
      minx=min(minx,vert[i]);
	  maxx=max(maxx,vert[i]);
      miny=min(miny,vert[i+1]);
	  maxy=max(maxy,vert[i+1]);
	}

 if(maxy-miny<1)
   {
	 free(scanlist);
	 return;
   }
 for(i=miny;i<maxy;i++)
    {
     scanlist->scanline_no=i;
     scanlist->next_scan=new_scanline();
	 scanlist=scanlist->next_scan;
	}
 scanlist->scanline_no=i;
 scanlist=head;

 no_of_edges=no_of_verts/2;

 for(edge=0;edge<no_of_edges;edge++)
    {
     edge_vert=edge<<2;

     if(vert[edge_vert+3]-vert[edge_vert+1]==0)
	slope[edge]=-99;
     else
	 if(vert[edge_vert+2]-vert[edge_vert]==0)
	slope[edge]=0;
	 else
	  slope[edge]=1.0/((float)(vert[edge_vert+3]-vert[edge_vert+1])/(float)(vert[edge_vert+2]-vert[edge_vert]));

	 yval[edge][0]=min(vert[edge_vert+3],vert[edge_vert+1]);
	 yval[edge][1]=max(vert[edge_vert+3],vert[edge_vert+1]);


     if(vert[edge_vert+3]==yval[edge][0])
	 xval[edge]=vert[edge_vert+2];
     else
	 xval[edge]=vert[edge_vert];

	}

 for(i=0;i<no_of_edges;i++)
   {
	if(slope[i]!=-99)
	  {
       while(scanlist->scanline_no<(int)yval[i][0])
	     scanlist=scanlist->next_scan;
	   current_x=xval[i];
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
============================================================================*/
void draw_scanlines(scanline scanlist,int color)
{
  int x,x2,y;
  scanline lastscan,scanhead;
  intersect first,second,intersect_head;
  setcolor(color);

  scanhead=scanlist;

  while(scanlist!=NULL)
	 {
	  lastscan=scanlist;
	  intersect_head=scanlist->x_value;
      while(scanlist->x_value!=NULL)
	  {
	   first=scanlist->x_value;
	   second=scanlist->x_value->next_intersect;
	   x=scanlist->x_value->x_coord;
	   if(second==NULL)
	      x2=x;
	   else
		  x2=scanlist->x_value->next_intersect->x_coord;
	   y=scanlist->scanline_no;
	   line(x,y,x2,y);
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
============================================================================*/
void add_xval(scanline scanlist, float current_x)
{
  intersect temp,head,last;

  temp=new_intersect();

  temp->x_coord=current_x;

  if(scanlist->x_value==NULL)
	 scanlist->x_value=temp;
  else
   {
    head=scanlist->x_value;
    while(scanlist->x_value->next_intersect->x_coord<=current_x
	  && scanlist->x_value->next_intersect!=NULL)
	  scanlist->x_value=scanlist->x_value->next_intersect;
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

   if(inside(x1,y1,boundary_type,boundary))
       if(inside(x2,y2,boundary_type,boundary))
	  {
	   outverts[outcount]=x1;
	   outverts[outcount+1]=y1;
	   outverts[outcount+2]=x2;
	   outverts[outcount+3]=y2;
	   outcount+=4;
	  }
       else
	  {
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

void mapmask(char plane)
{
  outp(0x3c4,2);
  outp(0x3c5,plane);
}

void setres(char value)
{
 outp(0x3ce,0);
 outp(0x3cf,value);
}

void esetres(char mask)
{
 outp(0x3ce,1);
 outp(0x3cf,mask);
}

void hline(int page,int x1,int x2,int y,int color)
{
 int i,x,x1byte,x2byte,nbyte,address,dx,offset=80;
 char mask1,mask2;

 esetres(0x0f);
 setres(color);

   if(x1>x2)
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

  if(dx == 0)
	 hsub1(address,mask1&mask2);
  if(dx==1)
     hsub2(address,mask1,mask2);
  if(dx>1)
	 hsub3(address,mask1,mask2,nbyte);

  bitmask(255);
  esetres(0);
/*  mapmask(15);*/
}

void bitmask(char val)
{
 outp(0x3ce,8);
 outp(0x3cf,val);
 }



























