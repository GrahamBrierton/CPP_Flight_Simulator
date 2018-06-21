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
#include "geo.h"
#include "graph3d.h"
#include "instru.h"
#include "f_sim.h"


FILE *infile;
float xr=0.0,yr=0.0,zr=0.0;
float xc=0.0,yc=0.0,zc=0.0;
float xpcntr=0.0,ypcntr=0.0,zpcntr=0.0;
float view_distance=0.0;
int sky[]={0,0,639,0,639,160,0,160};
float MINDRAG=1.7;
float L_D=200;
float P_DRAG=400;
int note=0;




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
  float temp_rot=0.0;
  int page=0;
  int read_key;
  int no_of_verts,no_of_faces,no_of_shapes,no_of_extents,no_of_beacons,no_of_airports;
  char *temp_str=(char*)(malloc(80));
  struct viewporttype info;
  int view_angle=0;
  int up_down;

  /*ALLOCATE THE MEMORY FOR THE PLANE DETAILS*/
  plane plane_details=allocate_plane_mem();
  nav nav_details=allocate_nav_mem();
  environment environ_stats=allocate_environment_mem();


  strcpy(view_text,"");

  initialize_plane(plane_details);
  initialize_nav(nav_details);
  initialize_thrust_required(plane_details);


  /*SETUP THE TRIGONOMETRY TABLES*/
  set_sin_table();
  set_cos_table();
  set_tan_table();

  /*INITIALIZE THE GRAPHICS*/
  mode=VGAMED;
  initgraph(&driver,&mode,"g:\\c\\bc3.0\\bgi");


  /*DRAW THE INSTRUMENT PANEL*/
  setpalette(LIGHTGREEN,BLACK);
  setactivepage(1);
  draw_instruments();
/*  save_screen(ptr);       */
  setactivepage(0);
/*  restore_screen(ptr);*/
  draw_instruments();
  setpalette(0,GREEN);

  /*LOAD UP ALL THE WORLD DATA FROM SCENERY FILE*/
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

  /*PRESSING RETURN EXITS THE GAME*/
  while(read_key!=RETURN)
  {
   /*CHECK FOR A KEYPRESS*/
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

	  case(UP)        :plane_details->dive_angle+=0.2;
					   update_dive_angle(plane_details,nav_details);
					   break;

	  case(DOWN)      :plane_details->dive_angle-=0.2;
					   update_dive_angle(plane_details,nav_details);
					   break;

	  case(LEFT)      :if(nav_details->alt>0 && plane_details->speed!=0)
						 {
						  plane_details->bank_angle--;
						  bank_plane(plane_details->bank_angle,plane_details);
						  temp_rot=(1091*quick_tan(plane_details->bank_angle))/(plane_details->speed*0.5925);
						 }
					   break;

	case(RIGHT)       :if(nav_details->alt>0 && plane_details->speed!=0)
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

	case(RUDDER_RIGHT) : plane_details->rudder=min(plane_details->rudder+1,25);
						 if(plane_details->speed!=0)
						   temp_rot=(1091*quick_tan(plane_details->rudder))/(plane_details->speed*0.5925);
						 break;

	case(RUDDER_LEFT) : plane_details->rudder=max(plane_details->rudder-1,-25);
						if(plane_details->speed!=0)
						  temp_rot=(1091*quick_tan(plane_details->rudder))/(plane_details->speed*0.5925);
						break;

	case(FRONT_VIEW)  : view_angle=0;
						strcpy(view_text,"");
						break;

	case(LEFT_VIEW)   : view_angle=90;
						strcpy(view_text,"LEFT");
						break;

	case(RIGHT_VIEW)  : view_angle=-90;
						strcpy(view_text,"RIGHT");
						break;

	case(REAR_VIEW)   : view_angle=180;
						strcpy(view_text,"REAR");
						break;

	case(GEAR)        : move_gear(plane_details);
						break;

	case(BRAKES)      : change_brakes(plane_details);
						break;

	case(CHANGE_NAV1) : change_nav(nav_details,NAV1);
						break;

	case(CHANGE_NAV2) : change_nav(nav_details,NAV2);
						break;

	case(FLAPS_DOWN)  : change_flaps(plane_details,DOWN);
						break;

	case(FLAPS_UP)    : change_flaps(plane_details,UP);
						break;

	case(CHANGE_OMI_100):change_omi_radial(nav_details,100);
						break;

	case(CHANGE_OMI_10) :change_omi_radial(nav_details,10);
						break;

	case(CHANGE_OMI_1)  :change_omi_radial(nav_details,1);
						break;

	case(CUT_ENGINE)    :plane_details->thrust=0;
						 nosound();
						break;

	case(MENU)          :
						if(page!=0)
						   { setactivepage(0);
							 setvisualpage(0);
						   }
						menu_bar(nav_details,plane_details,airport_list,no_of_airports);
						break;
	  }
	 }

  /*UPDATE RATE OF TURN OF PLANE*/
  plane_details->rot+=temp_rot/2;

  if(plane_details->rot==0)
	 plane_details->rot=0;
  if(plane_details->rot>=360)
	 plane_details->rot-=360;
  if(plane_details->rot<0)
	 plane_details->rot+=360;


  /*UPDATE ORIENTATION OF PLANE*/
  update_orientation(plane_details,view_angle);

  move_plane(plane_details,nav_details);

  if(nav_details->alt>0)
	 bank_plane(plane_details->bank_angle,plane_details);


	setactivepage(page&1);
	setviewport(0,0,639,349,1);

	/*REFRESH ALL INSTRUMENTS*/
	update_instruments(nav_details,plane_details);

	setviewport(0,0,639,200,1);
	clearviewport();

	/*CREATE THE VIEW OF THE WORLD*/
	set_view_reference_point(nav_details->ew,plane_details->view_alt,nav_details->ns);
	set_perspective((nav_details->ew-5)*plane_details->view_x,(plane_details->view_alt-5)*plane_details->view_y,(nav_details->ns-5)*plane_details->view_z);
	transform_mat=trans_eye(plane_details);
	create_horizon(transform_mat,plane_details->rot,nav_details,view_angle);
	create_view(transform_mat,vertex_list,face_list,shape_list,extent_list,no_of_shapes,no_of_extents);

	setcolor(RED);

	/*PRINT CURRENT VIEW ON SCREEN*/
	outtextxy(305,10,view_text);

	free(transform_mat);
	setvisualpage(page&1);
	page++;
	/*CHECK IF THE PLANE HAS CRASHED*/
	check_for_crash(plane_details,nav_details);
  }
nosound();
page++;
}

/*============================================================================
								CHECK_FOR_CRASH
==============================================================================
	Checks if any of the conditions pertaining to a plane crash hold.
============================================================================*/
void check_for_crash(plane plane_details,nav nav_details)
{
 if((plane_details->gear==UP && nav_details->alt==0) ||(nav_details->alt<10 && plane_details->bank_angle>1)
	||(nav_details->alt<=0 && plane_details->speed<-400))
   {
	crash(320,100,15,1,1);
	nosound();
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
	 sound(x_rand*2);
	 delay(2);
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
	Updates the orientation of the plane.
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
						INITIALIZE_THRUST_REQUIRED
==============================================================================
============================================================================*/
void initialize_thrust_required(plane plane_details)
{
 L_D+=plane_details->fuel/100;
 MINDRAG+=plane_details->fuel/20000;
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
								CREATE_VIEW
==============================================================================
	Creates the current view from the cockpit.
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

 /*TEST EXTENTS FOR VISIBILITY*/
 test_extents(extent_list,no_of_extents,transform);

 /*FOR EVERY SHAPE IN THE WORLD*/
 for(shape_count=0;shape_count<no_of_shapes;shape_count++)
 {
   /*CHECK THAT THE SHAPES EXTENT IS VISIBLE*/
   if(extent_list[shape_list[shape_count]->extent_no]->clip!=CLIPPED)

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

		/*IF THE SHAPE IS NOT TOO FAR AWAY TO BE SEEN*/
		if(!invisible(z,shape_list[shape_count]->detail_level))
		   {
			 if((z<=z2))  /*CLIP TO FRONT PLANE*/
			   {
				line_status=clip(&x,&y,&z,x2,y2,z2);
				end_clipped=NEAR;
			   }

			 else
			   {
				line_status=clip(&x2,&y2,&z2,x,y,z); /*CLIP TO FRONT PLANE*/
				end_clipped=FAR;
			   }
		   }
		   else
			 break;


	  if(z!=-1000 && z2!=-1000)/*IF THE LINE IS VISIBLE*/
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
		  }
	  }

	 poly_array[poly_count]=-9999;

	 /*IF THE SHAPE IS TO BE COLOUR FILLED*/
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
	Check for visibility due to distance.
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
	Test all extents for visibility.
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


		if((z<=z2))/*CLIP TO FRON CLIPPING PLANE*/
		  {
		   clip(&x,&y,&z,x2,y2,z2);
		  }
		else
		  {
		   clip(&x2,&y2,&z2,x,y,z);/*CLIP TO FRONT CLIPPING PLANE*/
		  }

		if(z==-1000 || z2==-1000)  /*CHECK IF LINE IS VISIBLE*/
		  clip_count++;
		else
		  {
		   x=x*2;
		   y=y*2;
		   x2=x2*2;
		   y2=y2*2;

		   /*CLIP TO CANONICAL VIEW VOLUME*/
		   clip_planes(&x,&y,&z,&x2,&y2,&z2,0,EXTENT_FILL);

		   if(z==-9999)
			  clip_count++;
		  }

	   }

	 if(clip_count==4)
	   extent_list[extent_count]->clip=CLIPPED; /*EXTENT IS INVISIBLE*/
	 else
	   extent_list[extent_count]->clip=NOT_CLIPPED;
	 clip_count=0;
 }
}
/*============================================================================
								TRANSFORM_POINT
==============================================================================
	Apply a transformation matrix to a point.
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

 /*update_artificial_horizon(plane_details->bank_angle);*/
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
  static float old_alt;

  update_fuel_flow(plane_details);
  update_lift(plane_details,nav_details);
  update_dive_angle(plane_details,nav_details);
  update_speed(plane_details,nav_details);


  actual_speed=plane_details->speed*0.5792;

  old_alt=nav_details->alt;

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
	 if(old_alt>0 && plane_details->gear==DOWN)
	   {
		sound(350);
		delay(1);
	   }
	}

  plane_details->view_alt=nav_details->alt+10;
}

/*============================================================================
								UPDATE_SPEED
==============================================================================
	Calculates acceleration of plane.
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


	  drag=500*(MINDRAG*((drag_param1*drag_param1)+(drag_param2*drag_param2)));

	  drag=drag+((1/quick_cos(plane_details->bank_angle/D_TO_R))
		  *(1/quick_cos(plane_details->bank_angle/D_TO_R)-1)*drag);
	 }

  drag=drag+plane_details->aoa_drag;

  /*IF PLANE IS MOVING ON THE GROUND*/
  if(plane_details->speed!=0 && nav_details->alt==0)
	 drag=P_DRAG*(plane_details->speed*0.5925/L_D)*(plane_details->speed*0.5925/L_D);

  if(nav_details->alt==0.0)
	   drag=drag+(total_weight/GRAVITY)*0.8+fabs(plane_details->acceleration)*(GRAVITY/total_weight)+BRAKE_DRAG*plane_details->brakes;

  /*SET UPPER LIMIT ON DRAG*/
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

  if(plane_details->thrust==0)
	 nosound();
  else
   sound((int)plane_details->thrust/40);
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

	/*CHECK FOR STALL*/
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
								UPDATE_TRANSFORM
==============================================================================
============================================================================*/
void update_transform(shape shape1)
{
  initialise_matrix(shape1->shape_mat);
  shape1->shape_mat->entry[0][0]=shape1->shape_mat->entry[1][1]=shape1->shape_mat->entry[2][2]=1;
  scale(shape1->shape_mat,shape1->scale_x,shape1->scale_y,shape1->scale_z);
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
  nav_details->ns=-160000.0;
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































