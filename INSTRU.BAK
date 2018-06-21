/**************************************************************************
	THIS MODELE CONTAINS ALL FUNCTIONS REQUIRED FOR THE UPDATING AND
 MAINTAINING OF INSTRUMENTATION.
**************************************************************************/
#include <graphics.h>
#include <stdio.h>
#include <alloc.h>
#include <dos.h>
#include <string.h>
#include "objects.h"
#include "instru.h"
#include "geo.h"

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
 if(brake==ON)      /*TURN ON BRAKE LIGHT*/
	{
	  setcolor(RED);
	  setfillstyle(SOLID_FILL,RED);
	  pieslice(607,312,0,360,2);
	 }
 else
   {                /*TURN OFF BRAKE LIGHT*/
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
 if(gear==DOWN)         /*TURN ON GEAR LIGHT*/
   {
	 setcolor(RED);
	 setfillstyle(SOLID_FILL,RED);
	 pieslice(607,292,0,360,2);
   }
 else
   {                    /*TURN OFF GEAR LIGHT*/
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

 pieslice(189,242,0,360,26);    /*ERASE OLD READING*/


 setcolor(RED);


 angle=36*(curr_alt/100);      /*CALCULATE HOW MANY DEGRESS THE NEW
								 READING (IN 100s OF FEET) TAKES*/

 angle_to_coord(&x,&y,angle);  /*CONVERT THIS ANGLE TO A COORDINATE
								 ON A CIRCLE PERIMETER*/

 x=x*25;
 y=y*-25;

 setcolor(WHITE);
 line(189,242,189+x,242+y);

 angle=36*(curr_alt/1000);	  /*CALCULATE HOW MANY DEGRESS THE NEW
								 READING (IN 1000s OF FEET) TAKES*/

 angle_to_coord(&x,&y,angle);  /*CONVERT THIS ANGLE TO A COORDINATE
								 ON A CIRCLE PERIMETER*/
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

 bar(482,293,567,296);       /*ERASE OLD READING*/

 setcolor(RED);
 setfillstyle(SOLID_FILL,RED);
 fuel_percentage=(fuel/MAXFUEL)*85; /*CALCULATE NEW READING*/

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

  bar(68,287,110,337);    /*ERASE OLD READING*/

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

 pieslice(289,240,0,360,21);  /*ERASE OLD READING*/

 angle=45*((speed*0.5925)*climb_vector*0.1)-90;/*CALCULATE NEW READING*/

 if(angle>180) angle=180;    /*CHECK FOR OVERFLOW*/
 if(angle<-180) angle=-180;  /*CHECK FOR UNDERFLOW*/

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

 bar(584,225,615,237);  /*ERASE OLD READING*/

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
 setfillstyle(SOLID_FILL,LIGHTGREEN); /*ERASE OLD READING*/
 bar(475,225,507,237);
 bar(475,247,507,259);
 bar(530,224,537,232);
 bar(531,249,537,256);

 /*CALCULATE  NAV1 READING*/
 angle=coord_to_angle(nav_details->ew,-nav_details->ns,nav_details->beacon_coords[nav_details->nav1*2],nav_details->beacon_coords[nav_details->nav1*2+1]);

 update_omi(angle,nav_details);

 setcolor(RED);
 sprintf(nav_text,"%d",(int)angle);
 outtextxy(477,227,nav_text);

 /*CALCULATE NAV2 READING*/
 angle=coord_to_angle(nav_details->ew,-nav_details->ns,nav_details->beacon_coords[nav_details->nav2*2],nav_details->beacon_coords[nav_details->nav2*2+1]);
 sprintf(nav_text,"%d",(int)angle);
 outtextxy(477,250,nav_text);

 /*UPDATE NAV1 VOR STATION*/
 sprintf(nav_text,"%d",(int)nav_details->nav1+1);
 outtextxy(530,225,nav_text);

 /*UPDATE NAV2 VOR STATION*/
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

 /*ERASE OLD READINGS*/
 bar(368,288,398,294);
 bar(368,331,397,337);
 bar(355,300,409,325);

 /*UPDATE COURSE DEVIATION NEEDLE*/
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
  bar(355,205,410,207);  /*ERASE OLD READINGS*/
  bar(355,270,410,272);

  xval=bank_angle;

  if(xval>27) xval=27;    /*CHECK FOR OVERFLOW*/
  if(xval<-27) xval=-27;  /*CHECK FOR UNDERFLOW*/

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
  bar(379,216,381,261);   /*ERASE OLD VALUE*/

  if(angle>23) angle=23;   /*CHECK FOR OVERFLOW*/
  if(angle<-23) angle=-23; /*CHECK FOR UNDERFLOW*/
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

 pieslice(190,312,0,360,30);   /*ERASE OLD READING*/

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
 bar(484,325,560,333);    /*ERASE OLD READING*/

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
 bar(475,270,540,282);  /*ERASE OLD READING*/

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

 pieslice(89,240,0,360,16);   /*ERASE OLD READING*/

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
 bar(280,290,305,300);               /*ERASE OLD READING*/

 sprintf(direction_text,"%d",(int)direction);
 setcolor(RED);
 outtextxy(280,290,direction_text);

 free(direction_text);
}