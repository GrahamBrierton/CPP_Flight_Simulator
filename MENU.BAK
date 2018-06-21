/*****************************************************************************
	THIS MODULE CONTAINS ALL FUNCTIONS FOR HANDLING THE MENU OPTIONS.
*****************************************************************************/
#include <graphics.h>
#include <string.h>
#include <ctype.h>
#include <bios.h>
#include <conio.h>
#include <stdio.h>
#include <process.h>
#include<sys\types.h>
#include<sys\stat.h>
#include <dos.h>
#include <dir.h>
#include <alloc.h>
#include "objects.h"
#define ESCAPE 1
#include "mouse.h"
#include "disc.h"

/*============================================================================
								MENU_BAR
==============================================================================
	Draws the menu bar.
============================================================================*/
void menu_bar(nav nav_details,plane plane_details,airport *airport_details,int no_of_airports)
{
  int option;

  setfillstyle(SOLID_FILL,LIGHTGRAY);
  setcolor(LIGHTGRAY);

  bar(0,0,639,10);

  setcolor(RED);
  outtextxy(10,1,"F");
  setcolor(LIGHTGREEN);
  outtextxy(18,1,"ILE");

  setcolor(RED);
  outtextxy(90,1,"L");
  setcolor(LIGHTGREEN);
  outtextxy(98,1,"OCATION");

  /*WAIT FOR USER TO CHOOSE AN OPTION*/
  while(bioskey(1)==0);
		 option=getch();
		 switch(toupper(option))
			   {
				 case 'F': file_menu(nav_details,plane_details);
						   break;
				 case 'L': location_menu(nav_details,airport_details,no_of_airports);
						   initialize_plane(plane_details);
						   break;
				 default : return;
				}

}

/*============================================================================
								LOCATION MENU
==============================================================================
	Menu listing all the locations the plane can take off from.
============================================================================*/
void location_menu(nav nav_details,airport *airport_details,int no_of_airports)
{
  int choice;
  int count=0;
  char **temp_airport_array;

  /*STORE ALL THE AIRPORT NAMES IN AN ARRAY*/
  for(count=0;count<no_of_airports;count++)
	 {
	  temp_airport_array[count]=(char*)malloc(35);
	  sprintf(temp_airport_array[count],"%d  ",count);
	  strcat(temp_airport_array[count],airport_details[count]->airport_name);
	 }
  count=0;

  /*DRAW LIST OF AIRPORTS*/
  choice=menu(90,10,temp_airport_array,no_of_airports);

  /*MAKE SURE THAT A VALID OPTION HAS BEEN MADE*/
  if(choice!=9999)
	 count=choice;

  /*SET LOCATION TO NEW AIRPORT*/
  nav_details->ew=airport_details[count]->ew_coord;
  nav_details->ns=-airport_details[count]->ns_coord;
  nav_details->alt=0.0;

  /*FREE UP MEMORY*/
  for(count=0;count<no_of_airports;count++)
	 {
	  free(temp_airport_array[count]);
	 }
}

/*============================================================================
								FILE_MENU
==============================================================================
============================================================================*/
void file_menu(nav nav_details,plane plane_details)
{
 int choice;
 char *filename;

  choice=menu(10,10,file_options,4);
  switch(choice){
				case 9999:return;
						  break;
				case 0   :filename=get_dir(LOAD);
						  if(filename!=NULL)
							{
							 load(filename,nav_details,plane_details);
							 free(filename);
							}
						  break;
				case 1   :filename=get_dir(SAVE);
						  if(filename!=NULL)
							{
							 save(filename,nav_details,plane_details);
							 free(filename);
							}
						  break;
				case 2   :closegraph();
						  exit(1);
			   }
}

/*============================================================================
								GET_DIR
==============================================================================
	Gets and prints the contents of a directory.
============================================================================*/
char *get_dir(int disc_mode)
{
 char **filenames;
 char *chosen_file;
 int count=1;
 int choice=0,oldchoice=0,xmouse,ymouse;
 long totalsize;
 int i;

 initmouse();
 count=1;
 getfileinfo(filenames,&count);
 show_dir(filenames,count);
 drawbox(220,160,380,175);

 while(1)
 {

	/*WHILE A CHOICE HAS NOT BEEN MADE*/
	while(!mousebuttonpressed(LEFTBUTTON));
		/*FIND OUT WHAT FILE THE MOUSE IS POINING TO*/
		choice=getfilename(count);

		/*IF THE SELECTION IS A NEW ONE*/
		if(choice!=oldchoice)
		  {
		hidemouse();

		/*ERASE THE SELECTION BOX AROUND THE OLD CHOICE*/
		setcolor(LIGHTGRAY);
		draw_select_box(oldchoice);

		/*DRAW A SELECTION BOX AROUND NEW CHOICE*/
		setcolor(LIGHTGREEN);
		draw_select_box(choice);

		oldchoice=choice;

		/*DISPLAY THE NAME OF THE FILE WHICH HAS BEEN CHOSEN*/
		setcolor(LIGHTGRAY);
		setfillstyle(SOLID_FILL,LIGHTGRAY);
		bar(222,162,378,173);
		setcolor(LIGHTGREEN);
		outtextxy(250,164,filenames[choice]);
		showmouse();
		  }
		getmousecoords(&xmouse,&ymouse);
	while(!mousebuttonreleased(LEFTBUTTON));
	/*CHECK FOR DOUBLE CLICK*/
	for(i=0;i<500;i++)
	  {
		if(mousebuttonpressed(LEFTBUTTON))
		{
		 if(strcmp(filenames[choice],"No Files!")!=0
			&& strcmp(filenames[choice],"CANCEL!")!=0)
		   {
			mousesoftwarereset();
			chosen_file=(char*)malloc(strlen(filenames[choice]));
			strcpy(chosen_file,filenames[choice]);
			delete_filenames(filenames,count);
			return (char *)chosen_file;
		   }
		}
	   }

	/*IF A FILE HAS NOT BEEN SELECTED THEN WHAT HAS ?*/

	/*UPDATING OF A FILENAME*/
	if(mouseinbox(220,160,380,175,xmouse,ymouse) && disc_mode==SAVE)
	   {
		mousesoftwarereset();
		chosen_file=(char*)malloc(strlen(filenames[choice]));
		strcpy(chosen_file,filenames[choice]);
		delete_filenames(filenames,count);
		return  edit_filename(chosen_file);
	   }
	else

	/*UPDATING OF THE PATHNAME*/
	if(mouseinbox(185,30,415,45,xmouse,ymouse))
	 {
	  edit_pathname();
	  while(hardware_error)
		  {
		   hardware_error=0;
		   edit_pathname();
		  }
	  hidemouse();
	  count=1;
	  getfileinfo(filenames,&count);
	  show_dir(filenames,count);
	  drawbox(220,160,380,175);
	  showmouse();
	 }
	else

	/*NOTHING AT ALL*/
	   {
		mousesoftwarereset();
		hidemouse();
		return NULL;
	   }
	}
}


/*============================================================================
							 DELETE_FILENAMES
==============================================================================
   Releases memory allocated to store filenames.

PARAMETERS PASSED :    filenames      : List of filenames
					   number_of_files: Number of names in list

CALLED BY         :    get_dir()
============================================================================*/
void delete_filenames(char **filenames,int number_of_files)
{
 int count;

 for(count=0;count<number_of_files;count++)
	 free(filenames[count]);
}


/*============================================================================
								SAVE
==============================================================================
	Saves current flight simulator status.

PARAMTERS PASSED   :  filename     : Name under which status is to be saved.
					  nav_details  : Current navigation details.
					  plane_details: Current plane_details.

CALLED_BY		   :  file_menu()

CALLS              :  display_error()
============================================================================*/
void save(char *filename,nav nav_details,plane plane_details)
{
 FILE *outfile;
 float *floatptr;
 int count;

 floatptr=(float*)nav_details;
 outfile=fopen(filename,"w"); /*OPEN A FILE*/

 /*WRITE OUT HEADER*/
 fputs("FLIGHT_SIMULATOR!\n",outfile);

 /*WRITE OUT NAVIGATION DETAILS*/
 for(count=0;count<sizeof(NAV)/sizeof(float);count++)
	 {
	   fprintf(outfile,"%f ",(float)*floatptr++);
	  }
	   fprintf(outfile,"\n");


 floatptr=(float*)plane_details;

 /*WRITE OUT PLANE STATUS DETAILS*/
 for(count=0;count<sizeof(PLANE)/sizeof(float);count++)
	 {
	  fprintf(outfile,"%f ",(float)*floatptr++);
	 }

 if(outfile==NULL)
	display_error(file_error[0]);
 if(ferror(outfile)!=0)
	display_error(file_error[1]);

 fclose(outfile);

 hardware_error=0;

}

/*============================================================================
								LOAD
==============================================================================
	Loads a saved game.

PARAMETERS PASSED    : filename     : Name of file to be loaded.
					   nav_details  : Structure containing current position
									  of aircraft.
					   plane_details: Structure containing current
									  disposition of aircraft.

CALLED BY			: file_menu()

CALLS				: display_error()
============================================================================*/
void load(char *filename,nav nav_details,plane plane_details)
{
 FILE *infile;
 char check_string[20];
 float *floatptr;
 float read_info;
 int count;

 infile=fopen(filename,"r");

 /*READ IN HEADER*/
 fgets(check_string,20,infile);

 /*CHECK IF HEADER MATCHES FLIGHT_SIMULATOR FORMAT*/
 if(strcmp(check_string,"FLIGHT_SIMULATOR!\n")!=0)
	display_error(file_error[5]);
 else
 {
 floatptr=(float*)nav_details;

 /*LOAD SAVED NAVIGATION DETAILS*/
 for(count=0;count<sizeof(NAV)/sizeof(float);count++)
	   {
		  fscanf(infile,"%f",&read_info);
		  *floatptr++=read_info;
	   }

 floatptr=(float*)plane_details;

 /*LOAD SAVED PLANE STATUS DETAILS*/
 for(count=0;count<sizeof(PLANE)/sizeof(float);count++)
	   {
		  fscanf(infile,"%f",&read_info);
		  *floatptr++=read_info;
	   }

 if(infile==NULL)
	display_error(file_error[0]);
 if(ferror(infile)!=0)
	display_error(file_error[2]);

  }
 fclose(infile);
}
/*=========================================================================*/
/*                         DISPLAY_ERROR                                   */
/*=========================================================================*/
/* This function displays any disk errors which may have take place.

   PARAMETERS PASSED : error :- character pointer to the error message.

   CALLED BY         : harderror_handler()
					   load();
					   save();

===========================================================================*/
void display_error(char *error)
{
 void far *screenptr;
 unsigned size;

 size = imagesize(200,100,400,120);
 if ((screenptr = farmalloc(size)) == NULL)
	  {
		 closegraph();
		 printf("Error: not enough heap space in display_error().\n");
	  }

	  getimage(200,100,400,120,screenptr);


 drawbox(200,100,400,120);
 setcolor(RED);
 outtextxy(202,105,error);

 getch();  /*WAIT FOR A KEYPRESS*/

 putimage(200,100,screenptr,COPY_PUT);
 farfree(screenptr);
}

/*=========================================================================*/
/*                        HARDERROR_HANDLER                                */
/*=========================================================================*/
/*  This function replaces the handler for DOS interrupt 24h (critical error)
   That is that any time a disk hardware error occurs this funtion will be
   called.

   PARAMETERS PASSED : errorcode  :- DI register value containing error code
					   deverror   :- AX register value containing more
									 detailed info on disk error.
					   bpval,sival:- BP and SI register values acting as a
									 pointer to information on the device
									 where the error occured.


   FUNCTIONS CALLED  : display_error();

===========================================================================*/

int harderror_handler(int errorcode,int deverror,int bpval,int sival)
{
  char dletter;


  hardware_error=1;  /*SET ERROR FLAG TO REFLECT AN ERROR HAS OCCURED*/

  if((errorcode & 0xff)==DRIVE_NOT_READY)

	  hardware_error=1;
	 /*  display_error(file_error[4],0);*/  /*DISPLAY ERROR MESSAGE*/

  hardretn(-1);
}

/*=========================================================================*/
/*                          UPDATE_PATH                                    */
/*=========================================================================*/
/*   This updates the current path according to the path stored in pathname.

   PARAMETERS PASSED : pathname :- array containing new pathname.

   CALLED BY         : edit_pathname();

===========================================================================*/
int update_path(char* pathname)
{
 char drive[MAXDRIVE],dir[MAXDIR],filename[MAXFILE],ext[MAXEXT];
 int drivenum,curr_drive;
 struct dfree dfinfo;

 strupr(pathname); /*CONVERT ALL CHARACTERS TO UPPERCASE*/

 /*SPLIT PATHNAME INTO ITS COMPONENT PARTS*/
 fnsplit(pathname,drive,dir,filename,ext);

 /*GET THE CURRENT LOGGED DRIVE*/
 drivenum=getdisk();

 /*CONVERT THE DRIVE NUMBER TO A LETTER IE A,B,C .. ETC*/
 curr_drive=drive[0]-'A';

 /*IF THE DRIVE IN THE PATHNAME IS DIFFERENT TO THE LOGGED ONE THEN LOG
   ONTO THE DRIVE SPECIFIED IN THE PATH*/

 if(drivenum!=curr_drive)
	setdisk(curr_drive);

 /*IF AN ERROR OCCURS THEN RESET TO THE OLD LOGGED DRIVE*/
 if(chdir(dir)==-1)
   {
	setdisk(drivenum);
	return FALSE;
   }

 getdfree(drivenum,&dfinfo);

 /*COPY THE DRIVE AND PATHNAME TO THE CHARACTER ARRAY PATHNAME*/
 strcpy(pathname,drive);
 strncat(pathname,dir,MAXDIR);
 return TRUE;

}

/*=========================================================================*/
/*                          EDIT_PATHNAME                                  */
/*=========================================================================*/
/*  Allows the updating of a pathname including the changing of the logged
  drive.

  FUNCTIONS CALLED : edit_string();
		     update_path();

  CALLED BY        : get_dir();

===========================================================================*/

void edit_pathname()
{
 char *pathname=(char*)malloc(81);
 int xdisp,ydisp,length=0;
 struct dfree disk_info;
 hidemouse();


 /*GET THE CURRENT PATHNAME AND STORE IN THE ARRAY PATHNAME*/
 getcwd(pathname,80);

 /*COUNT NUMBER OF CHARACTERES IN PATHNAME*/
 while(pathname[length]!='\x00')
	   ++length ;

 /*CALCULATE THE CURRENT POSITION OF THE CURSOR*/
 xdisp=190+(length*8);
 ydisp=32;

 /*EDIT THE PATHNAME STRING*/
 edit_string(pathname,xdisp,ydisp,length);

 /*IF UPDATE HAS BEEN UNSUCCESSFULL REVERT BACK TO OLD PATHNAME*/
 if(!update_path(pathname))
	getcwd(pathname,80);


 setcolor(getbkcolor());
 rectangle(182,28,417,47);
 setcolor(WHITE);
 showmouse();
}
/*=========================================================================*/
/*                          EDIT_STRING                                    */
/*=========================================================================*/
/*  This function allows the editing of a text string in a graphics
   environment.

   PARAMETERS PASSED : ed_str :- character array to be edited.
			   xdisp  :- the x coordinate of the cursor.
			   ydisp  :- the y coordinate of the cursor.
			   length :- the length of the string.

   CALLED BY         : edit_filename();
			   edit_pathname();

===========================================================================*/

void edit_string(char ed_str[81],int xdisp,int ydisp,int length)
{
 int keystroke=0;
 char *output_string=" ";


 while(keystroke!=RETURN)
	 {
	  /*DRAW THE CURSOR*/
	  setfillstyle(SOLID_FILL,WHITE);
	  bar(xdisp,ydisp,xdisp+8,ydisp+8);
	  setfillstyle(SOLID_FILL,LIGHTGRAY);
	  setcolor(LIGHTGREEN);
	  if(kbhit())
	{
	  keystroke=getch();
	  switch(keystroke)
	    {

		 case RETURN: break;

		 case DELETE: if(length>0)
			 {
			  length--;
			  xdisp-=8;
			  /*UPDATE CURSOR*/
			  bar(xdisp+8,ydisp,xdisp+16,ydisp+8);
			  ed_str[length]='\x00';
			 }
			   break;

		 default:

			   sprintf(output_string,"%c",keystroke);

			   /*UPDATE CURSOR*/
			   bar(xdisp,ydisp,xdisp+8,ydisp+8);

			   /*PRINT KEY PRESSED*/
			   outtextxy(xdisp,ydisp,output_string);
			   xdisp+=8;

			   /*ADD CHARACTER TO ARRAY*/
			   ed_str[length]=(char)keystroke;
			   ++length;

			  break;
		}
	 }
	   }
}


/*=========================================================================*/
/*                          EDIT_FILENAME                                  */
/*=========================================================================*/
/*   This function allows the editing of a filename so the user can define
  his own filenames. The function then returns the filename.

  PARAMETERS PASSED : filnames :- name of filename to be edited

  FUNCTIONS CALLED  : edit_string();

  CALLED BY         : get_dir();

===========================================================================*/

char *edit_filename(char *filenames)
{
  int xdisp,ydisp,length=0;

  hidemouse();


  /*COUNT NUMBER OF CHARACTERS IN FILENAME*/
  while(filenames[length]!='\x00')
	   ++length;

  /*CALCULATE CURSOR POSITION*/
  xdisp=250+(length*8);
  ydisp=162;

  /*EDIT THE FILNAME STRING*/
  edit_string(filenames,xdisp,ydisp,length);


  showmouse();

  /*APPEND 'FSM' EXTENSION TO END OF FILENAME*/
  length=0;
  while(filenames[length]!='.')
	++length;
  filenames[length]='\x00';
  strcat(filenames,".FSM");

  return filenames;
}

/*=========================================================================*/
/*                     DRAW_SELECT_BOX                                     */
/*=========================================================================*/
/*  Draws a box around the filename which the mouse is currently pointing
  at.

  PARAMETERS PASSED  : The number of the filename being pointed to.

  CALLED BY          : get_dir()
===========================================================================*/
void draw_select_box(int choice)
{
 /*IF IN LEFTHAND COLUMN*/
 if(choice%2==0)
	   rectangle(198,59+((choice/2)*10),308,69+((choice/2)*10));
	else
 /*MUST BE IN RIGHTHAND COLUMN*/
	   rectangle(310,59+(((choice-1)/2)*10),408,69+(((choice-1)/2)*10));
}

/*=========================================================================*/
/*                            GETFILENAME                                  */
/*=========================================================================*/
/*  This function simply decides which filename (if any) if the directory
  listing has been chosen.  It checks if  the file chosen is by seeing
  if the choice number is less than or equal to the count value of files
  made in get_dir.

  PARAMETERS PASSED -  count :-number of files in directory.

  CALLED BY         -  get_dir();

===========================================================================*/
int getfilename(int count)
{
 int xmouse,ymouse,boxw,boxh,nx,ny,choice=0;

 boxw=130;                     /*WIDTH OF BOX CONTAINING FILENAME*/
 boxh=10;                     /*HEIGHT OF BOX CONTAINING FILENAME*/

 getmousecoords(&xmouse,&ymouse);  /*GET CURRENT MOUSE COORDINATES*/

 nx=((410-xmouse)/boxw)+1;
 ny=((190-ymouse)/boxh)+1;

 if(mouseinbox(190,50,410,200,xmouse,ymouse))
	choice=25-(((ny-1)*2+nx)-1); /*THIS IS THE NUMBER OF THE BOX SELECTED*/

 if(choice>=count||choice<0)
	return 0;
 else
	return choice;
}

/*=========================================================================*/
/*                             SHOW_DIR                                    */
/*=========================================================================*/
/*  This function prints out all the files in the current directory, stored
  in the array FILENAMES.

  PARAMETERS PASSED : filenames :- array of filenames.
					  count     :- number of files in array.

  CALLED BY         : get_dir()

===========================================================================*/
void show_dir(char **filenames,int count)
{
 int xtext=200,ytext=60,i;
 char pathname[81];

 /*GET CURRENT PATHNAME AND STORE IT IN THE ARRAY PATHNAME*/
 getcwd(pathname,80);

 /*DRAW BOX TO DISPLAY FILENAMES*/
 setcolor(LIGHTGREEN);
 setfillstyle(SOLID_FILL,LIGHTGRAY);

 drawbox(190,50,410,150);
 /*DRAW BOX TO DISPLAY PATHNAME*/
 drawbox(185,30,415,45);

 setcolor(LIGHTGREEN);
 outtextxy(190,34,pathname);


 setcolor(LIGHTGREEN);
 /*THIS LOOP PRINTS THE FILENAMES ONE ROW AT A TIME IN THE FILENAMES BOX*/
 for(i=0;i<count;i++)
   {
	outtextxy(xtext,ytext,filenames[i]);
	xtext+=110;

	/*CHECK IF RIGHT HAND SIDE OF BOX HAS BEEN REACHED*/
	if(xtext>340)
	  {
	   xtext=200;     /*IF SO GO TO NEXT LINE*/
	   ytext+=10;
	  }
   }
}

/*============================================================================
								DRAWBOX
==============================================================================
Draws a box of dimensions given in parameters.

PARAMTERS PASSED :  minx,miny,x_max,y_max : Extremeties of box.

CALLED BY        :  menu()
					show_dir()
					get_dir()
============================================================================*/
void drawbox(int minx,int miny,int x_max,int y_max)
{

  setfillstyle(SOLID_FILL,LIGHTGRAY);
  setcolor(LIGHTGREEN);

  bar(minx,miny,x_max,y_max);
  setcolor(WHITE);

  line(minx+1,miny+1,x_max-1,miny+1);
  line(minx+1,miny+1,minx+1,y_max-1);

  setcolor(DARKGRAY);

  line(minx+1,y_max-1,x_max-1,y_max-1);
  line(x_max-1,miny+1,x_max-1,y_max-1);
}

/*============================================================================
								MENU
==============================================================================
	This function constructs a menu from the list of options passed to it
and draws the menu at a location specified in the parameters.

PARAMETERS PASSED : x,y     : The location at which the menu is to be drawn.
					optlist : List of options to be included on the menu.
					no_of_opts : Number of options in optlist.

RETURNS           : Integer value representing the option chosen.

CALLED BY         : filemenu_()
					location_menu()

CALLS             : drawbox()
============================================================================*/
int menu(int x,int y,char **optlist,int no_of_opts)
{
  int y_max,x_max=0,i;
  char option;
  int scancode;

  y_max=y+(no_of_opts*10); /*FIND HEIGHT OF MENU BOX*/

  /*FIND WIDTH OF MENU BOX*/
  for(i=0;i<no_of_opts;i++)
	 {
	  if(strlen(optlist[i])>x_max/10)
		 x_max=strlen(optlist[i])*10;
	 }

  setfillstyle(SOLID_FILL,LIGHTGRAY);
  setcolor(LIGHTGREEN);

  /*DRAW MENU BOX*/
  drawbox(x,y,x_max+x,y_max+20);

  setcolor(LIGHTGREEN);

  /*DISPLAY OPTIONS IN BOX*/
  for(i=0;i<no_of_opts;i++)
	  outtextxy(x+10,y+10+(i*10),optlist[i]);

  while(1)
	 {
	  if(bioskey(1)!=0) /*WAIT FOR KEYPRESS*/
	  {
		option=getch();
		/*SEARCH OPTIONS TO SEE WHICH WAS SELECTED. 1st LETTER = KEY*/
		for(i=0;i<no_of_opts;i++)
		  if(toupper(option)==optlist[i][0])
				return i;
	  return 9999; /*NO OPTION CHOSEN*/
	  }
	 }
}

/*=========================================================================*/
/*                            GETFILEINFO                                  */
/*=========================================================================*/
/*  This function finds all occurances of files with the extension .fsm and
  returns them in an array of filenames.

  PARAMETERS PASSED : filenames[][] :- array of filenames.
					  count         :- number of filenames found.

  CALLED BY         : get_dir();

===========================================================================*/
void getfileinfo(char **filenames,int *count)
{
 struct stat info;
 struct ffblk fileinfo;

  /*FIND FIRST OCCURANCE OF SUCH A FILE*/
  if(findfirst("*.FSM",&fileinfo,0) != -1)
	  {
	   /*PUT IT IN FIRST POSITION IN ARRAY*/
	   filenames[0]=(char*)malloc(14);
	   strcpy(filenames[0],(fileinfo.ff_name));

	   /*FIND THE REST OF THE FILENAMES WITH THE .ANI EXTENSION*/
	   while(findnext(&fileinfo)==0)
		{
		 filenames[*count]=(char*)malloc(14);
		 strcpy(filenames[*count],(fileinfo.ff_name));
		 ++*count;
		}
	   filenames[*count]=(char*)malloc(14);
	   strcpy(filenames[*count],"CANCEL!");
	   ++*count;
	  }
  else
	/*IF NO FILES ARE FOUND*/
   {
	filenames[*count]=(char*)malloc(14);
	strcpy(filenames[0],"No Files!");
   }
}







