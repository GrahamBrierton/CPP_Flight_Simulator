/*===========================================================================
	  THIS MODULE CONTAINS ALL FUNCTIONS NECESSARY FOR ALL FILE OPERATIONS
  AND DIRECTORY MANIPULATION.
===========================================================================*/

#include<dir.h>
#include<string.h>
#include<sys\types.h>
#include<sys\stat.h>
#include"mouse.h"
#include"disc.h"

/*=========================================================================*/
/*                            GET_DIR                                      */
/*=========================================================================*/
/*   This functions primary task is to return a filenmae to be loaded or
  written to depending on the mode selected.
     It also controls the updating of pathnames and filenames.

   PARAMETERS PASSED : disc_mode :- value representing what sort of disk
				    access has been selected.

   FUNCTIONS CALLED  : show_dir()         getfileinfo()
			   getfilename()      drawselectbox()
		       edit_pathname()    edit_filename()

   CALLED BY         : main();

===========================================================================*/


char *edit_filename(char*);


char *get_dir(int disc_mode)
{
  int count=1,choice=0,oldchoice=0,xmouse,ymouse;
  long totalsize;


  char filenames[100][13];
  char *save_file,drive[MAXDRIVE],dir[MAXDIR],fname[MAXFILE],ext[MAXEXT];


  while(1)
  {
	count=1;
	message_info=help_message[16];
    messagebar(1);
    /*FILL ARRAY FILENAMES WITH THE NAMES OF FILES IN THE CURRENT DIRECTORY*/
	getfileinfo(filenames,&count);

    hidemouse();

	/*DISPLAY THE FILENAMES CONTAINED IN THE ARRAY FILENAMES*/

    show_dir(filenames,count);
    bar(220,215,380,230);
	showmouse();

    /*WHILE A CHOICE HAS NOT BEEN MADE*/
    while(!mousebuttonpressed(RIGHTBUTTON))
	   {
	    /*FIND OUT WHAT FILE THE MOUSE IS POINING TO*/
	    choice=getfilename(count);

		/*IF THE SELECTION IS A NEW ONE*/
		if(choice!=oldchoice)
	      {
		hidemouse();

		/*ERASE THE SELECTION BOX AROUND THE OLD CHOICE*/
		setcolor(BLUE);
		draw_select_box(oldchoice);

		/*DRAW A SELECTION BOX AROUND NEW CHOICE*/
		setcolor(WHITE);
		draw_select_box(choice);

		oldchoice=choice;

		/*DISPLAY THE NAME OF THE FILE WHICH HAS BEEN CHOSEN*/
		bar(220,215,380,230);
		outtextxy(250,220,filenames[choice]);
		showmouse();
	      }
		}
	getmousecoords(&xmouse,&ymouse);

	/*IF A FILE HAS NOT BEEN SELECTED THEN WHAT HAS ?*/

	/*UPDATING OF A FILENAME*/
	if(mouseinbox(220,215,380,230,xmouse,ymouse) && disc_mode==SAVE)
	    return  edit_filename(filenames[choice]);
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
	 }
	else

	/*NOTHING AT ALL*/
	if(strcmp(filenames[choice],"No Files!")!=0
	   && strcmp(filenames[choice],"CANCEL!")!=0)
	   return (char *)filenames[choice];
	else
	return NULL;
	 }
}


/*=========================================================================*/
/*                     DRAW_SELECT_BOX                                     */
/*=========================================================================*/
/*  Draws a box around the filename which the mouse is currently pointing
  at.

  PARAMETERS PASSED  : The number of the filename being pointed to.

  CALLED BY          : get_dir()
===========================================================================*/
draw_select_box(int choice)
{
 /*IF IN LEFTHAND COLUMN*/
 if(choice%2==0)
       rectangle(208,59+((choice/2)*10),298,69+((choice/2)*10));
    else
 /*MUST BE IN RIGHTHAND COLUMN*/
       rectangle(300,59+(((choice-1)/2)*10),398,69+(((choice-1)/2)*10));
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

void show_dir(char filenames[100][13],int count)
{
 int xtext=210,ytext=60,i;
 char pathname[81];

 /*GET CURRENT PATHNAME AND STORE IT IN THE ARRAY PATHNAME*/
 getcwd(pathname,80);

 /*DRAW BOX TO DISPLAY FILENAMES*/
 setcolor(WHITE);
 setfillstyle(SOLID_FILL,BLUE);
 rectangle(198,48,402,202);
 bar(200,50,400,200);

 /*DRAW BOX TO DISPLAY PATHNAME*/
 bar(185,30,415,45);
 outtextxy(190,32,pathname);

 /*THIS LOOP PRINTS THE FILENAMES ONE ROW AT A TIME IN THE FILENAMES BOX*/
 for(i=0;i<count;i++)
   {
    outtextxy(xtext,ytext,filenames[i]);
    xtext+=100;

	/*CHECK IF RIGHT HAND SIDE OF BOX HAS BEEN REACHED*/
    if(xtext>340)
      {
	   xtext=210;     /*IF SO GO TO NEXT LINE*/
	   ytext+=10;
      }
   }
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

 boxw=75;                     /*WIDTH OF BOX CONTAINING FILENAME*/
 boxh=10;                     /*HEIGHT OF BOX CONTAINING FILENAME*/

 getmousecoords(&xmouse,&ymouse);  /*GET CURRENT MOUSE COORDINATES*/

 nx=((350-xmouse)/boxw)+1;
 ny=((200-ymouse)/boxh)+1;

 if(mouseinbox(200,50,350,200,xmouse,ymouse))
	choice=27-(((ny-1)*2+nx)-1); /*THIS IS THE NUMBER OF THE BOX SELECTED*/

 if(choice>=count||choice<0)
    return 0;
 else
	return choice;
}


/*=========================================================================*/
/*                            GET_MODE                                     */
/*=========================================================================*/
/*   This function allows the user which disk access mode he/she would like.
  IE. Save or load.

  CALLED BY : main();

===========================================================================*/

int get_mode()
{
 int xmouse,ymouse;

 hidemouse();
 setfillstyle(SOLID_FILL,BLUE);
 setcolor(WHITE);
 bar (250,100,300,120);
 bar (310,100,360,120);
 outtextxy(260,105,"SAVE");
 outtextxy(320,105,"LOAD");
 showmouse();

 do{
	while(!mousebuttonpressed(RIGHTBUTTON))
	;
	getmousecoords(&xmouse,&ymouse);

	/*IF IN THE SAVE BOX*/
	if(mouseinbox(250,100,300,120,xmouse,ymouse))
	   return SAVE;

	/*IF IN THE LOAD BOX*/
	if(mouseinbox(310,100,360,120,xmouse,ymouse))
	   return LOAD;
   }
 while(1);
}




/*=========================================================================*/
/*                            GETFILEINFO                                  */
/*=========================================================================*/
/*  This function finds all occurances of files with the extension .ANI and
  returns them in an array of filenames.

  PARAMETERS PASSED : filenames[][] :- array of filenames.
		      count         :- number of filenames found.

  CALLED BY         : get_dir();

===========================================================================*/
void getfileinfo(char filenames[100][13],int *count)
{
 struct stat info;
 struct ffblk fileinfo;

  /*FIND FIRST OCCURANCE OF SUCH A FILE*/
  if(findfirst("*.ANI",&fileinfo,0) != -1)
      {
       /*PUT IT IN FIRST POSITION IN ARRAY*/
       strcpy(filenames[0],(fileinfo.ff_name));

       /*FIND THE REST OF THE FILENAMES WITH THE .ANI EXTENSION*/
	   while(findnext(&fileinfo)==0)
		{
	     strcpy(filenames[*count],(fileinfo.ff_name));
	     ++*count;
	    }
	   strcpy(filenames[*count],"CANCEL!");
       ++*count;
      }
  else
    /*IF NO FILES ARE FOUND*/
	strcpy(filenames[0],"No Files!");
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
 char pathname[81];
 int xdisp,ydisp,length=0;
 struct dfree disk_info;
 hidemouse();

 message_info=help_message[18];
 messagebar(1);
 /*HIGHLIGHT THE PATHNAME BOX*/
 setcolor(getbkcolor());
 rectangle(198,48,402,202);
 setcolor(WHITE);
 rectangle(182,28,417,47);

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

  message_info=help_message[17];
  messagebar(1);
  setcolor(getbkcolor());
  rectangle(198,48,402,202);

  /*HIGHLIGHT FILENAME BOX*/
  setcolor(WHITE);
  rectangle(218,213,382,232);

  /*COUNT NUMBER OF CHARACTERS IN FILENAME*/
  while(filenames[length]!='\x00')
       ++length;

  /*CALCULATE CURSOR POSITION*/
  xdisp=250+(length*8);
  ydisp=220;

  /*EDIT THE FILNAME STRING*/
  edit_string(filenames,xdisp,ydisp,length);


  setcolor(getbkcolor());
  rectangle(218,213,382,282);
  setcolor(WHITE);
  showmouse();

  /*APPEND 'ANI' EXTENSION TO END OF FILENAME*/
  length=0;
  while(filenames[length]!='.')
	++length;
  filenames[length+1]='\x00';
  strcat(filenames,"ANI");

  return filenames;
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
      setfillstyle(SOLID_FILL,BLUE);

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
/*                          UPDATE_PATH                                    */
/*=========================================================================*/
/*   This updates the current path according to the path stored in pathname.

   PARAMETERS PASSED : pathname :- array containing new pathname.

   CALLED BY         : edit_pathname();

===========================================================================*/
int update_path(char pathname[81])
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
/*                         DISPLAY_ERROR                                   */
/*=========================================================================*/
/* This function displays any disk errors which may have take place.

   PARAMETERS PASSED : error :- character pointer to the error message.
		       color :- value of current drawing colour.

   CALLED BY         : harderror_handler()
		       read_anim();
			   write_anim();

===========================================================================*/

void display_error(char *error,int color)
{
 setfillstyle(SOLID_FILL,RED);
 setcolor(WHITE);
 bar(200,200,380,220);
 outtextxy(202,205,error);

 getch();  /*WAIT FOR A KEYPRESS*/

 setfillstyle(SOLID_FILL,color);
 setcolor(color);
}


/*=========================================================================*/
/*                             DISK_OK                                     */
/*=========================================================================*/
/*  This function makes sure that there is enough space on the disk to
   write the animation.

   PARAMETERS PASSED : color :- current drawing colour.

   CALLED BY         : write_anim();

==========================================================================*/

int disk_ok(int color)
{
 unsigned long bytes_per_cluster,free_space;
 unsigned curr_free_mem;

 struct dfree dfinfo;

 getdfree(0,&dfinfo);    /*GET INFORMATION ON DISK STATUS*/

 if(dfinfo.df_sclus==0)
   {
    display_error(file_error[0],color);
    return FALSE;
   }

 bytes_per_cluster=dfinfo.df_sclus * dfinfo.df_bsec;

 /*CALCULATE FREE SPACE AVAILABLE*/
 free_space=dfinfo.df_avail*bytes_per_cluster;

 curr_free_mem=coreleft();

 /*COMPARE SIZE OF ANIMATION WITH FREE DISK SPACE*/
 if(free_space<=startfreemem-curr_free_mem)
    return FALSE;

 return TRUE;
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

   CALLED BY         : main()

   FUNCTIONS CALLED  : display_error();

===========================================================================*/

int harderror_handler(int errorcode,int deverror,int bpval,int sival)
{
  char dletter;


  hardware_error=1;  /*SET ERROR FLAG TO REFLECT AN ERROR HAS OCCURED*/

  if((errorcode & 0xff)==DRIVE_NOT_READY)

       display_error(file_error[4],0);  /*DISPLAY ERROR MESSAGE*/

  hardretn(-1);
}
