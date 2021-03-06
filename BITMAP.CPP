#include<stdio.h>
#include<graphics.h>
#include<process.h>

/*============================================================================
							DISPLAYMAP
==============================================================================
	This function reads in a Windows Paintbrush file and displays it.
============================================================================*/
void displaymap(char *filename,int x_offset,int y_offset)
{
 #define HEADERSIZE   118
 #define TOP          0

  FILE *ifp;
  char ch;
  char p1,p2;
  int i=0,x=0,y = 149,fd;
  void *bitbuf;
  void *temp;
  char *t_read;
  int *intbit;


 if ((ifp = fopen(filename,"rb")) == NULL) {
	   printf("The following file not found --> %s",filename);
	   printf("\n\n Program terminating ");
	   exit(0);
  }


  for (i=0;i<=HEADERSIZE;i++) fgetc(ifp);  /*REMOVE HEADER*/

  while (y >= 0)
	   {
	 ch = fgetc(ifp);
	 p1 = ch & 240 ;
	 p2 = ch & 15 ;
	 p1 = p1 >> 4 ;


	 putpixel(x++,y+y_offset,p1);
	 putpixel(x++,y+y_offset,p2);
	 if (x >= 639) { x = 0 ;  /*END OF LINE REACHED - START NEW LINE*/
			   --y ; }
	   }
	   fclose(ifp);
}

