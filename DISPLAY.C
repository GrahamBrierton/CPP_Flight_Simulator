void displaymap(char *filename)

{
 #define HEADERSIZE   118
 #define TOP          0

  FILE *ifp;
  char ch;
  char p1,p2;
  int i=0,x=0,y = 200;


  if ((ifp = fopen(filename,"rb")) == NULL) {
       printf("The following file not found --> %s",filename);
       printf("\n\n Program terminating ");
       exit(0);
  }

  for (i=0;i<=HEADERSIZE;i++) fgetc(ifp);

  while (y >= 0)
       { ch = fgetc(ifp);
	 p1 = ch & 240 ;
	 p2 = ch & 15 ;
	 p1 = p1 >> 4 ;


	 putpixel(x++,y,p1);
	 putpixel(x++,y,p2);
	 if (x >= 300) { x = 0 ;
			   --y ; }
       }
       fclose(ifp);
}
