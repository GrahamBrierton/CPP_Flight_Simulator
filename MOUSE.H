
#ifndef boolean
#define boolean char
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#define LEFTBUTTON 0
#define RIGHTBUTTON 1
#define EITHERBUTTON 2
#define mousereset 0
#define mouseshow 1
#define mousehide 2
#define getmousestatus 3
#define setmousecoord 4
#define checkbuttonpress 5
#define checkbuttonrelease 6
#define setmousehorizrange 7
#define setmousevertrange 8
#define setgraphicscursor 9
#define mousemovement 15
#define conditionaloff 16
#define setmickeyratio 20

#define storagereqments 21  /* used 21,22,23 together */
#define savemousestate 22
#define restoremousestate 23

#define softwarereset 33

#define curswidth 16
#define cursheight 16
extern void mouse(int*,int*,int*,int*);
extern void hidemouse(void);
extern void showmouse(void);
extern boolean testbutton(int,int);
extern boolean resetmouse(void);
extern void movemouse(int,int);
extern boolean initmouse(void);
extern void getmousecoords(int*,int*);
extern void getmousemovement(int*,int*);
extern boolean mousebuttonreleased(int);
extern boolean mousebuttonpressed(int);
extern boolean mouseinbox(int,int,int,int,int,int);
extern int getinput(int);
extern void crosshaircursor(void);
extern void arrowcursor(void);
extern void setmouserange(int,int,int,int);
extern void mousesoftwarereset(void);
