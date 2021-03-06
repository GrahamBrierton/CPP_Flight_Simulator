#ifndef CLIPPED
#define CLIPPED 1
#endif

#ifndef NOT_CLIPPED
#define NOT_CLIPPED 0
#endif

#ifndef LEFT_CLIP
#define LEFT_CLIP 1
#endif

#ifndef RIGHT_CLIP
#define RIGHT_CLIP 2
#endif

#ifndef BOTTOM_CLIP
#define BOTTOM_CLIP 3
#endif

#ifndef TOP_CLIP
#define TOP_CLIP 4
#endif

#ifndef MAXX
#define MAXX 639
#endif

#ifndef MAXY
#define MAXY 349
#endif

#ifndef EXTENT_FILL
#define EXTENT_FILL 3
#endif

#ifndef SOLID
#define SOLID 1
#endif

#ifndef FRONT_Z
#define FRONT_Z 20.0
#endif


struct intersect_def
{
 float x_coord;
 struct intersect_def *next_intersect;
};
typedef struct intersect_def INTERSECT;
typedef INTERSECT *intersect;

struct scanline_def
{
 struct scanline_def *next_scan;
 float scanline_no;
 intersect x_value;
};

typedef struct scanline_def SCANLINE;
typedef SCANLINE *scanline;

extern void save_screen(void far *buf[4]);
extern void restore_screen(void far *buf[4]);
extern int  clip(float*,float*,float*,float,float,float);
extern int  clip_planes(float*,float*,float*,float*,float*,float*,int,int);
extern void add_xval(scanline,float);
extern void mypoly(float *,int,int,int);
extern void draw_scanlines(scanline,int);
extern void delete_scanlines(scanline);
extern void polyclip(float*,int*,int*,int);
extern void line_intersect(float,float,float,float,float*,float*,int*,int);
extern int  inside(float,float,int,int*);
extern hsub1(int,char);
extern hsub2(int,char,char);
extern hsub3(int,char,char,int);
extern void esetres(char);
extern void setres(char);
extern void bitmask(char);
extern void mapmask(char);
extern void hline(int,int,int,int,int);
extern int *calculate_outcode(float,float ,float);
extern intersect new_intersect(void);
extern scanline new_scanline(void);
