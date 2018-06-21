#define UP     18432
#define LEFT   19200
#define RIGHT  19712
#define DOWN   20480
#define ACCELERATE 13358
#define D_TO_R 57.29575

/*==========================GLOBAL DECLARATIONS=============================*/
float xr=0.0,yr=0.0,zr=0.0;
float xc=0.0,yc=0.0,zc=0.0;
float xpcntr=0.0,ypcntr=0.0,zpcntr=0.0; /*CENTRE OF PROJECTION*/
float dxn=0.0,dyn=0.0,dzn=-1.0;     /*VIEW DIRECTION VECTOR*/
float dxup=0.0,dyup=1.0,dzup=0.0;   /*VIEW-UP VECTOR*/
float view_distance=5;              /*DISTANCE FROM VIEW-PLANE*/



/*VERTEX AND LINE DEFINITIONS FOR DIAMOND*/
float diamond[][3]={-5,0,0,
		    0,0,-5,
		    5,0,0,
		    0,0,5,
		    0,5,0,
		    0,-5,0};

float diamond_lines[][2]={0,1,
			  1,2,
			  2,3,
			  3,0,
			  0,4,
			  1,4,
			  2,4,
			  3,4,
			  0,5,
			  1,5,
			  2,5,
			  3,5};


/*VERTEX AND LINE DEFINITIONS FOR CUBE*/
float cube[][3]={-5,-5,-5,
		 5,-5,-5,
		 5,-5,5,
		 -5,-5,5,
		 -5,5,5,
		 5,5,5,
		 5,5,-5,
		 -5,5,-5};

float cube_lines[][2]={0,1,
		       1,2,
		       2,3,
		       3,0,
		       0,7,
		       1,6,
		       2,5,
		       3,4,
		       4,7,
		       7,6,
		       6,5,
		       5,4};


/*VERTEX AND LINE DEFINITIONS FOR PYRAMID*/
float pyramid[][3]={-5,0,0,
		    0,0,-5,
		    5,0,0,
		    0,0,5,
		    0,10,0};

float pyramid_lines[][2]={0,1,
			  1,2,
			  2,3,
			  3,0,
			  0,4,
			  1,4,
			  2,4,
			  3,4};




/*VERTEX STRUCTURE*/
struct vertex
{
 float co_ord[3];
};

typedef struct vertex VERTEX;
typedef VERTEX *point;

/*EDGE STRUCTURE*/
struct edge
{
 float vert[2];
};
typedef struct edge EDGE;
typedef EDGE *line_def;

/*MATRIX STRUCTURE*/
struct mat
{
 float entry[4][3];
} ;

typedef struct mat MATRIX;
typedef MATRIX *matrix;

/*OBJECT DEFINITION STRUCTURE*/
struct shape_def
{
 float vert_array[60][3];
 float line_array[60][3];
 int no_of_verts;
 int no_of_lines;
 matrix shape_mat;
 int scale_factor;
 float x_dir,y_dir,z_dir;
 float y_rot,x_rot,z_rot;
 float curr_y_angle,curr_z_angle,curr_x_angle;
 float wx,wy,wz;
 int color;
};

typedef struct shape_def SHAPE;
typedef SHAPE *shape;

