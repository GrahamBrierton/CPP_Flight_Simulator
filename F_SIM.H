#define WINWIDTH 1
#define WINHEIGHT 1
#define SCREENWIDTH 640
#define SCREENHEIGHT 200
#define RETURN 7181
#define GRAVITY 32
#define PLANE_WEIGHT 8000
#define WING_AREA 340
#define NEAR 0
#define FAR  1
#define AILERON_LEFT 7777
#define AILERON_RIGHT 8051
#define ACCELERATE 13358
#define DECELERATE 13100
#define TRUE 1
#define FALSE 0
#define FRONT_VIEW 15104
#define LEFT_VIEW 15360
#define RIGHT_VIEW 15616
#define REAR_VIEW 15872
#define GEAR 8807
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
void update_orientation(plane,float);
void load_world(int*,int*,int*,int*,int*,int*,FILE*);
vertices *load_verts(vertices*,int,FILE *);
facelist *load_faces(facelist*,int,FILE *);
shape *load_shapes(shape*,int,FILE *);
extent*load_extents(extent*,int,FILE*);
airport *load_airports(airport*,int,FILE*);
void load_beacons(nav,int,FILE*);
void transform_point(matrix,float*,float*,float*);
void create_view(matrix,vertices*,facelist*,shape*,extent*,int,int);
void set_perspective(float,float,float);
void set_view_reference_point(float,float,float);
void transform_projection_point(matrix);
void bank_plane(float,plane);
void move_plane(plane,nav);
void set_view_up(float,float,float,plane);
void draw_instruments(void);
void draw_sky(void);
void update_transform(shape);
void create_horizon(matrix,float,nav,int);
void initialize_nav(nav);
void initialize_environment(environment);
void initialize_thrust_required(plane);
void test_extents(extent*,int,matrix);
void change_flaps(plane,int);
void change_omi_radial(nav,int);
extern void menu_bar(nav,plane,airport*,int);
extern void displaymap(char*,int,int);
void crash(int,int,int,int,int);
void set_new_palette(void);
void check_for_crash(plane,nav);
void move_gear(plane);
void change_nav(nav,int);
void change_brakes(plane);
int invisible(float,float);
int test(float,float,float);
