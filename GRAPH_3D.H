#define WINWIDTH 1
#define WINHEIGHT 1
#define SCREENWIDTH 640
#define SCREENHEIGHT 440
#define LEFT_CLIP 1
#define RIGHT_CLIP 2
#define BOTTOM_CLIP 3
#define TOP_CLIP 4
#define RETURN 7181

void init_shape(shape,float,float,float,int,float[][3],int,float[][2],int);
void initialise_matrix(matrix);
void mtranslate_3d(matrix,float,float,float);
void bounding_box(shape);
shape shape_3d(void);
void z_clip(float*,float*,float*,float,float,float);
void transform_point(matrix,float*,float*,float*);
void set_perspective(float,float,float);
void set_view_reference_point(float,float,float);
void set_view_up(float,float,float);
void mrotx3d(matrix,float,float);
void mroty3d(matrix,float,float);
void mrotz3d(matrix,float,float);
void bank(float);
void transform_projection_point(matrix);
void convert_3d_to_2d(int,float,float,float,float,float,float);
matrix trans_eye();
matrix mat3d();
void mroty3d(matrix,float,float);
void mrotz3d(matrix,float,float);
void create_view(shape,matrix);
shape *allocate_shape_mem(int);
void allocate_shapes(shape *,int,int,int);
int  random_select(int);
void scale(matrix,float);
int *calculate_outcode(float,float,float);
void clip_planes(float*,float*,float*,float*,float*,float*);
void update_transform(shape);




