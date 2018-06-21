#define SAVE 1
#define LOAD 2
#define RETURN  0xd
#define DELETE  0x8
#define DRIVE_NOT_READY 2
#define HARDERR_ABORT 2

char *file_error[] ={"UNABLE TO OPEN FILE!!",
			 "ERROR IN WRITING TO FILE",
			 "ERROR IN READING FROM FILE",
			 "NOT ENOUGH DISK SPACE",
			 "DISK NOT RESPONDING",
			 "NOT A FLIGHT_SIMULATION FILE!"
			};

char *file_options[]={"L   LOAD GAME",
					  "S   SAVE GAME",
					  "Q   QUIT GAME",
					  "ESC QUIT MENU"
					 };

extern void file_menu(nav,plane);
extern char *get_dir(int);
extern void getfileinfo(char**,int *);
extern int menu(int,int,char**,int);
extern void show_dir(char**,int count);
extern void drawbox(int,int,int,int);
extern int getfilename(int);
extern void draw_select_box(int);
extern void edit_pathname(void);
extern char*edit_filename(char*);
extern void edit_string(char*,int,int,int);
extern int update_path(char*);
extern void save(char*,nav,plane);
extern void load(char*,nav,plane);
extern void display_error(char*);
extern void location_menu(nav,airport*,int);
extern int hardware_error=0;
extern void delete_filenames(char**,int);
