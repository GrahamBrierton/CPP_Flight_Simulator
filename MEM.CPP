/*===========================================================================
   This module conatins all functions pertaining to allocating memory to the
different data structures in the application.
===========================================================================*/
#include <alloc.h>
#include <process.h>
#include <stdio.h>
#include "objects.h"
#include "mem.h"

/*============================================================================
							ALLOCATE_EXTENT_MEM
==============================================================================
============================================================================*/
extent *allocate_extent_mem(int no_of_extents)
{
  int count;
  extent *extents;

  /*ALLOCATE MEMORY TO ARRAY OF POINTERS TO STRUCTURES*/
  extents=(extent*)malloc(no_of_extents*sizeof(extent));

  /*ALLOCATE MEMORY TO ARRAY OF STRUCTURES*/
  for(count=0;count<no_of_extents;count++)
	 {
	  extents[count]=extent_3d();
	  if(extents[count]==NULL)
	   {
	exit(0);
	printf("MEMORY ALLOCATION ERROR");
	   }
	 }
   return(extents);
}

/*============================================================================
							ALLOCATE_ENVIRONMENT_MEM
==============================================================================
============================================================================*/
environment allocate_environment_mem()
{
 return (environment)malloc(sizeof(ENVIRON));
}

/*============================================================================
							ALLOCATE_AIRPORT_MEM
==============================================================================
============================================================================*/
airport* allocate_airport_mem(int no_of_airports)
{
 int count;
 airport *airports;

 /*ALLOCATE MEMORY TO ARRAY OF POINTERS TO STRUCTURES*/
 airports=(airport*)malloc(no_of_airports*sizeof(airport));

 /*ALLOCATE MEMORY TO ARRAY OF STRUCTURES*/
 for(count=0;count<no_of_airports;count++)
	{
	 airports[count]=airport_mem();
	 if(airports[count]==NULL)
	  {
	   exit(0);
	   printf("MEMORY ALLOCATION ERROR");
	  }
	}
 return(airports);
}
/*============================================================================
							ALLOCATE_SHAPE_MEM
==============================================================================
============================================================================*/
shape *allocate_shape_mem(int no_of_shapes)
{
 int count;
 shape *shapes;

 /*ALLOCATE MEMORY TO ARRAY OF POINTERS TO STRUCTURES*/
 shapes=(shape*)malloc(no_of_shapes*sizeof(shape));

 /*ALLOCATE MEMORY TO ARRAY OF STRUCTURES*/
 for(count=0;count<no_of_shapes;count++)
   {
	shapes[count]=shape_3d();
	if(shapes[count]==NULL)
	  {
	   exit(0);
	   printf("MEMORY ALLOCATION ERROR");
	  }
	shapes[count]->shape_mat=mat3d();
   }

 return shapes;
}

/*============================================================================
							MAT3D
==============================================================================
============================================================================*/
matrix mat3d()
{
 return (matrix)malloc(sizeof(MATRIX));
}


/*============================================================================
								ALLOCATE_FACE_MEM
==============================================================================
============================================================================*/
facelist *allocate_face_mem(int no_of_faces)
{
 int count;
 facelist *faces;

 /*ALLOCATE MEMORY TO ARRAY OF POINTERS TO STRUCTURES*/
 faces=(facelist*)malloc(no_of_faces*sizeof(facelist));

 /*ALLOCATE MEMORY TO ARRAY OF STRUCTURES*/
 for(count=0;count<no_of_faces;count++)
   {
	if((faces[count]=face_3d())==NULL)
	  {
	   printf("MEMOER ALLOCATION ERROR - EXITING ");
	   exit(0);
	  }
   }

 return faces;
}


/*============================================================================
							ALLOCATE_VERT_MEM
==============================================================================
============================================================================*/
vertices *allocate_vert_mem(int no_of_verts)
{
 int count;
 vertices *verts;

 /*ALLOCATE MEMORY TO ARRAY OF POINTERS TO STRUCTURES*/
 verts=(vertices*)malloc(no_of_verts*sizeof(vertices));

 /*ALLOCATE MEMORY TO ARRAY OF STRUCTURES*/
 for(count=0;count<no_of_verts;count++)
   {
	verts[count]=vert_3d();
   }

 return (vertices*)verts;
}


/*============================================================================
								AIRPORT_MEM
==============================================================================
============================================================================*/
airport airport_mem()
{
 return (airport)malloc(sizeof(AIRPORT));
}

/*============================================================================
								SHAPE_3D
==============================================================================
============================================================================*/
shape shape_3d()
{
 return (shape)malloc(sizeof(SHAPE));
}

/*============================================================================
								FACE_3D
==============================================================================
============================================================================*/
facelist face_3d()
{
 return (facelist)malloc(sizeof(FACE));
}


/*============================================================================
									VERT_3D
==============================================================================
============================================================================*/
vertices vert_3d()
{
  return (vertices)malloc(sizeof(VERTEX));
}

/*============================================================================
									EXTENT_3D
==============================================================================
============================================================================*/
extent extent_3d()
{
 return (extent)malloc(sizeof(EXTENT));
}

/*============================================================================
							ALLOCATE_NAV_MEM
==============================================================================
============================================================================*/
nav allocate_nav_mem()
{
  return (nav)malloc(sizeof(NAV));
}

/*============================================================================
							ALLOCATE_PLANE_MEM
==============================================================================
============================================================================*/
plane allocate_plane_mem()
{
  return (plane)malloc(sizeof(PLANE));
}
