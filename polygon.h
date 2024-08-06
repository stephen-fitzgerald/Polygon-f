/* 	
	FILE: polygon.h
	Polygon typedefs and prototypes for polygon.c
	12.9.92,	Stephen Fitzgerald                                    */

#ifndef MAX
#define MAX(A,B) ((A)>(B) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A,B) ((A)<(B) ? (A) : (B))
#endif

#ifndef NULL
#define NULL	((void *) 0)
#endif

#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

#define BAD_HANDLE	-1
#define BAD_POLY	0
#define OTHER_ERROR	0

/* # of vertices to allocate new space for in a polygon */
#define kSizeIncr	10

#define kPolygon	1
#define kRGoneScore 3
#define kForwards 1
#define kBackwards (-1)

/*==== typedefs ====*/

typedef long double f_type;
typedef	struct _fpoint { f_type x, y; } f_point;
typedef struct _vertex { f_type x, y; int status; } Vertex, *VertexPtr, **VertexHndl ;

typedef void *POINTER, **HANDLE ;

typedef struct _shape {

	int				type ;
	VertexHndl		coords ;
	int				size, max_size ;
	struct _shape	**next ;
	
} Shape, *ShapePtr, **ShapeHndl ;

typedef	ShapeHndl  PolyHndl ;

/*==== Prototypes ====*/
HANDLE		NewHandle( size_t );
void 		DisposeHandle( HANDLE);
PolyHndl	newPolygon( void );
void		freePolygon( PolyHndl P ) ;
int			numVertices( PolyHndl P ) ;
int			addVertex( PolyHndl P, f_type x, f_type y ) ;
int			insertVertex( PolyHndl P, f_type x, f_type y, int afterV ) ;
PolyHndl	copyPoly( PolyHndl P ) ;
f_type		_vertex_angle( Vertex pt1, Vertex pt2, Vertex pt3 );
f_type		vertexAngle( PolyHndl P, int v1, int v2, int v3 ) ;
int			fSign( f_type n ) ;
int			convexPoly( PolyHndl P ) ;
int			removeSubPoly( PolyHndl P, int start, int end, int dir, int score ) ;
f_type		cnvxPolyArea( PolyHndl P, int start, int end, int dir) ;
int			UpdateStatus( PolyHndl P ) ;
int			lineHitsSide(PolyHndl P, int start, int end) ;
int			linesIntersect( Vertex start1, Vertex end1, Vertex start2, Vertex end2 ) ;
int			nextVertex(PolyHndl P,int current, int dir) ;
int			nextActiveVertex(PolyHndl P,int current, int dir) ;
int			nextRCorner(PolyHndl P,int current, int dir) ;
int			isRCorner(PolyHndl P,int current) ;
int			scoreBarrier(PolyHndl P,int start, int end, int dir ) ;
f_type		PolyArea( PolyHndl P ) ;
