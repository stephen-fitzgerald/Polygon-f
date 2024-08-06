/* 	
	FILE: polygon.c

	Polygon functions for simple area calculations.

	started 12.5.92, finished version 1.0 ?
	
	Stephen Fitzgerald
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "polygon.h"

#define DBUG(A) {}

/*---------------------------------------------------------------------------
	Dumb-ass memory manager
----------------------------------------------------------------------------*/
HANDLE NewHandle(size_t size) {
    // Allocate memory for the handle (a pointer to a pointer)
    HANDLE handle = (HANDLE)malloc(sizeof(void*));
    if (handle == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        return NULL;
    }

    // Allocate memory for the actual data
    *handle = malloc(size);
    if (*handle == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        free(handle);
        return NULL;
    }

    return handle;
}

void DisposeHandle( HANDLE hdl ){
	free(*hdl);
	free(hdl);
}

void SetHandleSize(HANDLE h, size_t newSize) {
    if (h == NULL || *h == NULL) {
        fprintf(stderr, "Invalid handle\n");
        return;
    }

    // Reallocate memory for the new size
    void* newPtr = realloc(*h, newSize);
    if (newPtr == NULL) {
        fprintf(stderr, "Memory reallocation failed\n");
        return;
    }

    *h = newPtr;
}
/*---------------------------------------------------------------------------
	newPolygon() creates a new polygon that initialy has a maximum of 
	'kSizeIncr' vertices.
----------------------------------------------------------------------------*/

PolyHndl	newPolygon( void ) {

	PolyHndl		P = NULL;
	int				i = 0;
		
	P = (ShapeHndl) NewHandle( sizeof(Shape) );
	
	if ( P==NULL || (*P)==NULL ) {
		return( NULL ) ;
	}
	
	(*P)->coords = (VertexHndl) NewHandle(	kSizeIncr * (sizeof(Vertex)+1) );
	
	if ( (*P)->coords == NULL || *((*P)->coords) == NULL ) {
		DisposeHandle( (HANDLE) P ) ;
		return(NULL) ;
	}
	
	(*P)->type = kPolygon ;
	(*P)->size = 0 ;
	(*P)->max_size = kSizeIncr ;
	(*P)->next = NULL ;

	for( i=1; i<=kSizeIncr; i++ ) {
		(*((*P)->coords))[i].x = (f_type) 0.0 ;
		(*((*P)->coords))[i].y = (f_type) 0.0 ;
		(*((*P)->coords))[i].status = (int) 0 ;
	}

	return( P );
}

/*---------------------------------------------------------------------------
	freePolygon() frees the memory used by a polygon.
----------------------------------------------------------------------------*/

void	freePolygon( PolyHndl P ) {

	if ( P!=NULL ) {
		if ( (*P)->coords != NULL  ) {
			DisposeHandle( (HANDLE) (*P)->coords ) ;
		}
		DisposeHandle( (HANDLE) P ) ;
	}
}


/*---------------------------------------------------------------------------
	numVertices() returns the number of vertices in a polygon, or BAD_HANDLE
	if there's an error.
----------------------------------------------------------------------------*/

int	numVertices( PolyHndl P ) {
	
	if ( P==NULL || (*P)==NULL ) {
		return( BAD_HANDLE ) ;
	} 
	
	return( (*P)->size ) ;
}

/*---------------------------------------------------------------------------
	addVertex() adds a new vertex to the polygon, P,  passed as a handle.
	Returns number of vertices now in polygon, or BAD_HANDLE if there's an
	error.
----------------------------------------------------------------------------*/

int	addVertex( PolyHndl P, f_type x, f_type y ) {

	int next, max ;
	
	if ( P==NULL || (*P)==NULL ) {
		return( BAD_HANDLE ) ;
	}
	
	if ( ((*P)->coords)==NULL || (*((*P)->coords))==NULL ) {
		return( BAD_HANDLE ) ;
	}
	
	next = (*P)->size + 1 ;
	max  = (*P)->max_size ;
	
	/* create new vertex slots if needed */
	
	if ( next > max ) {
	
		max = max + kSizeIncr;
		SetHandleSize( (HANDLE) ((*P)->coords), (max+1)*sizeof(Vertex) );
	}
	
	(*P)->size = next ;
	(*P)->max_size = max ;

	(*((*P)->coords))[next].x = x ;
	(*((*P)->coords))[next].y = y ;
	(*((*P)->coords))[next].status = 0 ;

	return( next );
}

/*---------------------------------------------------------------------------
	insertVertex() inserts a new vertex into the polygon, P, after vertex 
	afterV.
	Returns number of vertices now in polygon, or BAD_HANDLE if there's an
	error.
----------------------------------------------------------------------------*/

int	insertVertex( PolyHndl P, f_type x, f_type y, int afterV ) {

	int i, next, max ;
	
	if ( P==NULL || (*P)==NULL ) {
		return( BAD_HANDLE ) ;
	}
	
	if ( ((*P)->coords)==NULL || (*((*P)->coords))==NULL ) {
		return( BAD_HANDLE ) ;
	}
	
	next = (*P)->size + 1 ;
	max  = (*P)->max_size ;
	
	if( afterV >= next ) { return( addVertex( P, x, y ) ) ; }
	
	/* create new vertex slots if needed */
	
	if ( next > max ) {
		max = max + kSizeIncr;
		SetHandleSize( (HANDLE) ((*P)->coords), (max+1)*sizeof(Vertex) );
	}
	
	(*P)->size = next ;
	(*P)->max_size = max ;
	
	for( i=afterV+1; i<next; i++ ) {
		(*((*P)->coords))[i+1].x = (*((*P)->coords))[i].x ;
		(*((*P)->coords))[i+1].y = (*((*P)->coords))[i].x ;
	}
	
	(*((*P)->coords))[afterV].x = x ;
	(*((*P)->coords))[afterV].y = y ;
	(*((*P)->coords))[afterV].status = 0 ;
		
	return( next );
}

/*---------------------------------------------------------------------------
	copyPoly() creates a copy of P and returns a handle to it.
----------------------------------------------------------------------------*/
PolyHndl	copyPoly( PolyHndl P ) {
	
	PolyHndl	temp ;
	int			N, i ;
	
	temp = newPolygon() ;
	
	if( temp==NULL || (*temp)==NULL || P==NULL || (*P)==NULL ) {
		return( NULL ) ;
	}
	
	/* Get # of vertices */
	N = numVertices( P ) ;
	
	/* check for empty polygon */
	if ( N <= 0 ) { 
		freePolygon(temp ) ;
		return( NULL ) ; 
	}
		
	for ( i=1; i<=N; i++ ) {		
		addVertex( temp, (*((*P)->coords))[i].x, (*((*P)->coords))[i].y ) ;
	}
	
	return( temp ) ;

}

/*---------------------------------------------------------------------------
	_vertex_angle() calculates the angle in degrees between vectors 
	1-2 and 2-3. Interior is on the right when facing 2 from 1.
----------------------------------------------------------------------------*/

f_type	_vertex_angle( Vertex pt1, Vertex pt2, Vertex pt3 ) {

	f_type	theta ;
	double	crossAbs, dot, x1x2, y3y2, x3x2, y1y2;
	
	x1x2 = (double) ((double)pt2.x - (double)pt1.x);
	x3x2 = (double) ((double)pt3.x - (double)pt2.x);
	y1y2 = (double) ((double)pt2.y - (double)pt1.y);
	y3y2 = (double) ((double)pt3.y - (double)pt2.y);
	
	crossAbs = x1x2 * y3y2 - x3x2 * y1y2 ;
	
	dot = x1x2 * x3x2 + y3y2 * y1y2 ;
	
	if( fabs(dot)<1e-12 ) dot=0.0;
	
	theta = (f_type) (atan2( crossAbs, dot ) * 57.29577951 );

	return( 180-theta );
}

/*---------------------------------------------------------------------------
	vertexAngle() returns the inside angle at vertex v2. 
----------------------------------------------------------------------------*/

f_type	vertexAngle( PolyHndl P, int v1, int v2, int v3 ) {

	f_type 		theta ;
	Vertex 		pt1, pt2, pt3 ;
	int			N ;
		
	if ( P==NULL || (*P)==NULL ) { 
		return( BAD_HANDLE ) ;
	} 

	pt1 = (*((*P)->coords))[v1]; 
	
	pt2 = (*((*P)->coords))[v2] ;
	
	pt3 = (*((*P)->coords))[v3]; 

	theta = _vertex_angle( pt1, pt2, pt3 );
	
	return( theta );
}


/*---------------------------------------------------------------------------
	fSign(n) returns 1 if n>0.0, -1 if n<0.0, and 0 if n==0.0 
----------------------------------------------------------------------------*/
int	fSign( f_type n ) {

	if( n<0.0 ) {
		return(-1) ;
	} else if( n>0.0 ) {
		return(1) ;
	} else {
		return(0) ;
	}
}

/*---------------------------------------------------------------------------
	convexPoly() returns 1 if the polygon is convex, 0 if not
----------------------------------------------------------------------------*/

int	convexPoly( PolyHndl P ) {

	int		i, N ;
	int		nx, ny ;
	int		x_dir, y_dir, new_x_dir, new_y_dir ;
		
	/* Check for valid Handle & Pointer */
	if( P==NULL || (*P)==NULL ) { return(BAD_HANDLE) ; }
	
	/* Get # of vertices */
	N = numVertices( P ) ;
	
	/* check for empty polygon */
	if ( N < 3 ) { return( BAD_POLY ) ; }
	
	/* Triangles are always convex */
	if ( N == 3 ) { return( TRUE ) ; }
	
	/* Get initial direction of polygon edge */
	x_dir = fSign( (*((*P)->coords))[1].x - (*((*P)->coords))[N].x ) ;
	y_dir = fSign( (*((*P)->coords))[1].y - (*((*P)->coords))[N].y ) ;
	
	/* nx & ny are number of sign changes in dx & dy */
	nx = ny = 0 ;
	
	/* loop through edges looking for direction changes */
	
	for ( i=1; i<N; i++ ) {
		new_x_dir = fSign( (*((*P)->coords))[i+1].x - (*((*P)->coords))[i].x ) ;
		new_y_dir = fSign( (*((*P)->coords))[i+1].y - (*((*P)->coords))[i].y ) ;
		
		if( new_x_dir * x_dir == -1 ) nx++ ;
		if( new_y_dir * y_dir == -1 ) ny++ ;
		
		if( new_x_dir != 0 ) 
			x_dir = new_x_dir ;		
		if( new_y_dir != 0 ) 
			y_dir = new_y_dir ;
	}
	
	/* if x and y travel changes sign less than thrice polygon is convex */
	if( nx<=2 && ny<=2 ) { 
		return(TRUE) ; 
	} else { 
		return(FALSE) ; 
	} 
}
/*-----------------------------------------------------------------------------
	removeSubPoly() deactivates all the active vertices between start and end.
	start is set to convex if score = 3 or 7, end is set to convex if score
	= 5 or 7.
------------------------------------------------------------------------------*/

int	removeSubPoly( PolyHndl P, int start, int end, int dir, int score ) {

	int			i, ip1, N ;
	f_type		xi, yi, xip1, yip1 ;
	f_type		area = 0.0 ;
	
	if( P==NULL || (*P)==NULL ) { return(BAD_HANDLE) ; }
		
	if( score==3 || score==7 ) {
		(*((*P)->coords))[start].status = 1 ;
	} 
	
	if( score==5 || score==7 ) {
		(*((*P)->coords))[end].status = 1 ;
	} 
	
	i = start ;
	
	while( (i=nextActiveVertex(P, i, dir)) != end ) {
		(*((*P)->coords))[i].status = 0 ;
	}
	
	return( 1 );
}

/*---------------------------------------------------------------------------
	cnvxPolyArea() calculates the area of a convex sub-polygon of P.
	The sub-poly starts at start and ends at end, traveling in direction
	dir. Only active vertices are considered.
----------------------------------------------------------------------------*/

f_type	cnvxPolyArea( PolyHndl P, int start, int end, int dir) {

	int			i, ip1, N ;
	f_type		xi, yi, xip1, yip1 ;
	f_type		area = 0.0 ;
	
	if( P==NULL || (*P)==NULL ) { return(BAD_HANDLE) ; }
	
	xi = (*((*P)->coords))[end].x ;
	yi = (*((*P)->coords))[end].y ;
	xip1 = (*((*P)->coords))[start].x ;
	yip1 = (*((*P)->coords))[start].y ;
	
	area = ( xi * yip1 - xip1 * yi ) ;
	
	i = ip1 = start ;
	
	while( ip1 != end ) {
		i = ip1 ;
		ip1 = nextActiveVertex( P, i, dir ) ;
		
		xi = (*((*P)->coords))[i].x ;
		yi = (*((*P)->coords))[i].y ;
		xip1 = (*((*P)->coords))[ip1].x ;
		yip1 = (*((*P)->coords))[ip1].y ;
		
		area += ( xi * yip1 - xip1 * yi ) ;
		
	} ;
	
	if ( area < 0.0 ) {
		area *= -1.0 ;
	}

	area = area / 2.0 ;
	DBUG( printf("\n cnvxPolyArea() -> start=%d, end=%d, dir=%d, area=%g ",start,end,dir,area);)
	return( area );
}

/*---------------------------------------------------------------------------
	UpdateStatus() updates the status field for each vertex in P.
	0=inactive, 1=convex, 2=R-corner. 
----------------------------------------------------------------------------*/
int	UpdateStatus( PolyHndl P ) {

	int				prev, next, i, N ;
	f_type			angle ;
	
	N = numVertices( P ) ;

	/* Mark R corners with 2 and convex corners with 1 */

	for( i=1; i<=N; i++ ) {
	
		next = nextVertex(P,i,kForwards  ) ;
		prev = nextVertex(P,i,kBackwards ) ;
		
		if( (angle = vertexAngle( P, prev, i, next )) <= (f_type)180.0 ) {
			(*((*P)->coords))[i].status = 1 ;
		}
		else {
			(*((*P)->coords))[i].status = 2 ;
		}
	}
	
	return(N) ;
}

/*---------------------------------------------------------------------------
	lineHitsSide() returns non-zero if a line segment from start to end 
	would intersect any segment of the polygon P.
----------------------------------------------------------------------------*/
int	lineHitsSide(PolyHndl P, int start, int end) {

	int				next, i, N, found ;
	Vertex			L1_start, L1_end, L2_start, L2_end ;
	
	N = numVertices( P ) ;
	
	L1_start = (*((*P)->coords))[start] ;
	L1_end   = (*((*P)->coords))[end] ;
	found = 0 ;
	
	/*-------------------------------------------------------------------
		Check every side, unless it shares a vertex with the given line
	---------------------------------------------------------------------*/
	for( i=1; i<=N && !found ; i++ ) {
	
		next = nextVertex(P,i,kForwards  ) ;
		
		if( i!=start && i!=end && next!=start && next!=end ) {
		
			L2_start = (*((*P)->coords))[i] ;
			L2_end   = (*((*P)->coords))[next] ;
			
			if( linesIntersect( L1_start, L1_end , L2_start, L2_end ) ) {
				found = 1 ;
			}
		}
	}
	
	return(found) ;
}

/*---------------------------------------------------------------------------
	linesIntersect() returns non-zero if two line segments intersect.
----------------------------------------------------------------------------*/
int	linesIntersect( Vertex start1, Vertex end1, Vertex start2, Vertex end2 ) {

	f_type	x1max, x1min, y1max, y1min ;
	f_type	dx1, dy1, slope1, intercept1 ;
	f_type	x2max, x2min, y2max, y2min ;
	f_type	dx2, dy2, slope2, intercept2 ;
	f_type	xint ;
	double	kEps = 1e-12, kBigNum = 1e99 ;

	x1max = MAX( start1.x, end1.x ) ;
	x1min = MIN( start1.x, end1.x ) ;
	y1max = MAX( start1.y, end1.y ) ;
	y1min = MIN( start1.y, end1.y ) ;
	
	x2max = MAX( start2.x, end2.x ) ;
	x2min = MIN( start2.x, end2.x ) ;
	y2max = MAX( start2.y, end2.y ) ;
	y2min = MIN( start2.y, end2.y ) ;
	
	/*---------------------------------------------------------------------
		If the boxes enclosing the lines do not intersect, then the lines
		don't either.
	---------------------------------------------------------------------*/
	if( x1max<x2min || x1min>x2max || y1max<y2min || y1min>y2max ) {
		return( 0 ) ;
	}
	
	dx1 = end1.x - start1.x ;
	dy1 = end1.y - start1.y ;
	
	dx2 = end2.x - start2.x ;
	dy2 = end2.y - start2.y ;
	
	/*---------------------------------------------------------------------
		If the boxes enclosing the lines intersect, and both lines are
		horizontal or vertical then the lines intersect.
	---------------------------------------------------------------------*/
	if( (fabs((double)dx1)<kEps || fabs((double)dy1)<kEps) && (fabs((double)dx2)<kEps || fabs((double)dy2)<kEps) ) {
		return( 1 ) ;
	}
	
	if( fabs(dx1)<kEps ) {
		slope1 = kBigNum ;
	} else {
		slope1 = dy1 / dx1 ;
	}
	
	if( fabs(dx2)<kEps ) {
		slope2 = kBigNum ;
	} else {
		slope2 = dy2 / dx2 ;
	}
	
	if( fabs((double)(slope1-slope2))<kEps ) {
		/*---------------------------------------------------------------
			If the boxes enclosing the lines intersect, and lines are
			parallel, and they have same y-intercepts, then the lines 
			intersect.
		-----------------------------------------------------------------*/
		if( fabs((double)(intercept1-intercept2))<kEps ) {
			return(1) ;
		} else {
			return(0) ;
		}
	} else {
		/*---------------------------------------------------------------
			If the boxes enclosing the lines intersect, and lines are
			not parallel, then determine the point that they cross.  If
			that point is the range of both lines then we have an 
			intersection.
		-----------------------------------------------------------------*/
		xint = (intercept2-intercept1)/(slope1-slope2) ;
		if( x1max>=xint && x1min<=xint && x2max>=xint && x2min<=xint ) {
			return(1) ;
		} else {
			return(0) ;
		}
	}
	
	/*------------------------------------------------------------------
		The program should not make it to here. If it does, return 2
		so the error can be detected.
	-------------------------------------------------------------------*/
	return(2);
}


/*---------------------------------------------------------------------------
	nextVertex() returns the index of the next vertex in given direction. 
----------------------------------------------------------------------------*/
int	nextVertex(PolyHndl P,int current, int dir) {

	int		N, ret ;
	
	N = numVertices( P ) ;
	
	ret = current + dir ;
	
	if( ret > N ) ret = 1 ;
	if( ret < 1 ) ret = N ;
		
	return( ret ) ;
}

/*---------------------------------------------------------------------------
	nextActiveVertex() finds the next active corner. Returns 0 if none.
----------------------------------------------------------------------------*/
int	nextActiveVertex(PolyHndl P,int curr, int dir) {

	int				i, ret, current ;
	
	current = curr ;
	if( current<1 || current>numVertices( P ) ) current=1 ;
	
	ret = 0 ;
	i = nextVertex( P, current, dir) ;
	
	while( 1 ) {
		if( (*((*P)->coords))[i].status > 0 ) {
			ret = i ;
			break ;
		}
		else {
			i = nextVertex( P, i, dir) ;
			if( i==current || i==0 ) break ;
		}		
	}
	
	return( ret ) ;
}

/*---------------------------------------------------------------------------
	nextRCorner() finds the next R-corner. Returns 0 if none.
----------------------------------------------------------------------------*/
int	nextRCorner(PolyHndl P,int curr, int dir) {

	int				i, N, ret, current ;
	
	N = numVertices( P ) ;
	
	current = curr ;
	if( current<1 || current>N ) current=1 ;
	
	ret = 0 ;
	i = current ;
	
	while( (i = nextActiveVertex(P, i, dir)) ) {
		if( (*((*P)->coords))[i].status == 2 ) {
			ret = i ;
			break ;
		}
		else {
			if( i==current || i==0 ) break ;
		}		
	}
	
	return( ret ) ;
}

/*---------------------------------------------------------------------------
	isRCorner() returns non-zero if current is R-corner.
----------------------------------------------------------------------------*/
int	isRCorner(PolyHndl P,int current) {

	int				ret ;
	
	if( current<1 || current>numVertices( P ) ) return(0) ;
	
	ret = ( (*((*P)->coords))[current].status == 2 ) ;

	return( ret ) ;
}


/*---------------------------------------------------------------------------
	scoreBarrier() returns non-zero if barrier from start to end is valid.
	It returns :
				0 : Barrier is invalid
				1 : Barrier is valid
				3 : Barrier is valid, and removes the R-corner at start.
				5 : Barrier is valid, and removes an R-Corner at end.
				7 : Barrier is valid, and removes 2 R-corners.	
---------------------------------------------------------------------------*/
int	scoreBarrier(PolyHndl P,int start, int end, int dir ) {

	int				ret;
	int 			beforeStart, afterStart, beforeEnd, afterEnd ;
	f_type			alpha1, alpha2, beta1, beta2 ;
	static f_type	one80 = (f_type)180.0 ;

	DBUG( printf("\n scoreBarrier() -> start=%d, end=%d, dir=%d ", start, end, dir) ; )

	if( start==end  ) { return( 0 ) ; }
		
	beforeStart	= nextActiveVertex( P, start, kBackwards ) ;	
	afterStart	= nextActiveVertex( P, start, kForwards    ) ;
	beforeEnd	= nextActiveVertex( P, end,   kBackwards ) ;
	afterEnd	= nextActiveVertex( P, end,   kForwards    ) ;

	if( end==afterStart || end==beforeStart || start==beforeEnd || start==afterEnd ) {
		return( 0 ) ;
	}

	if( dir == kForwards ) alpha1 = vertexAngle(P, end, start, afterStart) ;
	else if( dir == kBackwards ) alpha1 = vertexAngle(P, beforeStart, start, end) ;
	if( alpha1 > one80 ) { DBUG(printf(" alpha1>180 ");) return(0) ; }
	
	if( dir == kForwards ) alpha2 = vertexAngle(P, beforeEnd, end, start) ;
	else if( dir == kBackwards ) alpha2 = vertexAngle(P, start, end, afterEnd) ;
	if( alpha2 > one80 ) { DBUG(printf(" alpha1>180 ");) return(0) ; }

	beta1 = vertexAngle(P, beforeStart, start, afterStart) ;
	if( beta1<=alpha1 ) { DBUG(printf(" beta1<=alpha1 ");) return(0) ; }

	beta2 = vertexAngle(P, beforeEnd, end, afterEnd) ;
	if( beta2<=alpha2 ) { DBUG(printf(" beta2<=alpha2 ");) return(0) ; }	

	if( lineHitsSide(P, start, end) ) { DBUG(printf("Hit Line");) return(0) ; }
	
	ret = 1 ;
	
	if( beta1-alpha1 <= one80 ) { 
		ret += 2 ; 
	}
	
	if( beta2 > one80 && beta2-alpha2 <= one80 ) { 
		ret += 4 ; 
	}
	
	DBUG( printf(" score=%d", ret ) ; )

	return( ret ) ;
}


/*---------------------------------------------------------------------------
	PolyArea() calculates the area of a simple, but not necessarily
	convex, polygon.
----------------------------------------------------------------------------*/

f_type	PolyArea( PolyHndl P ) {

	int				bStart, bEnd, currentDir;
	int				bestStart, bestEnd, bestScore, bestDir ;
	int				thisScore ;
	f_type			area = 0.0, subPolyArea = 0.0 ;
	
	area = 0.0 ;
	bStart = 1 ;
	
	/*-------------------------------------------------------------
		Update the status field of each vertex in the polygon.
		This is where we determine which vertices are R-Corners.
	---------------------------------------------------------------*/
	UpdateStatus( P ) ;
	
	/*-------------------------------------------------------------
		Eliminate all R-Corners in the polygon by finding valid
		barriers, and removing the convex sub-polygons they bound.
	---------------------------------------------------------------*/
	while( (bStart = nextRCorner(P,bStart,kForwards)) ) {
		
		bestStart = bestEnd = bestScore = bestDir = 0 ;
		thisScore = 0 ;
		
		/*--------------------------------------------------------
			Search for a valid barrier from bStart to any 
			active Vertex, bEnd.
		----------------------------------------------------------*/
		currentDir = kForwards ;
		bEnd = nextActiveVertex( P, bStart, currentDir) ;
		
		/*-----------------------------------------------------------
			Search forwards for a valid barrier.
		------------------------------------------------------------*/
		while( ! isRCorner(P,bEnd) && bestScore<kRGoneScore ) {
		
			bEnd = nextActiveVertex( P, bEnd, currentDir) ;
			
			thisScore = scoreBarrier(P,bStart,bEnd,currentDir) ;
			
			/*----------------------------------------------------
				If this is the best barrier so far, save it.
			-----------------------------------------------------*/
			if( thisScore > bestScore ) {
				bestScore = thisScore ;
				bestStart = bStart ;
				bestEnd = bEnd ;
				bestDir = currentDir ;
			}
						
		} /* while() - End of forwards barrier search for current R-corner  */
		
		/*-----------------------------------------------------------
			Search backwards for a valid barrier.
		------------------------------------------------------------*/
		currentDir = kBackwards ;
		bEnd = nextActiveVertex( P, bStart, currentDir ) ;
		
		while( ! isRCorner(P,bEnd) && bestScore<kRGoneScore ) {
		
			bEnd = nextActiveVertex( P, bEnd, currentDir) ;
			
			thisScore = scoreBarrier(P,bStart,bEnd,currentDir) ;
			
			/*----------------------------------------------------
				If this is the best barrier so far, save it.
			-----------------------------------------------------*/
			if( thisScore > bestScore ) {
				bestScore = thisScore ;
				bestStart = bStart ;
				bestEnd = bEnd ;
				bestDir = currentDir ;
			}
			
		} /* while() - End of backwards barrier search for current R-corner  */
		
		/*---------------------------------------------------------
			If a barrier is found add the sub-polygon area to
			our total, and remove the sub-polygon.
		-----------------------------------------------------------*/			
		if( bestScore ) {
			subPolyArea = cnvxPolyArea( P, bestStart, bestEnd, bestDir ) ;
			area += subPolyArea ;
			removeSubPoly( P, bestStart, bestEnd, bestDir, bestScore ) ;
DBUG( printf("\n PolyArea() -> bestScore=%d, bestStart=%d, bestEnd=%d, area=%g ",bestScore,bestStart,bestEnd,subPolyArea);)
		}
		
	}  /* while() - All R-corners have been delt with.  */
	
	/*---------------------------------------------------------------
		The remaining vertices form a convex polygon.
	----------------------------------------------------------------*/
	bStart = nextActiveVertex(P,1,kForwards) ;
	bEnd = nextActiveVertex(P,bStart,kBackwards) ;
	area += cnvxPolyArea( P, bStart, bEnd, kForwards ) ;
	
	return( area );
}
