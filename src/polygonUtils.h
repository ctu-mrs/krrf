#ifndef _POLYGON_UTILS_H_
#define _POLYGON_UTILS_H_

#include "types.h"
#include <vector>
#include <list>
#include <algorithm>
//#include "map.h"

namespace rrtPlanning {


inline int wind(const TPoint &a, const TPoint &b, const TPoint &c);
inline int in_cone (const TPoint &a0,const TPoint &a1,const TPoint &a2,const TPoint &b);
inline int inBetween (const TPoint &a,const TPoint &b,const TPoint &c);

inline int intersect(const TPoint &a,const TPoint &b,const TPoint &c,const TPoint &d);

inline double isLeft(const  TPoint &P0, const TPoint &P1, const TPoint &P2);

int cn_PnPoly(const TPoint &P, const std::vector< TPoint > &V);
int cn_PnPoly2(const TPoint &P, const std::vector< TPoint > &V);

int wn_PnPoly( const TPoint &P, const std::vector< TPoint > &V);
int wn_PnPoly2( const TPoint &P, const std::vector< TPoint > &V);

bool robotMapCollision(const std::vector< TPoint > &a, const std::vector< TPoint > &b);
bool robotObstaclesCollision(const std::vector< TPoint > &a, const std::vector< TPoint > &b);
bool robotObstacleCollision(const std::vector< TPoint > &a, const std::vector< TPoint > &b);
bool robotRobotCollision(const std::vector< TPoint > &a, const std::vector< TPoint > &b);
bool robotRobotCollision(const std::vector< std::vector< TPoint > > &a, const std::vector< std::vector< TPoint > > &b);
bool robotRobotCollisionCn(const std::vector< TPoint > &a, const std::vector< TPoint > &b);
bool robotRobotCollisionCn(const std::vector< std::vector< TPoint > > &a, const std::vector< std::vector< TPoint > > &b);


double getRelevance(const TPoint &pi, const TPoint &pj, const TPoint &pk);
std::vector< TPoint > filterRelevance(const std::vector< TPoint > &pts, const double maxR);
std::vector< TPoint >cutPolygon(const std::vector< TPoint > &pts, const double maxDistance, const TPoint &center = TPoint(0,0));

double dist_Point_to_Segment(const TPoint &p, const TPoint &s1, const TPoint &s2);

int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
bool pointInPolygon2(const TPoint &point, const std::vector<TPoint > &polygon);


bool pointInPolygon(const TPoint &point, const std::vector<TPoint> &polygon);

std::vector< TPoint > removeCollinearPoints_old(const std::vector<TPoint> &pts);
inline bool isCollinear(const TPoint &a, const TPoint &b, const TPoint &c);
inline bool isCollinear(const TPoint &a, const TPoint &b, const TPoint &c);
std::vector<TPoint> removeCollinearPoints(const std::vector<TPoint> &pts);


void approximateSegment(const TPoint &from, const TPoint &to, const int num, vector< TPoint> &result);

bool isVisible(const TPoint &p1, const TPoint &p2, const std::vector<TPoint> &map);
int getLastVisiblePoint(const std::vector<TPoint> &pts, const int from, const std::vector<TPoint> &map);


std::vector<TPoint> getVisiblePoints(const std::vector<TPoint> &pts, const std::vector<TPoint> &map);




// for given points find largest visible segment in given map
//
template<typename RT, typename MT>
std::vector<TPoint> smoothPtsVisibility(const std::list<TPoint> &pts, MT &map, RT &robot) {
	const std::vector<TPoint> &mapPts(map.getMap());	
	return getVisiblePoints(std::vector<TPoint>(pts.begin(),pts.end()), mapPts);
}





// this method is from  http://tog.acm.org/GraphicsGems//gemsiv/ptpoly_haines/
//int pointInPolygon_CrossingsTest( pgon, numverts, point )
template<typename T>
int pointInPolygon_CrossingsTest( T **pgon, int numverts, T point[2] )
//double	pgon[][2] ;
//int	numverts ;
//double	point[2] ;
{
#ifdef	WINDING
register int	crossings ;
#endif
register int	j, yflag0, yflag1, inside_flag, xflag0 ;
register double ty, tx, *vtx0, *vtx1 ;
#ifdef	CONVEX
register int	line_flag ;
#endif

	const int X = 0;
	const int Y = 1;
    tx = point[X] ;
    ty = point[Y] ;

    vtx0 = pgon[numverts-1] ;
    /* get test bit for above/below X axis */
    yflag0 = ( vtx0[Y] >= ty ) ;
    vtx1 = pgon[0] ;

#ifdef	WINDING
    crossings = 0 ;
#else
    inside_flag = 0 ;
#endif
#ifdef	CONVEX
    line_flag = 0 ;
#endif
    for ( j = numverts+1 ; --j ; ) {

	yflag1 = ( vtx1[Y] >= ty ) ;
	/* check if endpoints straddle (are on opposite sides) of X axis
	 * (i.e. the Y's differ); if so, +X ray could intersect this edge.
	 */
	if ( yflag0 != yflag1 ) {
	    xflag0 = ( vtx0[X] >= tx ) ;
	    /* check if endpoints are on same side of the Y axis (i.e. X's
	     * are the same); if so, it's easy to test if edge hits or misses.
	     */
	    if ( xflag0 == ( vtx1[X] >= tx ) ) {

		/* if edge's X values both right of the point, must hit */
#ifdef	WINDING
		if ( xflag0 ) crossings += ( yflag0 ? -1 : 1 ) ;
#else
		if ( xflag0 ) inside_flag = !inside_flag ;
#endif
	    } else {
		/* compute intersection of pgon segment with +X ray, note
		 * if >= point's X; if so, the ray hits it.
		 */
		if ( (vtx1[X] - (vtx1[Y]-ty)*
		     ( vtx0[X]-vtx1[X])/(vtx0[Y]-vtx1[Y])) >= tx ) {
#ifdef	WINDING
		    crossings += ( yflag0 ? -1 : 1 ) ;
#else
		    inside_flag = !inside_flag ;
#endif
		}
	    }
#ifdef	CONVEX
	    /* if this is second edge hit, then done testing */
	    if ( line_flag ) goto Exit ;

	    /* note that one edge has been hit by the ray's line */
	    line_flag = TRUE ;
#endif
	}

	/* move to next pair of vertices, retaining info as possible */
	yflag0 = yflag1 ;
	vtx0 = vtx1 ;
	vtx1 += 2 ;
    }
#ifdef	CONVEX
    Exit: ;
#endif
#ifdef	WINDING
    /* test if crossings is not zero */
    inside_flag = (crossings != 0) ;
#endif

    return( inside_flag ) ;
}

template<typename NT>
bool pointInPolygon_Crossingtest(const TPoint &point, const std::vector<TPoint> &polygon) {

	double **pgon = new double*[polygon.size()];
	for(int i=0;i<(int)polygon.size();i++) {
		pgon[i] = new double[2];
	}

	for(int i=0;i<(int)polygon.size();i++) {
		pgon[i][0] = (double)polygon[i].x;
		pgon[i][1] = (double)polygon[i].y;
	}

	double po[2];
	po[0] = (double)point.x;
	po[1] = (double)point.y;

	int result = pointInPolygon_CrossingsTest(pgon,polygon.size(), po);

	for(int i=0;i<(int)polygon.size();i++) {
		delete [] pgon[i];
	}
	delete [] pgon;

	return result;
}



std::vector<double> getLengths(const std::vector<TPoint> &pts);

std::vector<TPoint> resamplePolygon(const std::vector<TPoint> &polygon, const double delta);


bool pointInTriangle(const TPoint3 &pt, const TPoint3 &v1, const TPoint3 &v2, const TPoint3 &v3);


} // namesapce

#endif



