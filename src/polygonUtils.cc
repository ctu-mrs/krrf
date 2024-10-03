#include "polygonUtils.h"

#include <list>
#include <vector>

namespace rrtPlanning {

using namespace std;

inline int wind(const TPoint &a, const TPoint &b, const TPoint &c) {
   double w = ((a.y - b.y)*(c.x - b.x) - (c.y - b.y)*(a.x - b.x));
   return (w > .0001) ? 1 : ((w < -.0001) ? -1 : 0);
}

inline int in_cone (const TPoint &a0,const TPoint &a1,const TPoint &a2,const TPoint &b) {
   int m = wind(b,a0,a1);
   int p = wind(b,a1,a2);
   if (wind(a0,a1,a2) > 0)
      return ( m >= 0 && p >= 0 ); /* convex at a */
   else
      return ( m >= 0 || p >= 0 ); /* reflex at a */
}

inline int inBetween (const TPoint &a,const TPoint &b,const TPoint &c) {
   if (a.x != b.x)   /* not vertical */
      return (((a.x < c.x) && (c.x < b.x)) || ((b.x < c.x) && (c.x < a.x)));
   else
      return (((a.y < c.y) && (c.y < b.y)) || ((b.y < c.y) && (c.y < a.y)));
}



/* the code is adopted from J.Faigl's cis.cpp from imrh::pathPlanning */
inline int intersect(const TPoint &a,const TPoint &b,const TPoint &c,const TPoint &d) {
   int a_abc;
   int a_abd;
   int a_cda;
   int a_cdb;
   a_abc = wind(a,b,c);
   if ((a_abc == 0) && inBetween (a,b,c)) {
      return 1;
   }
   a_abd = wind(a,b,d);
   if ((a_abd == 0) && inBetween (a,b,d)) {
      return 1;
   }
   a_cda = wind(c,d,a);
   a_cdb = wind(c,d,b);
   // True if c and d are on opposite sides of ab,
   // and a and b are on opposite sides of cd.
   //
   return (((a_abc * a_abd) < 0) && ((a_cda * a_cdb) < 0));
}

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2 on the line
//            <0 for P2 right of the line
//    See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
inline double isLeft(const  TPoint &P0, const TPoint &P1, const TPoint &P2 )
{
    return ( (P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y) );
}

//===================================================================

// cn_PnPoly(): crossing number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  0 = outside, 1 = inside
// This code is patterned after [Franklin, 2000]

int cn_PnPoly(const TPoint &P, const vector< TPoint > &V) {
    int    cn = 0;    // the crossing number counter
	const int n = (int)V.size()-1;

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {    // edge from V[i] to V[i+1]
       if (((V[i].y <= P.y) && (V[i+1].y > P.y))    // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <= P.y))) { // a downward crossing
            // compute the actual edge-ray intersect x-coordinate
            double vt = 1.0*(P.y - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x < V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
                ++cn;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if odd (in)

}


/** this function do same thig as function cn_PnPoly but
  * for given polygon, v[0] != v[last] !!!!
  */
int cn_PnPoly2(const TPoint &P, const vector< TPoint > &V) {
    int    cn = 0;    // the crossing number counter
	const int n = (int)V.size();

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {    // edge from V[i] to V[i+1]
		const int j = (i == (n-1))?0:(i+1);
       if (((V[i].y <= P.y) && (V[j].y > P.y))    // an upward crossing
        || ((V[i].y > P.y) && (V[j].y <= P.y))) { // a downward crossing
            // compute the actual edge-ray intersect x-coordinate
            double vt = 1.0*(P.y - V[i].y) / (V[j].y - V[i].y);
            if (P.x < V[i].x + vt * (V[j].x - V[i].x)) {
				// P.x < intersect
                ++cn;   // a valid crossing of y=P.y right of P.x
			}
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if odd (in)

}

// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only if P is outside V[])
int wn_PnPoly( const TPoint &P, const vector< TPoint > &V) { 
    int    wn = 0;    // the winding number counter
	const int n = (int)V.size()-1;

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to V[i+1]
        if (V[i].y <= P.y) {         // start y <= P.y
            if (V[i+1].y > P.y)      // an upward crossing
                if (isLeft( V[i], V[i+1], P) > 0)  // P left of edge
                    ++wn;            // have a valid up intersect
        }
        else {                       // start y > P.y (no test needed)
            if (V[i+1].y <= P.y)     // a downward crossing
                if (isLeft( V[i], V[i+1], P) < 0)  // P right of edge
                    --wn;            // have a valid down intersect
        }
    }
    return wn;

}



// wn_PnPoly2(): winding number test for a point in a polygon
//
// this function assume, that v[0] != v[n] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//      Input:   P = a point,
//               V[] = vertex points of a polygon 
//      Return:  wn = the winding number (=0 only if P is outside V[]) 
int wn_PnPoly2( const TPoint &P, const vector< TPoint > &V) {
    int    wn = 0;    // the winding number counter
	const int n = (int)V.size();
	//int ni;

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to V[i+1]
		const int ni = (i == n-1) ? 0 : (i+1);
        if (V[i].y <= P.y) {         // start y <= P.y
            if (V[ni].y > P.y)      // an upward crossing
                if (isLeft( V[i], V[ni], P) > 0) {
				   	// P left of edge
                    ++wn;            // have a valid up intersect
				}
        }
        else {                       // start y > P.y (no test needed)
            if (V[ni].y <= P.y) {     // a downward crossing
                if (isLeft( V[i], V[ni], P) < 0) {  // P right of edge
                    --wn;            // have a valid down intersect
				}
			}
        }
    }
    return wn;

}



// return true if robot( polygon a) is not in collision with 
// map (polygon b)
// false     all vertices of a are in b and no intersection
//           between segment of a vs b.
// true      othervise
bool robotMapCollision(const vector< TPoint > &a, const vector< TPoint > &b) {

	const int asize = (int)a.size();
	const int bsize = (int)b.size();
	int ni,nj;

	// check wether all point of a are in 'b'	
	for(int i=0;i<asize;i++) {
		if (wn_PnPoly(a[i],b) == 0)
			return true;
	}

	for(int i=0 ; i<asize ; i++) {
			
		ni = (i == asize-1) ? 0:i+1;

		for(int j=0 ; j<bsize ; j++) {

			nj = (j == bsize-1) ? 0:j+1;
			if (intersect(a[i],a[ni],b[j],b[nj])) {
				return true;
			}

		}

	}

	return false;

}





// return true if robot( polygon a) is not in collision with 
// map (polygon b)
// false     all vertices of a are in b and no intersection
//           between segment of a vs b.
// true      othervise
// for robot now map is needed v[last]==v[0]
bool robotObstaclesCollision(const vector< TPoint > &a, const vector< TPoint > &b) {

	const int asize = (int)a.size();
	const int bsize = (int)b.size();
	int ni,nj;

	// check wether all point of a are in 'b'	
	for(int i=0;i<asize;i++) {
		if (wn_PnPoly2(a[i],b) != 0)
			return true;
	}

	for(int i=0 ; i<asize ; i++) {
			
		ni = (i == asize-1) ? 0:i+1;

		for(int j=0 ; j<bsize ; j++) {

			nj = (j == bsize-1) ? 0:j+1;
			if (intersect(a[i],a[ni],b[j],b[nj])) {
				return true;
			}

		}

	}

	return false;

}




/* return true if there is intersection between any segments of 'a' and
   any segment of 'b'. return false otherwise
   a .. points of robot polygon
   b .. map polygon (obstacle)
*/
bool robotObstacleCollision(const vector< TPoint > &a, const vector< TPoint > &b) {

	const int asize = (int)a.size();
	const int bsize = (int)b.size();
	int ni,nj;

	// check wether all point of a are in 'b'	
	for(int i=0;i<asize;i++) {
		if (wn_PnPoly(a[i],b) != 0) {
		//	cerr << "pts" << a[i].x << "," << a[i].y << " is in ";
			return true;
		}
	}

	for(int i=0;i<bsize;i++) {
		if (wn_PnPoly2(b[i],a) != 0) {
			return true;
		}
	}

	for(int i=0 ; i<asize ; i++) {
			
		ni = (i == asize-1) ? 0:i+1;

		for(int j=0 ; j<bsize ; j++) {

			nj = (j == bsize-1) ? 0:j+1;

			if (intersect(a[i],a[ni],b[j],b[nj])) {
				return true;
			}

		}

	}

	return false;

}

/* return true if there is intersection between any segments of 'a' and
   any segment of 'b'. return false otherwise
   a .. points of robot polygon
   b .. point of second robot polygon
   assume that a[0] != a[n-1] && b[0] != b[n-1]

   wn_PnPoly2 is used for point in polygon test
*/
bool robotRobotCollision(const vector< TPoint > &a, const vector< TPoint > &b) {

	const int asize = (int)a.size();
	const int bsize = (int)b.size();
	int ni,nj;

	
	// check wether all point of a are in 'b'	
	for(int i=0;i<asize;i++) {
		if (wn_PnPoly2(a[i],b) != 0) {
		//	cerr << "pts" << a[i].x << "," << a[i].y << " is in ";
			return true;
		}
	}

	for(int i=0;i<bsize;i++) {
		if (wn_PnPoly2(b[i],a) != 0) {
			return true;
		}
	}

	for(int i=0 ; i<asize ; i++) {
			
		ni = (i == asize-1) ? 0:i+1;

		for(int j=0 ; j<bsize ; j++) {

			nj = (j == bsize-1) ? 0:j+1;

			if (intersect(a[i],a[ni],b[j],b[nj])) {
				return true;
			}

		}

	}

	return false;

}




bool robotRobotCollision(const vector< vector< TPoint > > &a, const vector< vector< TPoint > > &b) {


	const int asize = (int)a.size();
	const int bsize = (int)b.size();

	bool col = false;
	for(int i=0;i<asize && !col;i++) {
		for(int j=0;j<bsize && !col;j++) {
			col = robotRobotCollision(a[i],b[i]);
		}
	}
	return col;

}



/* return true if there is intersection between any segments of 'a' and
   any segment of 'b'. return false otherwise
   a .. points of robot polygon
   b .. point of second robot polygon
   assume that a[0] != a[n-1] && b[0] != b[n-1]

	this function use cp_poly2 for point in polygon test
*/
bool robotRobotCollisionCn(const vector< TPoint > &a, const vector< TPoint > &b) {

	const int asize = (int)a.size();
	const int bsize = (int)b.size();
	int ni,nj;

	// check wether all point of a are in 'b'	
	for(int i=0;i<asize;i++) {
		if (cn_PnPoly2(a[i],b) != 0) {
		//	cerr << "pts" << a[i].x << "," << a[i].y << " is in ";
			return true;
		}
	}

	for(int i=0;i<bsize;i++) {
		if (cn_PnPoly2(b[i],a) != 0) {
			return true;
		}
	}

	for(int i=0 ; i<asize ; i++) {
			
		ni = (i == asize-1) ? 0:i+1;

		for(int j=0 ; j<bsize ; j++) {

			nj = (j == bsize-1) ? 0:j+1;

			if (intersect(a[i],a[ni],b[j],b[nj])) {
				return true;
			}

		}

	}

	return false;

}




bool robotRobotCollisionCn(const vector< vector< TPoint > > &a, const vector< vector< TPoint > > &b) {


	const int asize = (int)a.size();
	const int bsize = (int)b.size();

	bool col = false;
	for(int i=0;i<asize && !col;i++) {
		for(int j=0;j<bsize && !col;j++) {
			col = robotRobotCollisionCn(a[i],b[i]);
		}
	}
	return col;

}



struct S {
		int idx;
		double r;
		S(const int i, const double rel):idx(i),r(rel) {}
};

template<typename T>
class myLess {
	public:
	bool operator()(const T &a, const T &b) {
		return a.r < b.r;
	}
};

double getRelevance(const TPoint &pi, const TPoint &pj, const TPoint &pk) {
	double rel, dx,dy;
	dx = pi.x - pj.x;
	dy = pi.y - pj.y;
	rel = sqrt(dx*dx + dy*dy);
	dx = pj.x - pk.x;
	dy = pj.y - pk.y;
	rel += sqrt(dx*dx + dy*dy);
	dx = pi.x - pk.x;
	dy = pi.y - pk.y;
	rel -= dx*dx+ dy*dy;
	return fabs(rel);
}

double getRelevance(const list< S > &s, list< S >::const_iterator i, const vector< TPoint > &pts) {
	list< S >::const_iterator pred = i;
	list< S >::const_iterator succ = i;
	pred--;
	succ++;

	double rel = -1;
	if (i != s.begin() && succ != s.end()) {
		rel = getRelevance(pts[pred->idx],pts[i->idx],pts[succ->idx]);
	} 
	return rel;
}

vector< TPoint > filterRelevance(const vector< TPoint > &pts, const double maxR) {
	
	typedef list< S >::iterator Iterator;
	typedef list< S >::const_iterator Const_iterator;
	if (pts.size() < 3)
		return pts;

	list< S > r;
	double maxRel = -1;
	double rel;

	for(int j=0;j<(int)pts.size();j++) {
		if (j > 0 && j < (int)pts.size()-1) {
			rel = getRelevance(pts[j-1], pts[j], pts[j+1]);
		} else 
			rel = -1;
		r.push_back(S(j,rel));
		if (rel > maxRel || maxRel == -1)
			maxRel = rel;
	}
	r.front().r = 2*maxRel;
	r.back().r = 2*maxRel;

	double minR,rr;
	do {
		Iterator me = std::min_element(r.begin(),r.end(),myLess< S >());
		minR = me->r;
		if (me != r.begin() && me != r.end()) {
			Iterator i = me;
			i--;
			Iterator j = me;
			j++;
			r.erase(me);
			if (i != r.begin() && i != r.end()) {
				rr = getRelevance(r,i,pts);
				if (rr >= 0)
					i->r = rr;
			}
			if (j != r.begin() && j != r.end()) {
				rr = getRelevance(r,j,pts);
				if (rr >= 0)
					j->r = rr;
			}
		}
		if (r.size() <= 3)
			break;
	} while (minR < maxR);
	
	vector< TPoint > res;
	res.reserve(r.size());
	for(Const_iterator i = r.begin(); i != r.end(); i++) {
		res.push_back(pts[i->idx]);
	}

	return res;
}




vector< TPoint >cutPolygon(const vector< TPoint > &pts, const double maxDistance, const TPoint &center) {
	vector< TPoint > result;
	double dx,dy;
	const double d2 = maxDistance * maxDistance;

	for(int i=0;i<(int)pts.size();i++) {
		dx = pts[i].x - center.x;
		dy = pts[i].y - center.y;
		if ( (dx * dx + dy * dy) < d2) {
			result.push_back(pts[i]);
		}
	}

	return result;
}

// return distance from point 'p' to segment  's1--s2'
double dist_Point_to_Segment(const TPoint &p, const TPoint &s1, const TPoint &s2) {
	const double dx = s2.x - s1.x;
	const double dy = s2.y - s1.y;

	const double wx = p.x - s1.x;
	const double wy = p.y - s1.y;

	const double c1 = wx * dx + wy * dy;

	double ddx,ddy;
    if ( c1 <= 0 ) {
		ddx = p.x - s1.x;
		ddy = p.y - s1.y;
		return (ddx*ddx + ddy*ddy);
	}
	const double c2 = dx*dx + dy*dy;
    if ( c2 <= c1 ) {
		ddx = p.x - s2.x;
		ddy = p.y - s2.y;
		return (ddx*ddx + ddy*ddy);
	}

	const double b = c1 / c2;

	const double ax = s1.x + b*dx;
	const double ay = s1.y + b*dy;

	ddx = p.x - ax;
	ddy = p.y - ay;

	return (ddx*ddx + ddy*ddy);
}


int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy) {
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
		 (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) 
	   ) {
		c = !c;
	}
 }
 return c;
}

/** 
  * first and last point are not same!
  */
bool pointInPolygon2(const TPoint &point, const vector<TPoint > &polygon) {

	const int size = (int)polygon.size();

	double *arrayx = new double[size];
	double *arrayy = new double[size];

	for(int i=0; i < size ; i++) {
		arrayx[i] = polygon[i].x;
		arrayy[i] = polygon[i].y;
	}
	int res = pnpoly(size,arrayx,arrayy,point.x,point.y);

	delete [] arrayx;
	delete [] arrayy;

	return res;
}


bool pointInPolygon(const TPoint &point, const vector<TPoint> &polygon) {
    int i, j; //, c = 0;
    bool c = false;
    const int nvert = (int)polygon.size();

    for (i = 0, j = nvert-1; i < nvert; j = i++) {

        if ( ((polygon[i].y > point.y) != (polygon[j].y> point.y)) &&
                (point.x < (polygon[j].x-polygon[i].x) * (point.y-polygon[i].y) / (polygon[j].y-polygon[i].y) + polygon[i].x) 
           ) {
            c = !c;
        }
    }
    return c;
}

/** replace segment of collinear points by their end points
  */
vector< TPoint > removeCollinearPoints_old(const vector<TPoint> &pts) {
	
	const int size = (int)pts.size();
	
	
	int from,to;
	from = 0;
	to = 0+2;

	vector< TPoint > result;
	result.reserve(size*2/3);


	while(from < size && to < size) {
		while(to < size && wind(pts[from],pts[to-1],pts[to]) == 0) {
			to++;
		}
		--to;
		if (result.size() > 0) {
			if (!(result.back().x == pts[from].x && result.back().y == pts[from].y) ) {
				result.push_back(pts[from]);
			}
		} else {
			result.push_back(pts[from]);
		}
		result.push_back(pts[to]);
		from = to;
		to = from+2;
	}

	return result;
}




/** return true if line point a,b,c, are collinear */
inline bool isCollinear(const TPoint &a, const TPoint &b, const TPoint &c) {

	const double d1 = pointDistanceEucleid(a,b);
	const double d2 = pointDistanceEucleid(b,c);
	const double d3 = pointDistanceEucleid(a,c);
	return  (fabs(d1+d2-d3) < 0.0001);
}

/** remove collinear points from give polyline */
std::vector<TPoint> removeCollinearPoints(const std::vector<TPoint> &pts) {

	std::vector<TPoint> result;
	std::vector<bool> del(pts.size(),false);

	for(int i=1;i<(int)pts.size()-1;i++) {
		del[i] = (isCollinear(pts[i-1],pts[i],pts[i+1]));
	}

	for(int i=0;i<(int)del.size();i++) {
		if (!del[i]) {
			result.push_back(pts[i]);
		}
	}
	
	del.clear();
	return result;
}

/** appoximate a segment with 'num' points */
void approximateSegment(const TPoint &from, const TPoint &to, const int num, vector<TPoint> &result) {

    result.clear();
	result.reserve(num+2);

	result.push_back(from);

	double x,y;
	for(double t=0.0;t < 1.0; t+=1.0/(double)num) {
		x = from.x*(1-t) + to.x*t;
		y = from.y*(1-t) + to.y*t;
		result.push_back(TPoint(x,y));
	}
	result.push_back(to);

}


// return of points p1 and p2 can visit each other in polygon
// given in a vector 'map'
// 'map' contains point of a polygon, so first and last point does not equal
bool isVisible(const TPoint &p1, const TPoint &p2, const vector<TPoint> &map) {

	for(int i=0;i<(int)map.size()-1;i++) {
		if (rrtPlanning::intersect(p1,p2,map[i],map[i+1])) {
			return false;
		}
	}
	return true;
}


int getLastVisiblePoint(const vector<TPoint> &pts, const int from, const vector<TPoint> &map) {
	bool end = false;

	int first = from;
	int last = pts.size() - 1;
	int middle;

	while(!end) {
		middle = first + (last - first) / 2;
		if (isVisible(pts[first],pts[last],map)) {
			end = true;
		} else if (isVisible(pts[first],pts[middle],map)) {
			last = middle + (last - middle) / 2;
		} else {
			last = middle;
		}	
	}
	return last;
}




vector<TPoint> getVisiblePoints(const vector<TPoint> &pts, const vector<TPoint> &map) {

	if (pts.size() == 0) {
		return vector<TPoint>();
	}	

	vector<TPoint> tmp;

	int first = 0;
	int last;
	tmp.push_back(pts[0]);
	do {
		last = getLastVisiblePoint(pts,first,map);
		if (last != first) {
			tmp.push_back(pts[last]);
			first = last;
		} else {
			first++;
		}
	} while (first < (int)pts.size() -1);

	return tmp;	
}


/// for each segment in polygon, return it's lenght
/// polygon: p0,p1,..,pn
/// result:  l1=0,l1,..,ln
/// l0 = lenght of (p0,p1), l1 = length(p1,p2), ... , ln = length(pn,p0)
std::vector<double> getLengths(const std::vector<TPoint> &pts) {
	std::vector<double> result;
	result.reserve(pts.size());

	int j;
	for(int i=0;i<(int)pts.size();i++) {
		j = (i == ((int)pts.size() -1) ) ? 0 : (i+1);
		result.push_back(pointDistanceEucleid(pts[i],pts[j]));
	}
	return result;
}






std::vector<TPoint> resamplePolygon(const std::vector<TPoint> &polygon, const double delta) {

	double length = 0;
	vector<double> lengths(getLengths(polygon));

	for(int i=0;i<(int)polygon.size()-1;i++) {
		length += pointDistanceEucleid(polygon[i],polygon[i+1]);
	}

	const int numOfSamples = (int)lround(length / delta);

	std::vector<TPoint> result;
	result.reserve(numOfSamples);

	int last = 0;
	const int size = polygon.size();
	double alength = 0;
	double tmpl;
	int lnext;
	for(double l = 0; l < length; l+=delta) {
		while((alength + lengths[last]) < l  && last < size) {
			alength+=lengths[last];
			last++;
		}
		if (last == size) {
			break;
		}
		lnext = (last == (size-1)) ? 0 : (last+1);
		tmpl = l - alength;
		result.push_back(TPoint(polygon[last].x*(1-tmpl / lengths[last])+polygon[lnext].x*(tmpl/lengths[last]),
					polygon[last].y*(1-tmpl/lengths[last])+polygon[lnext].y*(tmpl/lengths[last])));		
	}

	return result;
}



/** sign function for pointInTriangle method. Altough the Point3 type is used, only x and y axes are
  considered in the function */
double Sign2(const TPoint3 &p1, const TPoint3 &p2, const TPoint3 &p3) {
      return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}


/** return true if the point 's' lies inside triangle a,b,c. Altough 3D points are used in the function,
  only x a y coordinates are considered.
  */
bool pointInTriangle(const TPoint3 &pt, const TPoint3 &v1, const TPoint3 &v2, const TPoint3 &v3) {
    bool b1, b2, b3;

    b1 = Sign2(pt, v1, v2) < 0.0;
    b2 = Sign2(pt, v2, v3) < 0.0;
    b3 = Sign2(pt, v3, v1) < 0.0;

    return ((b1 == b2) && (b2 == b3));
}







}



