
#include "ut.h"
#include "polygonUtils.h"
#include "WLog.h"

#include <algorithm>
#include <numeric>
#include <math.h>
#include <fstream>

namespace rrtPlanning {


void getTime(struct rusage *t){
	getrusage(RUSAGE_SELF,t);
}

std::string timeStr() {
	time_t t = time(NULL);
	std::string res(ctime(&t));
	for(int i=0;i<(int)res.size();i++) {
		if (res[i] == '\n') {
			res[i] = ' ';
		}
	}
	return res;
}



// from = left upper
// to = right bottom
std::vector< std::vector< TPoint > > box2triangles(const TPoint &from, const TPoint &to) {

    std::vector< std::vector<TPoint> > res;

    if (from.x > to.x || from.y < to.y) {
        WDEBUG("Cannot make box from " << from << " to " << to << " (top-left -> bottom-right");
        exit(0);
    }

    res.push_back( std::vector< TPoint >() );
    res.back().push_back(from);
    res.back().push_back(TPoint(from.x, to.y));
    res.back().push_back(to);

    res.push_back( std::vector< TPoint >() );
    res.back().push_back(from);
    res.back().push_back(to);
    res.back().push_back(TPoint(to.x, from.y));

    return res;

}



void saveTriangles(const vector< Triangle > &triangles, const char *filename) {
	std::ofstream ofs(filename);
	for(int i=0;i<(int)triangles.size();i++) {
        const Triangle &t(triangles[i]);
        ofs << t.a.x << " " << t.a.y << " " << t.a.z << " ";
        ofs << t.b.x << " " << t.b.y << " " << t.b.z << " ";
        ofs << t.c.x << " " << t.c.y << " " << t.c.z << "\n";
	}
	ofs.close();
}



double getTime(struct rusage one, struct rusage two) {

	const unsigned long as = one.ru_utime.tv_sec;
	const unsigned long bs = two.ru_utime.tv_sec;
	const unsigned long aus = one.ru_utime.tv_usec;
	const unsigned long bus = two.ru_utime.tv_usec;

	return (double)((double)bs-(double)as) + 
		(double)((double)bus-(double)aus)/1000000.0;

}

void removeComment(std::string &s, const char commentChar) {
	std::size_t p = s.find_first_of(commentChar);
	
	if (p != std::string::npos) {
		s.erase(p);
	}

}


/** return [min max mean dev med] for given data */
std::vector<double> getStats(const std::vector<double> &data) {
	std::vector<double> result(5,0.0);
	if (data.size() == 0) {
		return result;
	}
	const double sum = std::accumulate(data.begin(),data.end(),0.0);
	const double mean = sum / data.size();
	double dev = 0;
	for(int i=0;i<(int)data.size();i++) {
		dev += (data[i] - mean)*(data[i]-mean);
	}
	dev = sqrt(dev / data.size());

	const double min = *std::min_element(data.begin(),data.end());
	const double max = *std::max_element(data.begin(),data.end());

	std::vector<double> tmp(data);
	std::nth_element(tmp.begin(),tmp.begin()+tmp.size()/2,tmp.end());
	const double med = *(tmp.begin()+tmp.size()/2);

	tmp.clear();

	result[0] = min;
	result[1] = max;
	result[2] = mean;
	result[3] = dev;
	result[4] = med;
	return result;
}

void printPercentStatus(const double actual, const double max, double &old, const std::string &prefix, std::ostream &os) {

	char tmp[20];
	double n = 100*actual/max;
	if (fabs(n-old) > 0.1) {
		snprintf(tmp,sizeof(tmp),"%.2lf%%   \r",n);
		old = n;
		os << prefix << tmp; 
	}
}

/** convert qwuaternion to rotation: Rx, Ry, Rz in this order */
void convertQuaternionToRotation(double &rx, double &ry, double &rz, const double q0, const double q1, const double q2, const double q3) {

	rx = atan2( 2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2) );
	ry = asin( 2*(q0*q2 - q3*q1) );
	rz = atan2( 2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3) );

}

/** get vector after rotation about Rx then Ry then Rz. */
void rotateVector(double &x, double &y, double &z, const double rx, const double ry, const double rz) {

	const double ca = cos(rz);
	const double sa = sin(rz);
	const double cb = cos(ry);
	const double sb = sin(ry);
	const double cg = cos(rx);
	const double sg = sin(rx);
	double matrix[3][3];

	matrix[0][0] = ca*cb;
	matrix[0][1] = ca*sb*sg-sa*cg;
	matrix[0][2] = ca*sb*cg+sa*sg;
	matrix[1][0] = sa*cb;
    matrix[1][1] = sa*sb*sg+ca*cg;
	matrix[1][2] = sa*sb*cg-ca*sg;
	matrix[2][0] = -sb;
	matrix[2][1] = cb*sg;
	matrix[2][2] = cb*cg;
	const double vx = x;
	const double vy = y;
	const double vz = z;

	x = matrix[0][0]*vx + matrix[0][1]*vy + matrix[0][2]*vz;
	y = matrix[1][0]*vx + matrix[1][1]*vy + matrix[1][2]*vz;
	z = matrix[2][0]*vx + matrix[2][1]*vy + matrix[2][2]*vz;
	
}


/** return true if given file exists */
bool file_exists(const char *filename) {
	std::ifstream ifs(filename);
	if (ifs.is_open()) {
		ifs.close();
		return true;
	}
	return false;
}

/** make a triangle for a cube of size 'size' 
  * can be used for definitino of a tiangles for RAPID model 
  * the 'idx' parameter is between (0,11) and it determines which triangle will be defined */
void makeCube(const int idx, const double size, const double x, const double y, const double z, double p1[3], double p2[3], double p3[3]) {
	const double s = size/2;

	switch(idx) {
		case 0: { // front face
					p1[0] = x-s; p1[1] = y-s; p1[2] = z-s; 
					p2[0] = x+s; p2[1] = y-s; p2[2] = z-s; 
					p3[0] = x+s; p3[1] = y-s; p3[2] = z+s; 
					break;
				}
		case 1: { // front face
					p1[0] = x-s; p1[1] = y-s; p1[2] = z-s; 
					p2[0] = x+s; p2[1] = y-s; p2[2] = z+s; 
					p3[0] = x-s; p3[1] = y-s; p3[2] = z+s; 
					break;
				}
		case 2: { // right face
					p1[0] = x+s; p1[1] = y-s; p1[2] = z-s; 
					p2[0] = x+s; p2[1] = y-s; p2[2] = z+s; 
					p3[0] = x+s; p3[1] = y+s; p3[2] = z-s; 
					break;
				}
		case 3: { // right face
					p1[0] = x+s; p1[1] = y-s; p1[2] = z+s; 
					p2[0] = x+s; p2[1] = y+s; p2[2] = z-s; 
					p3[0] = x+s; p3[1] = y+s; p3[2] = z+s; 
					break;
				}
		case 4: { // left face
					p1[0] = x-s; p1[1] = y-s; p1[2] = z-s; 
					p2[0] = x-s; p2[1] = y-s; p2[2] = z+s; 
					p3[0] = x-s; p3[1] = y+s; p3[2] = z-s; 
					break;
				}
		case 5: { // left face
					p1[0] = x-s; p1[1] = y-s; p1[2] = z+s; 
					p2[0] = x-s; p2[1] = y+s; p2[2] = z-s; 
					p3[0] = x-s; p3[1] = y+s; p3[2] = z+s; 
					break;
				}
		case 6: { // rear face
					p1[0] = x-s; p1[1] = y+s; p1[2] = z-s; 
					p2[0] = x+s; p2[1] = y+s; p2[2] = z-s; 
					p3[0] = x+s; p3[1] = y+s; p3[2] = z+s; 
					break;
				}
		case 7: { // rear face
					p1[0] = x-s; p1[1] = y+s; p1[2] = z-s; 
					p2[0] = x+s; p2[1] = y+s; p2[2] = z+s; 
					p3[0] = x-s; p3[1] = y+s; p3[2] = z+s; 
					break;
				}
		case 8: { // top face
					p1[0] = x-s; p1[1] = y+s; p1[2] = z+s; 
					p2[0] = x-s; p2[1] = y-s; p2[2] = z+s; 
					p3[0] = x+s; p3[1] = y-s; p3[2] = z+s; 
					break;
				}
		case 9: { // top face
					p1[0] = x-s; p1[1] = y+s; p1[2] = z+s; 
					p2[0] = x+s; p2[1] = y-s; p2[2] = z+s; 
					p3[0] = x+s; p3[1] = y+s; p3[2] = z+s; 
					break;
				}
		case 10: { // bottom face
					p1[0] = x-s; p1[1] = y+s; p1[2] = z-s; 
					p2[0] = x-s; p2[1] = y-s; p2[2] = z-s; 
					p3[0] = x+s; p3[1] = y-s; p3[2] = z-s; 
					break;
				}
		case 11: { // bottom face
					p1[0] = x-s; p1[1] = y+s; p1[2] = z-s; 
					p2[0] = x+s; p2[1] = y-s; p2[2] = z-s; 
					p3[0] = x+s; p3[1] = y+s; p3[2] = z-s; 
					break;
				}

	}
}

/** make a cube and rotates it around z (vertical) axis in rad.
  *
        /------/
	   / |    /|
	  /------/ |
  x3  |  |---|-| x2
	  | /    | / 
	  |/     |/
  x0  -------/  x1
   */
void makeCubeZRot(const int idx, const double zrot, const double size, const double x, const double y, const double z, double p1[3], double p2[3], double p3[3]) {


	const double s = size/2;
	const double co = cos(zrot);
	const double si = sin(zrot);
	
	const double x0x = co*(-s)-si*(-s)+x;
	const double x0y = si*(-s)+co*(-s)+y;
	
	const double x1x = co*(+s)-si*(-s)+x;
	const double x1y = si*(+s)+co*(-s)+y;
	
	const double x2x = co*(+s)-si*(+s)+x;
	const double x2y = si*(+s)+co*(+s)+y;
	
	const double x3x = co*(-s)-si*(+s)+x;
	const double x3y = si*(-s)+co*(+s)+y;

	switch(idx) {
		case 0: { // front face
					p1[0] = x0x; p1[1] = x0y; p1[2] = z-s; 
					p2[0] = x1x; p2[1] = x1y; p2[2] = z-s; 
					p3[0] = x1x; p3[1] = x1y; p3[2] = z+s; 
					break;
				}
		case 1: { // front face
					p1[0] = x0x; p1[1] = x0y; p1[2] = z-s; 
					p2[0] = x1x; p2[1] = x1y; p2[2] = z+s; 
					p3[0] = x0x; p3[1] = x0y; p3[2] = z+s; 
					break;
				}
		case 2: { // right face
					p1[0] = x1x; p1[1] = x1y; p1[2] = z-s; 
					p2[0] = x1x; p2[1] = x1y; p2[2] = z+s; 
					p3[0] = x2x; p3[1] = x2y; p3[2] = z-s; 
					break;
				}
		case 3: { // right face
					p1[0] = x1x; p1[1] = x1y; p1[2] = z+s; 
					p2[0] = x2x; p2[1] = x2y; p2[2] = z-s; 
					p3[0] = x2x; p3[1] = x2y; p3[2] = z+s; 
					break;
				}
		case 4: { // left face
					p1[0] = x0x; p1[1] = x0y; p1[2] = z-s; 
					p2[0] = x0x; p2[1] = x0y; p2[2] = z+s; 
					p3[0] = x3x; p3[1] = x3y; p3[2] = z-s; 
					break;
				}
		case 5: { // left face
					p1[0] = x0x; p1[1] = x0y; p1[2] = z+s; 
					p2[0] = x3x; p2[1] = x3y; p2[2] = z-s; 
					p3[0] = x3x; p3[1] = x3y; p3[2] = z+s; 
					break;
				}
		case 6: { // rear face
					p1[0] = x3x; p1[1] = x3y; p1[2] = z-s; 
					p2[0] = x2x; p2[1] = x2y; p2[2] = z-s; 
					p3[0] = x2x; p3[1] = x2y; p3[2] = z+s; 
					break;
				}
		case 7: { // rear face
					p1[0] = x3x; p1[1] = x3y; p1[2] = z-s; 
					p2[0] = x2x; p2[1] = x2y; p2[2] = z+s; 
					p3[0] = x3x; p3[1] = x3y; p3[2] = z+s; 
					break;
				}
		case 8: { // top face
					p1[0] = x3x; p1[1] = x3y; p1[2] = z+s; 
					p2[0] = x0x; p2[1] = x0y; p2[2] = z+s; 
					p3[0] = x1x; p3[1] = x1y; p3[2] = z+s; 
					break;
				}
		case 9: { // top face
					p1[0] = x3x; p1[1] = x3y; p1[2] = z+s; 
					p2[0] = x1x; p2[1] = x1y; p2[2] = z+s; 
					p3[0] = x2x; p3[1] = x2y; p3[2] = z+s; 
					break;
				}
		case 10: { // bottom face
					p1[0] = x3x; p1[1] = x3y; p1[2] = z-s; 
					p2[0] = x0x; p2[1] = x0y; p2[2] = z-s; 
					p3[0] = x1x; p3[1] = x1y; p3[2] = z-s; 
					break;
				}
		case 11: { // bottom face
					p1[0] = x3x; p1[1] = x3y; p1[2] = z-s; 
					p2[0] = x1x; p2[1] = x1y; p2[2] = z-s; 
					p3[0] = x2x; p3[1] = x2y; p3[2] = z-s; 
					break;
				}

	}


}

bool collideSpheres(const double x1, const double y1, const double z1, const double r1,
        const double x2, const double y2, const double z2, const double r2) {
    const double dxx = x1-x2;
    const double dyy = y1-y2;
    const double dzz = z1-z2;

    if ((dxx*dxx + dyy*dyy + dzz*dzz) <= (r1+r2)*(r1+r2)) {
        return true;
    }
    return false;
}

//#define SMALL_NUM  0.00000001 // anything that avoids division overflow
 // dot product (3D) which allows vector operations in arguments
struct Line {
    TPoint3 P0,P1;  
};

struct Vector {
    double x,y,z;
    Vector(const TPoint3 &p1, const TPoint3 &p2) {
        x = p1.x - p2.x;
        y = p1.y - p2.y;
        z = p1.z - p2.z;
    }
    Vector(double xx=0, double yy=0, double zz=0):x(xx),y(yy),z(zz) {}
    
    Vector(const Vector &rhs):x(rhs.x),y(rhs.y),z(rhs.z) {}
};

#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define norm(v)    sqrt(dot(v,v))  // norm = length of vector
#define d(u,v)     norm(u-v)       // distance = norm of difference
#define abs(x)     ((x) >= 0 ? (x) : -(x))   // absolute value

double dist3D_Line_to_Line(
        const TPoint3 &P1, const TPoint3 &P2,
        const TPoint3 &P3, const TPoint3 &P4) {

    // line 1 = P1, P2
    // line 2 = P3, P4

//    Vector   u = L1.P1 - L1.P0;
//    Vector   v = L2.P1 - L2.P0;
//    Vector   w = L1.P0 - L2.P0;
//    Vector u(L1.P1,L1.P0);
//    Vector v(L2.P1,L2.P0);
//    Vector w(L1.P0,L2.P0);
    Vector u(P2, P1);
    Vector v(P4, P3);
    Vector w(P1, P3);

    double    a = dot(u,u);        // always >= 0
    double    b = dot(u,v);
    double    c = dot(v,v);        // always >= 0
    double    d = dot(u,w);
    double    e = dot(v,w);
    double    D = a*c - b*b;       // always >= 0
    double    sc, tc;
    const double SMALL_NUM = 0.00000001;
    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) {         // the lines are almost parallel
        sc = 0.0;
        tc = (b>c ? d/b : e/c);   // use the largest denominator
    }
    else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    // get the difference of the two closest points
//    Vector   dP = w + (sc * u) - (tc * v);  // = L1(sc) - L2(tc)
    double dpX = w.x + sc*u.x - tc*v.x;
    double dpY = w.y + sc*u.y - tc*v.y;
    double dpZ = w.z + sc*u.z - tc*v.z;
    Vector dP(dpX,dpY,dpZ);

    return norm(dP);   // return the closest distance
}


// dist3D_Segment_to_Segment():
 //    Input:  two 3D line segments S1 and S2
 //    Return: the shortest distance between S1 and S2
double dist3D_Segment_to_Segment(
        const TPoint3 &P1, const TPoint3 &P2,
        const TPoint3 &P3, const TPoint3 &P4) 
{
    // segment1 = p1, p2
    // segment2 = p3, p4
//    Vector   u = S1.P1 - S1.P0;
//    Vector   v = S2.P1 - S2.P0;
//    Vector   w = S1.P0 - S2.P0;
    
    Vector u(P2,P1);
    Vector v(P4,P3);
    Vector w(P1,P3);

    double    a = dot(u,u);        // always >= 0
    double    b = dot(u,v);
    double    c = dot(v,v);        // always >= 0
    double    d = dot(u,w);
    double    e = dot(v,w);
    double    D = a*c - b*b;       // always >= 0
    double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    const double SMALL_NUM = 0.00000001;

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    // get the difference of the two closest points
//    Vector   dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)
    double dpX = w.x + sc*u.x - tc*v.x;
    double dpY = w.y + sc*u.y - tc*v.y;
    double dpZ = w.z + sc*u.z - tc*v.z;

    Vector dP(dpX, dpY, dpZ);
    const double dist = norm(dP);   // return the closest distance
//    WDEBUG("Dist=" << dist );
    return dist;
}
                                                                                                         

#undef dot
#undef norm
#undef d
#undef abs


std::vector<double> getDimension(const std::vector<TPoint> &p) {
	std::vector<double> result(4,0.0);
	bool first = true;

	for(int i=0;i<(int)p.size();i++) {
		if (first) {
			result[0] = p[i].x;
			result[1] = p[i].x;
			result[2] = p[i].y;
			result[3] = p[i].y;
			first = false;
		} else {
			result[0] = std::min<double>(result[0], p[i].x);
			result[1] = std::max<double>(result[1], p[i].x);
			result[2] = std::min<double>(result[2], p[i].y);
			result[3] = std::max<double>(result[3], p[i].y);
		}
	}
	return result;
}


std::vector<double> getDimensionPolygons(const std::vector< std::vector< TPoint > > &p) {
	std::vector<double> result(4,0.0);
	bool first = true;
	for(int i=0;i<(int)p.size();i++) {
		std::vector<double> tmp(getDimension(p[i]));
		if (first) {
			result = tmp;
			first = false;
		} else {
			result[0] = std::min(result[0],tmp[0]);
			result[1] = std::max(result[1],tmp[1]);
			result[2] = std::min(result[2],tmp[2]);
			result[3] = std::max(result[3],tmp[3]);
		}
		tmp.clear();
	}
	return result;
}

/// for each segment in polygon, return it's lenght
/// polygon: p0,p1,..,pn
/// result:  l1=0,l1,..,ln
/// l0 = lenght of (p0,p1), l1 = length(p1,p2), ... , ln = length(pn,p0)

std::vector<double> getLengths(const std::vector<TPoint> &pts, const bool polygon) {
	std::vector<double> result;
	result.reserve(pts.size());
	int a = 0;
	if (!polygon) {
		a = -1;
	}
	for(int i=0;i<(int)pts.size()+a;i++) {
		result.push_back(pointDistanceEucleid(pts[i],pts[(i+1)%(int)pts.size()]));
	}
	return result;
}

/**
  * resample given polygon.
  * numOfSamples > 0
  */
std::vector<TPoint> resamplePolygon(const std::vector<TPoint> &polygon, const int numOfSamples, double &delta, const bool isPolygon) {

	std::vector<double> lengths(getLengths(polygon,isPolygon));
	const double polygonLenght = std::accumulate(lengths.begin(),lengths.end(),0.0);
	const double dl = polygonLenght / (double)numOfSamples;
	delta = dl;

	std::vector<TPoint> result;
	result.reserve(numOfSamples);

	int last = 0;
	const int size = polygon.size();
	double alength = 0;
	double tmpl;
	int lnext;
	for(double l = 0; l < polygonLenght; l+=dl) {
		while(last < size && (alength + lengths[last]) < l  ) {
			alength+=lengths[last];
			last++;
		}
		if (last == size) {
			break;
		}
		lnext = (last == (size-1)) ? 0 : (last+1);
		tmpl = l - alength;
		result.push_back(TPoint(polygon[last].x*(1-tmpl / lengths[last]) + 
					polygon[lnext].x*(tmpl/lengths[last]),
					polygon[last].y*(1-tmpl/lengths[last])+polygon[lnext].y*(tmpl/lengths[last])));		
	}

	lengths.clear();
	return result;
}

/** return center of gravity for TPoint3<> */
TPoint3 getGravityCenter(const std::vector< std::vector< TPoint3 > > &pts) {
	double cx = 0;
	double cy = 0;
	double cz = 0;
	int n = 0;
	for(int i=0;i<(int)pts.size();i++) {
		for(int j=0;j<3;j++) {
			cx += pts[i][j].x;
			cy += pts[i][j].y;
			cz += pts[i][j].z;
			n++;
		}
	}
	if (n > 0) {
		return TPoint3(cx/n,cy/n,cz/n);
	} else {
		return TPoint3(0,0,0);
	}
}


inline TPoint3 rotateTranslate(const TPoint3 &point, double rotation[3][3], double translation[3]) {
	TPoint3 p;
	p.x = rotation[0][0]*point.x + rotation[0][1]*point.y + rotation[0][2]*point.z + translation[0];
	p.y = rotation[1][0]*point.x + rotation[1][1]*point.y + rotation[1][2]*point.z + translation[1];
	p.z = rotation[2][0]*point.x + rotation[2][1]*point.y + rotation[2][2]*point.z + translation[2];
	return p;
}


std::vector< TPoint3 > rotateTranslate(const std::vector< TPoint3 > &pts, double rotation[3][3], double translation[3]) {
	std::vector< TPoint3 > res;
	for(int i=0;i<(int)pts.size();i++) {
		res.push_back(rotateTranslate(pts[i],rotation,translation));
	}
	return res;
}

std::vector< std::vector<TPoint3 > > rotateTranslate(const std::vector< std::vector<TPoint3> > &triangles, double rotation[3][3], double translation[3]) {
	std::vector< std::vector< TPoint3 > > result;
	for(int i=0;i<(int)triangles.size();i++) {
		result.push_back(rotateTranslate(triangles[i],rotation,translation));
	}
	return result;
}


double triangleArea(const std::vector<TPoint3> &triangle) {
	const double abx = triangle[1].x - triangle[0].x;
	const double aby = triangle[1].y - triangle[0].y;
	const double abz = triangle[1].z - triangle[0].z;
	
	const double acx = triangle[2].x - triangle[0].x;
	const double acy = triangle[2].y - triangle[0].y;
	const double acz = triangle[2].z - triangle[0].z;
	const double d = abx*acx + aby*acy + abz*acz;
	return 0.5*sqrt( (abx*abx + aby*aby + abz*abz)*(acx*acx+acy*acy+acz*acz) - d*d);
	
}

double triangleArea(const TPoint3 &p0, const TPoint3 &p1, const TPoint3 &p2) {
	const double abx = p1.x - p0.x;
	const double aby = p1.y - p0.y;
	const double abz = p1.z - p0.z;
	
	const double acx = p2.x - p0.x;
	const double acy = p2.y - p0.y;
	const double acz = p2.z - p0.z;
	const double d = abx*acx + aby*acy + abz*acz;
	return 0.5*sqrt( (abx*abx + aby*aby + abz*abz)*(acx*acx+acy*acy+acz*acz) - d*d);
	
}

double triangle2Area(const std::vector<TPoint> &triangle) {
	const double abx = triangle[1].x - triangle[0].x;
	const double aby = triangle[1].y - triangle[0].y;
	const double acx = triangle[2].x - triangle[0].x;
	const double acy = triangle[2].y - triangle[0].y;
	const double d = abx*acx + aby*acy;
	return 0.5*sqrt( (abx*abx + aby*aby)*(acx*acx+acy*acy) - d*d);
}

void translate(TPoint3 &p, double matrix[3][3], double vector[3]) {
	double x = matrix[0][0]*p.x + matrix[0][1]*p.y + matrix[0][2]*p.z + vector[0];
	double y = matrix[1][0]*p.x + matrix[1][1]*p.y + matrix[1][2]*p.z + vector[1];
	double z = matrix[2][0]*p.x + matrix[2][1]*p.y + matrix[2][2]*p.z + vector[2];
	p.x = x;
	p.y = y;
	p.z = z;
}


/** returns random point in polygon but tries to maximize its distance to the edges and vertices.
  * for convert polygon it should return center point 
  * if tryCenter == true, the center of the polygon is tried as a point prior to random sampling */
TPoint getLargestFreePointInPolygon(const std::vector<TPoint> &pol, const bool tryCenter) {



	TPoint p;
	std::vector<double> dim(getDimension(pol));
	if (tryCenter) {
		double sx = 0;
		double sy = 0;
		for(int i=0;i<(int)pol.size();i++) {
			sx += pol[i].x;
			sy += pol[i].y;
		}
		if (pol.size() > 0) {
			sx /= pol.size();
			sy /= pol.size();
		}
		p.x = sx;
		p.y = sy;
		if (wn_PnPoly2(p,pol) != 0) {
			return p;
		}
	}

	do {
		p.x = getRandom(dim[0],dim[1]);
		p.y = getRandom(dim[2],dim[3]);
	} while (wn_PnPoly2(p,pol) == 0);

	return p;
}


/** create plane ax+by+cz+d=0 */
void createPlane(const TPoint3 &a, const TPoint3 &b, const TPoint3 &c, double &aa, double &bb, double &cc, double &dd) {
    const double ddd = getDeterminant(a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z);
    dd = 1;
    aa = -(dd/ddd)*getDeterminant(1.,a.y,a.z,1.,b.y,b.z,1.,c.y,c.z);
    bb = -(dd/ddd)*getDeterminant(a.x,1.,a.z,b.x,1.,b.z,c.x,1.,c.z);
    cc = -(dd/ddd)*getDeterminant(a.x,a.y,1.,b.x,b.y,1.,c.x,c.y,1.);
}

/** for line determined by two points define angles for rotation x,y resp. z in RAD */
void getRotAngles(const TPoint3 &a, const TPoint3 &b, double &rotx, double &roty, double &rotz) {
    /*
	rotz = atan2(b.y-a.y,b.x-a.x);
	roty = atan2(b.z-a.z,b.x-a.x);
	rotx = atan2(b.z-a.z,b.y-a.y) + M_PI/2.0; 
    */
    
	rotz = -atan2(b.x-a.x,b.y-a.y);
	roty = 0;
	rotx = -atan2(b.y-a.y,b.z-a.z) + M_PI/2.0;
    
}

/** return point in middle of segment (a,b) */
TPoint3 getMiddlePoint(const TPoint3 &a, const TPoint3 &b) {
	return TPoint3(0.5*a.x+0.5*b.x,0.5*a.y+0.5*b.y,0.5*a.z+0.5*b.z);
}



void getMeanDev(const std::vector<double> &data, double &mean, double &dev) {
	if (data.size() == 0) {
        mean = 0;
        dev = 0;
		return;
	}

	const double sum = std::accumulate(data.begin(),data.end(),0.0);
	mean = sum / data.size();
	dev = 0;
	for(int i=0;i<(int)data.size();i++) {
		dev += (data[i] - mean)*(data[i]-mean);
	}
	dev = sqrt(dev / data.size());
}

/** returns correaltion coeeficient between two given data arrays */
double correlation(const vector<double> &a, const vector<double> &b) {
	if (a.size() == 0 || b.size()==0 || a.size() != b.size()) {
		WDEBUG("Cannot compute correlation. Data size: a.size="<<a.size() << ", b.size=" << b.size());
		return 0;
	}
	const double ma = accumulate(a.begin(),a.end(),0.0) / a.size();
	const double mb = accumulate(b.begin(),b.end(),0.0) / b.size();

	double sx = 0;
	double sy = 0;
	double s = 0;
	for(int i=0;i<(int)a.size();i++) {
		s += (a[i]-ma)*(b[i]-mb);
		sx += (a[i]-ma)*(a[i]-ma);
		sy += (b[i]-mb)*(b[i]-mb);
	}
	const double r = s / (sqrt(sx*sy));
	return r;
}


// return probability of t-test
double ttest(const std::vector<double> &a, const std::vector<double> &b) {
	if (a.size() <=1 || b.size() <=1) {
		return -1;
	}
	const double ma = std::accumulate(a.begin(),a.end(),0.0) / (double)a.size();
	const double mb = std::accumulate(b.begin(),b.end(),0.0) / (double)b.size();

	double deva = 0;
	double devb = 0;
	for(int i=0;i<(int)a.size();i++) {
		deva += (a[i] - ma)*(a[i] - ma);
	}
	deva /= (double)a.size()-1;

	for(int i=0;i<(int)b.size();i++) {
		devb += (b[i] - mb)*(b[i] - mb);
	}
	devb /= (double)b.size() - 1;

	if (a.size() == b.size()) {
		const double sasb = sqrt(0.5*(deva+devb));
		const double t = (ma-mb)/(sasb*sqrt(2.0/a.size()));
		const int df = 2*(int)a.size() - 2;
//		cerr << "t=" << t << "\n";
		const double prob = gsl_cdf_tdist_Q(fabs(t),df);
		return 2*prob;
	} else {
		// different length of vectors
		const double sasb = sqrt( ( ((double)a.size()-1)*deva + ((double)b.size()-1)*devb ) / ((double)a.size()+b.size()-2));
		const double t = (ma-mb)/(sasb*sqrt(1.0/a.size()+1.0/b.size()));
		const int df = (int)(a.size()+b.size()-2);
		const double prob = gsl_cdf_tdist_Q(fabs(t),df);
		return 2*prob;
	}
}



// slices: 0 2 3 ... -> indicates maximum of each variable in the vector
bool getNextInputPermutation(vector<int> &perm, const vector<int> &inputSlices) {
	// return true if input is valid, otherwise return false (last permutation was reached)
	
	int j = (int)perm.size()-1;
	bool p = false;
	do {
		p = false;
		perm[j]++;
		if (perm[j] > inputSlices[j]) {
			perm[j] = 0;
			j--;
			p = true;
		}
	} while (p && j >= 0);

	if (p && j < 0) {
		return false;
	}

	return true;
}

int getNumberOfCombinations(const std::vector<int> &slices, const int robotInputSize) {
	vector<int> perm(robotInputSize,0);
	int r = 0;
	while(getNextInputPermutation(perm, slices)) {
		r++;
	}
	return r;
}


vector<double> getTrianglesDim(const vector< vector<TPoint3> > &triangles) {
    vector<double> dimension(6,0);
    bool first = true;

	for(int i=0;i<(int)triangles.size();i++) {
		for(int j=0;j<(int)triangles[i].size();j++) {
			if (first) {
				dimension[0] = triangles[i][j].x;
				dimension[1] = triangles[i][j].x;
				dimension[2] = triangles[i][j].y;
				dimension[3] = triangles[i][j].y;
				dimension[4] = triangles[i][j].z;
				dimension[5] = triangles[i][j].z;
				first = false;
			} else {
				dimension[0] = std::min(dimension[0],triangles[i][j].x);
				dimension[1] = std::max(dimension[1],triangles[i][j].x);
				dimension[2] = std::min(dimension[2],triangles[i][j].y);
				dimension[3] = std::max(dimension[3],triangles[i][j].y);
				dimension[4] = std::min(dimension[4],triangles[i][j].z);
				dimension[5] = std::max(dimension[5],triangles[i][j].z);
			}
		}
    }
    return dimension;
}


vector< Triangle > oldTriangles2newTriangles(const std::vector< std::vector<TPoint> > &triangles) {
    vector< Triangle > res;
    res.reserve(triangles.size());
    for(int i=0;i<(int)triangles.size();i++) {
        Triangle t(triangles[i][0], triangles[i][1], triangles[i][2]);
        res.push_back(t);
    }
    return res;
}

vector< Triangle > oldTriangles2newTriangles(const std::vector< std::vector<TPoint3> > &triangles) {
    vector< Triangle > res;
    res.reserve(triangles.size());
    for(int i=0;i<(int)triangles.size();i++) {
        Triangle t(triangles[i][0], triangles[i][1], triangles[i][2]);
        res.push_back(t);
    }
    return res;
}

std::vector<TPoint> makeRectangle(const TPoint &center, const double width) {
    vector<TPoint> rect( 4,center);
    rect[0].x -= width;
    rect[0].y -= width;
    rect[1].x += width;
    rect[1].y -= width;
    rect[2].x += width;
    rect[2].y += width;
    rect[3].x -= width;
    rect[3].y += width;
    return rect;
}

std::vector<TPoint> makeRectangle(const TPoint &p1, const TPoint &p2) {

    std::vector<TPoint> rect;
    rect.push_back( TPoint( std::min(p1.x, p2.x), std::min(p1.y, p2.y) ) );
    rect.push_back( TPoint( std::max(p1.x, p2.x), std::min(p1.y, p2.y) ) );
    rect.push_back( TPoint( std::max(p1.x, p2.x), std::max(p1.y, p2.y) ) );
    rect.push_back( TPoint( std::min(p1.x, p2.x), std::max(p1.y, p2.y) ) );
    return rect;

}



void saveTriangles2D(const std::vector< std::vector< TPoint > > &triangles, const char *filename) {
    ofstream ofs(filename);
    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0; j < (int)triangles[i].size(); j++) {
            ofs << triangles[i][j].x << " " << triangles[i][j].y << " ";
        }
        ofs << "\n";
    }
    ofs.close();
}

void saveTriangles2D(const std::vector< Triangle > &triangles, const char *filename) {
    ofstream ofs(filename);
    for(int i=0;i<(int)triangles.size();i++) {
        ofs << triangles[i].a.x << " " << triangles[i].a.y << " ";
        ofs << triangles[i].b.x << " " << triangles[i].b.y << " ";
        ofs << triangles[i].c.x << " " << triangles[i].c.y;
        ofs << "\n";
    }
    ofs.close();
}


// Check the given point are perpendicular to x or y axis
static bool IsPerpendicular(const TPoint &pt1, const TPoint &pt2, const TPoint &pt3) {
	const double yDelta_a= pt2.y - pt1.y;
	const double xDelta_a= pt2.x - pt1.x;
	const double yDelta_b= pt3.y - pt2.y;
	const double xDelta_b= pt3.x - pt2.x;
    //WDEBUG("a,a,b,b" << yDelta_a << " "<<  xDelta_a << " " << yDelta_b << " " <<  xDelta_b);

	// checking whether the line of the two pts are vertical
	if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
//		WDEBUG("The points are pependicular and parallel to x-y axis\n");
		return false;
	}

	if (fabs(yDelta_a) <= 0.0000001){
//		TRACE(" A line of two point are perpendicular to x-axis 1\n");
		return true;
	}
	else if (fabs(yDelta_b) <= 0.0000001){
//		TRACE(" A line of two point are perpendicular to x-axis 2\n");
		return true;
	}
	else if (fabs(xDelta_a)<= 0.000000001){
//		TRACE(" A line of two point are perpendicular to y-axis 1\n");
		return true;
	}
	else if (fabs(xDelta_b)<= 0.000000001){
//		TRACE(" A line of two point are perpendicular to y-axis 2\n");
		return true;
	}
    //WDEBUG("#@$");
	return false ;
}


// return valid circle center (c,y) and radius from threee points. if radius == -1, then the circle cannot be determined (collinear points etc..)
static void CalcCircle(const TPoint &pt1, const TPoint &pt2, const TPoint &pt3, double &cx, double &cy, double &radius) {
    const double yDelta_a= pt2.y - pt1.y;
    const double xDelta_a= pt2.x - pt1.x;
    const double yDelta_b= pt3.y - pt2.y;
    const double xDelta_b= pt3.x - pt2.x;

    if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
        cx= 0.5*(pt2.x + pt3.x );
        cy= 0.5*(pt1.y + pt2.y );
        radius = pointDistanceEucleid(TPoint(cx,cy), pt1);
    }

    // IsPerpendicular() assure that xDelta(s) are not zero
    const double aSlope=yDelta_a/xDelta_a; //
    const double bSlope=yDelta_b/xDelta_b;
    if (fabs(aSlope-bSlope) <= 0.000000001){	// checking whether the given points are colinear.
        //TRACE("The three pts are colinear\n");
        WDEBUG("points are col");
        radius = -1;
    }

    // calc center
    cx= (aSlope*bSlope*(pt1.y - pt3.y) + bSlope*(pt1.x + pt2.x) - aSlope*(pt2.x+pt3.x) )/(2* (bSlope-aSlope) );
    cy = -1*(cx - (pt1.x+pt2.x)/2)/aSlope +  (pt1.y+pt2.y)/2;
    radius = pointDistanceEucleid(TPoint(cx,cy), pt1);
}



/** based on:
  http://paulbourke.net/geometry/circlesphere/Circle.cpp
  C++ code implemented as MFC (MS Foundation Class) supplied by Jae Hun Ryu. Circle.cpp, Circle.h.
  */
void circleFromPoints(const TPoint &pt1, const TPoint &pt2, const TPoint &pt3, double &x, double &y, double &radius) {

    radius = -1;

    if (!IsPerpendicular(pt1, pt2, pt3) ) {
        CalcCircle(pt1, pt2, pt3,x,y,radius);
    } 	else if (!IsPerpendicular(pt1, pt3, pt2) ) {
        CalcCircle(pt1, pt3, pt2,x,y,radius);
    } 	else if (!IsPerpendicular(pt2, pt1, pt3) ) {
        CalcCircle(pt2, pt1, pt3, x,y,radius);
    } 	else if (!IsPerpendicular(pt2, pt3, pt1) ) {
        CalcCircle(pt2, pt3, pt1, x,y,radius);
    } 	else if (!IsPerpendicular(pt3, pt2, pt1) )	{ 
        CalcCircle(pt3, pt2, pt1, x,y,radius);
    } else if (!IsPerpendicular(pt3, pt1, pt2) ) {
        CalcCircle(pt3, pt1, pt2, x,y,radius);
    }

}


} // namespace



