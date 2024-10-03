#ifndef _RTYPES_H__
#define _RTYPES_H__

#include <math.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include "gui/CPainters.h"
#include "Voronoi.h"
#include "random.h"

namespace rrtPlanning {

class TPoint {
	public:
		TPoint(const double ix = 0, const double iy = 0): x(ix),y(iy) {}
        TPoint &operator=(const TPoint &rhs) {
            if (&rhs != this) {
                x = rhs.x;
                y = rhs.y;
            }
            return *this;
        }
		friend std::ostream &operator<<(std::ostream &os, const TPoint &r);
		double x,y;

};

class TPoint3 {
	public:
		TPoint3(const double ix=0, const double iy=0, const double iz=0):x(ix),y(iy),z(iz){}
		friend std::ostream &operator<<(std::ostream &os, const TPoint3 &r);
		double x,y,z;
};

inline double squaredPointDistanceEucleid(const TPoint &p1, const TPoint &p2) {
	return ((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}



inline double pointDistanceEucleid(const TPoint &p1, const TPoint &p2) {
	return sqrt(squaredPointDistanceEucleid(p1,p2));
}


inline double squaredPointDistanceEucleid(const TPoint3 &p1, const TPoint3 &p2) {
	return ((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}

inline double pointDistanceEucleid(const TPoint3 &p1, const TPoint3 &p2) {
	return sqrt(squaredPointDistanceEucleid(p1,p2));
}



class Triangle {
    public:
        Triangle() {}
        Triangle(const TPoint3 &aa, const TPoint3 &bb, const TPoint3 &cc):a(aa),b(bb), c(cc) {}
        Triangle(const TPoint &aa, const TPoint &bb, const TPoint &cc) {
            a.x = aa.x;
            a.y = aa.y;
            a.z = 0;
            b.x = bb.x;
            b.y = bb.y;
            b.z = 0;
            c.x = cc.x;
            c.y = cc.y;
            c.z = 0;
        }
        Triangle(const Triangle &rhs):a(rhs.a), b(rhs.b), c(rhs.c) {}

        TPoint3 &operator[](const int idx) { 
            if (idx == 0) {
                return a;
            } else if (idx == 1) {
                return b;
            } else if (idx == 2) {
                return c;
            }
            std::cerr << " wrong operator[" << idx << "] for triangle !\n";
            exit(0);
            return a;
        }

        const TPoint3 &operator[](const int idx) const { 
            if (idx == 0) {
                return a;
            } else if (idx == 1) {
                return b;
            } else if (idx == 2) {
                return c;
            }
            std::cerr << " wrong operator[" << idx << "] for triangle !\n";
            exit(0);
            return a;
        }

        double lengthAB() const {
            return pointDistanceEucleid(a,b);
        }
        double lengthBC() const {
            return pointDistanceEucleid(b,c);
        }
        double lengthAC() const {
            return pointDistanceEucleid(a,c);
        }
        TPoint3 midPoint(const TPoint3 &p1, const TPoint3 &p2) const {
            return TPoint3( (p1.x+p2.x)/2., (p1.y+p2.y) / 2.0, (p1.z+p2.z)/2.0);
        }

        // divide triangle along longest edge
        void divide(Triangle &t1, Triangle &t2) const {
            const double ab = lengthAB();
            const double ac = lengthAC();
            const double bc = lengthBC();
            if (ab >= max(ac,bc)) {
                // split by ab
                t1.a = a;
                t1.b = midPoint(a,b);
                t1.c = c;
                t2.a = midPoint(a,b);
                t2.b = b;
                t2.c = c;
            } else if (ac > max(ab,bc)) {
                // split by ac
                t1.a = a;
                t1.b = b;
                t1.c = midPoint(a,c);
                t2.a = midPoint(a,c);
                t2.b =b;
                t2.c = c;
            } else {
                // split by bc
                t1.a = a;
                t1.b = b;
                t1.c = midPoint(b,c);
                t2.a = a;
                t2.b = midPoint(b,c);;
                t2.c = c;
            }
        }




        TPoint3 a,b,c;

    double area() const  {
        const double abx = b.x - a.x;
        const double aby = b.y - a.y;
        const double acx = c.x - a.x;
        const double acy = c.y - a.y;
        const double d = abx*acx + aby*acy;
        return 0.5*sqrt( (abx*abx + aby*aby)*(acx*acx+acy*acy) - d*d);
}



};




class TLight {
    public:
        TLight(const TPoint3 &_p, const bool _shadowless=false, const bool _energy=1.0):p(_p), shadowless(_shadowless), energy(_energy) {}
        TLight(const TLight &l): p(l.p), shadowless(l.shadowless), energy(l.energy) {}
        TPoint3 p;
        bool shadowless;
        double energy;

        std::string toString() const {
            std::stringstream ss;
            ss << "light_source {<" << p.x <<","<<p.y<<","<<p.z<<"> color White ";
            if (shadowless) {
                ss << " shadowless ";
            }
            ss << "}\n";
            return ss.str();
        }
};


std::ostream &operator<<(std::ostream &os, const TPoint3 &r);
std::ostream &operator<<(std::ostream &os, const TPoint &r);
TPoint rotatePointR(const TPoint &p, const double alpha);
TPoint translatePointR(const TPoint &p, const double tx, const double ty);


struct TCircle {
	double x,y,r;
	TCircle(const double xx, const double yy, const double rr):x(xx),y(yy),r(rr) {}
	TCircle(const TCircle &rhs):x(rhs.x),y(rhs.y),r(rhs.r) {}
	TPoint getPoint() const {
		return TPoint(x,y);
	}
};

// for searching in 2d graphs
struct SEdge {
    SEdge(const int f, const int t, const double c = -1):from(f),to(t),cost(c) {}
    SEdge(const SEdge &rhs): from(rhs.from),to(rhs.to),cost(rhs.cost) {}

    int from, to;
    double cost; // negative cost means that the edge does not exists
};

struct TAreaOfInterest {
    TPoint p1,p2; // p1 is left low, p2 is upper right
    double resx, resy;
    std::vector< std::vector<int> > grid;
    std::vector< TPoint > guidingPath;

    TAreaOfInterest(const TPoint a, const TPoint b): p1(a), p2(b), resx(-1), resy(-1) {
        checkPoints();
    }
    TAreaOfInterest(const TAreaOfInterest &rhs): p1(rhs.p1), p2(rhs.p2), resx(rhs.resx), resy(rhs.resy), grid(rhs.grid),
        guidingPath(rhs.guidingPath) {
        checkPoints();
    }

    ~TAreaOfInterest() {
        grid.clear();
    }

    void checkPoints() {
        if (! (p1.x < p2.x && p1.y < p2.y)) {
            TPoint tmp(p1);
            p1 = p2;
            p2 = tmp;
        }
    }

    std::vector<TPoint> getPolygon() const {
        std::vector<TPoint> res;
        res.push_back(p1);
        res.push_back(TPoint(p2.x, p1.y));
        res.push_back(p2);
        res.push_back(TPoint(p1.x, p2.y));
        return res;
    }

    TPoint getCenter() const {
        return TPoint((p1.x+p2.x)/2.0, (p1.y+p2.y)/2.0);
    }

    bool isPointIn(const TPoint &p) const {
        return ( (p.x >= p1.x) && (p.x <= p2.x) && (p.y >= p1.y) && (p.y <= p2.y) );
    }
    
    bool isPointIn(const double x, const double y) const {
        return ( (x >= p1.x) && (x <= p2.x) && (y >= p1.y) && (y <= p2.y) );
    }

    void createGrid(const int resolutionx, const int resolutiony, const int initValue = 0) {
        resx = resolutionx;
        resy = resolutiony;
        const int sizex = (int)ceil((p2.x - p1.x)/resx);
        const int sizey = (int)ceil((p2.y - p1.y)/resy);
        if (sizex < 0 || sizey < 0) {
            std::cerr << "Cannot create the grid, sizex=" << sizex <<", sizey=" << sizey << "\n";
            exit(0);
        }
        grid = std::vector< std::vector<int> >(sizex, std::vector<int>(sizey,initValue));
    }

    void setGridToValue(const int value) {
        for(int i=0;i<(int)grid.size();i++) {
            for(int j=0;j<(int)grid[i].size();j++) {
                grid[i][j] = value;
            }
        }
    }

    // return true if point lie inside and return its coordinates as indices to the grid created in previous run of createGrid
    void gridCoordinates(const double x, const double y, const double resx, const double resy, int &gx, int &gy) const {
        gx = (int)lround((x - p1.x) / resx);
        gy = (int)lround((y - p1.y) / resy);
//        return isPointIn(x,y);
    }

    double getArea() const {
        return (fabs(p1.x-p2.x)*fabs(p1.y-p2.y));
    }

    std::vector<double> getDimension() const {
        std::vector<double> res(4,0);
        res[0] = p1.x;
        res[1] = p2.x;
        res[2] = p1.y;
        res[3] = p2.y;
        return res;
    }

};



void rotatePoint(TPoint &p, const double alpha);



void rotatePoint(TPoint &p, const double alpha, const TPoint &center);
double distancePointEucleidSquared(const TPoint &a, const TPoint &b);
double distancePointEucleid(const TPoint &a, const TPoint &b);


TPoint transformPointRT_R(const TPoint &p, const double dx, const double dy, const double alpha);
TPoint transformPointTR_R(const TPoint &p, const double dx, const double dy, const double alpha);

void translatePoint(TPoint &p, const double tx, const double ty);


template<typename Iter>
void transformRT(Iter begin, Iter end, const double tx, const double ty, const double alpha) {
    for(Iter i = begin; i != end; i++) {
		rotatePoint(*i,alpha);
		translatePoint(*i,tx,ty);
	}
}

double getAnglePoints(const TPoint &p1, const TPoint &p2);


template<typename T>
typename T::value_type inputDiff(const T &a, const T &b) {
	if (a.size() != b.size()) {
		return 10000;
	}
	
	typename T::value_type d = 0;

	for(int i=0;i<(int)a.size();i++) {
		d+= (typename T::value_type)fabs(a[i] - b[i]);
	}

	return d;
}

template<typename T>
bool isEqual(const T &a, const T &b) {
	for(int i=0;i<(int)a.size();++i) {
		if (fabs(a[i] - b[i]) > 0.001) {
			return false;
		}
	}
	return true;
}


double getAngleRad(const TPoint &p1, const TPoint &p2, const TPoint &p3);


struct isSamePoint{
	inline bool operator()(const TPoint &p1, const TPoint &p2) const {
		return p1.x == p2.x && p1.y == p2.y;
	}
};


enum TCPG {
	SINUSCPG = 1000,
	NONLINEARCPG,
    HOPFCPG,
	CPG_NONE
};

std::string getCPGName(const TCPG cpgtype);
std::string getCPGAllNames();


enum TShapeType {
    PTS_ONLY,
    LINE_ONLY,
    PTS_AND_LINE,
    POLY_ONLY,
    PTS_AND_POLY
};

struct TShape {

    TShape() {}

    TShape(const std::vector<TPoint> &points, const TShapeType _type=PTS_ONLY):pts(points),type(_type) {}
    TShape(const TShape &rhs):pts(rhs.pts),type(rhs.type),ptsColor(rhs.ptsColor),ptsFillColor(rhs.fillColor),
        lineColor(rhs.lineColor),fillColor(rhs.fillColor),
        ptsSize(rhs.ptsSize), ptsLineWidth(rhs.ptsLineWidth), lineWidth(rhs.lineWidth) {}

    std::vector<TPoint> pts;
    TShapeType type;
    CPainters::CColor ptsColor, ptsFillColor, lineColor, fillColor;
    int ptsSize, ptsLineWidth, lineWidth;

};

std::vector<TPoint> getVoronoiSegments(Voronoi::VoronoiDiagramGenerator &vdg);
std::vector< std::vector<TPoint > > getVoronoiCells(Voronoi::VoronoiDiagramGenerator &vdg);



template <typename RobotType>
struct TRobotVersion {
    RobotType *robot;
    double scale;
    std::string name;

    TRobotVersion(const TRobotVersion &rhs): robot(rhs.robot),scale(rhs.scale), name(rhs.name) {}
    TRobotVersion(RobotType *r, double s, const std::string &n):robot(r), scale(s), name(n) {}
    void swap(TRobotVersion &other) {
        double tmp = scale;
        scale = other.scale;
        other.scale = tmp;
        RobotType *tmpr = robot;
        robot = other.robot;
        other.robot = tmpr;
        std::string tmps = name;
        name = other.name;
        other.name = tmps;
    }
};


struct SDimension {
    SDimension(): min(0,0,0), max(0,0,0) {}
    SDimension(const double minx, const double maxx, const double miny, const double maxy, const double minz, const double maxz):
        min(minx,miny,minz),max(maxx,maxy,maxz) {}
    SDimension(const std::vector<double> &dim) {
        if (dim.size() == 6) {
            min.x = dim[0];
            max.x = dim[1];
            min.y = dim[2];
            max.y = dim[3];
            min.z = dim[4];
            max.z = dim[5];
        }
    }
    SDimension(const SDimension &rhs):min(rhs.min), max(rhs.max) {}

    void clear() {
        min = TPoint3(0,0,0);
        max = TPoint3(0,0,0);
    }

    SDimension &operator=(const SDimension &rhs) {
        if (&rhs != this) {
            min = rhs.min;
            max = rhs.max;
        }
        return *this;
    }

    double getSize(const int dim) const {
        if (dim == 0) {
            return max.x - min.x;
        } else if (dim == 1) {
            return max.y - min.y;
        } else {
            return max.z - min.z;
        } 
    }


    std::vector<double> getVector() const {
        std::vector<double> v(6,0);
        v[0] = min.x;
        v[1] = max.x;
        v[2] = min.y;
        v[3] = max.y;
        v[4] = min.z;
        v[5] = max.z;
        return v;
    }

    bool isIn(const TPoint3 &p) const {
        if (p.x >= min.x && p.x <= max.x &&
                p.y >= min.y && p.y <= max.y &&
                p.z >= min.z && p.z <= max.z) {
            return true;
        }
        return false;
    }


    bool isIn(const double x, const double y, const double z) const {
        if (x >= min.x && x <= max.x &&
                y >= min.y && y <= max.y &&
                z >= min.z && z <= max.z) {
            return true;
        }
        return false;
    }


    bool isIn(const TPoint &p) const {
        if (p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y) {
            return true;
        }
        return false;

    }

    bool isIn(const double x, const double y) const {
        if (x >= min.x && x <= max.x && y >= min.y && y <= max.y) {
            return true;
        }
        return false;
    }


    void update(const Triangle &t) {
        update(t.a);
        update(t.b);
        update(t.c);
    }


    // change dimension if the point is outside
    void update(const TPoint3 &p) {
        if (p.x < min.x) {
            min.x = p.x;
        }
        if (p.y < min.y) {
            min.y = p.y;
        }
        if (p.z < min.z) {
            min.z = p.z;
        }
        if (p.x > max.x) {
            max.x = p.x;
        }
        if (p.y > max.y) {
            max.y = p.y;
        }
        if (p.z > max.z) {
            max.z = p.z;
        }
    }

    void update(const TPoint &p) {
        if (p.x < min.x) {
            min.x = p.x;
        }
        if (p.y < min.y) {
            min.y = p.y;
        }
        if (p.x > max.x) {
            max.x = p.x;
        }
        if (p.y > max.y) {
            max.y = p.y;
        }
    }


    // set the dimension according to one point (usefull for initialization
    void setToPoint(const double x, const double y, const double z) {
        min.x = x;
        min.y = y;
        min.z = z;
        max.x = x;
        max.y = y;
        max.z = z;
    }
    

    void setToPoint(const TPoint3 &p) {
        min.x = p.x;
        min.y = p.y;
        min.z = p.z;
        max.x = p.x;
        max.y = p.y;
        max.z = p.z;
    }

    // set the dimension according to one point (usefull for initialization
    void setToPoint(const TPoint &p) {
        min.x = p.x;
        min.y = p.y;
        max.x = p.x;
        max.y = p.y;
    }


    void addToMin(const TPoint3 &p) {
        min.x += p.x;
        min.y += p.y;
        min.z += p.z;
    }

    void addToMax(const TPoint3 &p) {
        max.x += p.x;
        max.y += p.y;
        max.z += p.z;
    }

    // shrink all dimensions - so the min points is increased and max point is descreased
    void shrink(const double scale1, const double scale2, const double scale3) {
        const double lx = (max.x - min.x)/2.0;
        const double ly = (max.y - min.y)/2.0;
        const double lz = (max.z - min.z)/2.0;
        TPoint3 center(min.x+lx,min.y+ly,min.z+lz);
        min.x = center.x - lx*scale1;
        min.y = center.y - ly*scale2;
        min.z = center.z - lz*scale3;

        max.x = center.x + lx*scale1;
        max.y = center.y + ly*scale2;
        max.z = center.z + lz*scale3;
    }

    double volume() const {
        return (max.x - min.x)*(max.y-min.y)*(max.z-min.z);
    }

    // random point in the dimension
    TPoint3 getRandomPoint() const {
        const double rx = getRandom(min.x, max.x);
        const double ry = getRandom(min.y, max.y);
        const double rz = getRandom(min.z, max.z);
        return TPoint3(rx,ry,rz);
    }

    void getRandomPoint(double &x, double &y, double &z) const {
        x = getRandom(min.x, max.x);
        y = getRandom(min.y, max.y);
        z = getRandom(min.z, max.z);
    }

    void getRandomPoint(double &x, double &y) const {
        x = getRandom(min.x, max.x);
        y = getRandom(min.y, max.y);
    }

    TPoint getRandomPoint2() const {
        double x,y;
        getRandomPoint(x,y);
        return TPoint(x,y);
    }

    void getRandomPoint(double &x, double &y, gsl_rng *r) const {
        x = getRandom(min.x, max.x,r);
        y = getRandom(min.y, max.y,r);
    }




    void borderLine(vector< TPoint > &pts) const {
        pts.clear();
        pts.push_back( TPoint(min.x, min.y ) );
        pts.push_back( TPoint(max.x, min.y ) );
        pts.push_back( TPoint(max.x, max.y ) );
        pts.push_back( TPoint(min.x, max.y ) );
    }

    TPoint3 getCenter() const {
        return TPoint3( (min.x + max.x) / 2., (min.y + max.y) / 2., (min.z + max.z) / 2.);
    }

    TPoint getCenter2D() const {
        return TPoint( (min.x + max.x) / 2., (min.y + max.y) / 2.);

    }

    TPoint somePoint(const double tx, const double ty) const {
        const double px = min.x*(1-tx) + max.x*tx;
        const double py = min.y*(1-ty) + max.y*ty;
        return TPoint(px,py);
    }



    TPoint3 min, max;

};

std::ostream &operator<<(std::ostream &os, const SDimension &dim);


void makeBorderTriangles(const std::vector<TPoint> &pts, std::vector< std::vector< TPoint > > &triangles, const double borderWidth);

class SamplingPrimitive {
    public:
    // for dimension2: poiints = [x1,y1, ... x3,y3]
    SamplingPrimitive(const int dimension) {
        _dimension = dimension;
        _numPts = 3; // just for testing
        x = vector<double>(_dimension*_numPts, 0);
    }

    SamplingPrimitive(const SamplingPrimitive &rhs):_dimension(rhs._dimension), _numPts(rhs._numPts), x(rhs.x) {}

    void setPoint(const vector<double> &pt, const int index) {
        if (pt.size() != _dimension) {
            std::cerr << "cannot setup " << index << "-th point for bezier!";
            exit(0);
        }
        for(int i=0;i<_dimension;i++) {
            x[index*_dimension+i] = pt[i];
        }
    }


    void getValue(vector<double> &res, const double t) const {
        
        for(int i=0;i<_dimension;i++) {
            const double val = (1-t)* ( (1-t)*x[_dimension*0+i]+t*x[_dimension*(1)+i]) + t*((1-t)*x[_dimension*1+i] + t*x[_dimension*2+i]  );
            res[i] = val;
        }
        //std::cerr << "val("<< t << ") is " << printString(x) << "\n";

    }

    int _dimension, _numPts;
    vector<double> x; // representation of the primitive [x11,x12, ..x1_dim1, ... xnpts,1,xnpts,2, ... x_npts,dim]

};

struct Graph2 {
    Graph2(int _size) {
        size = _size;

        for(int i=0;i<size;i++) {
            edges.push_back(vector<int>());
        }
    }

    void dfs(int actual, int dest, vector<bool> &visited, vector<int> &path) {
        visited[actual] = true;
        path.push_back(actual);

        if (actual == dest) {
            allPaths.push_back(path);
        } else {
            for(int i=0; i <(int)edges[actual].size();i++) {
                if (visited[edges[actual][i]] == false) {
                    dfs(edges[actual][i], dest, visited, path);            
                }
            }
        }
        path.pop_back();
        visited[actual] = false;
    }


    void getAllPath(const int from, const int to, vector< vector<int> > &result) {


        vector<bool> visited(size, false);
        vector<int> path;
        dfs(from, to, visited, path);
        result = allPaths;

    }

    vector< vector<int> > edges;
    vector< vector<int> > allPaths;
    int size;
};


void subdivide(const std::vector<Triangle> &triangles, std::vector<Triangle> &result, const double maxArea);


} // namespace 


#endif

