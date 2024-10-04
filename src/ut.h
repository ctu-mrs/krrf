#ifndef _RUT_H__
#define _RUT_H__

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <ostream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <math.h>
#include "types.h"
#include "WLog.h"
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <typeinfo>

#include "rapid/RAPID.H"
#include "multiann.h"
#include "ANN.h"
#include "CStat.h"


namespace rrtPlanning {


std::vector<double> getTrianglesDim(const std::vector< std::vector<TPoint3> > &triangles);

std::vector< std::vector< TPoint > > box2triangles(const TPoint &from, const TPoint &to);


void saveTriangles(const std::vector< Triangle > &triangles, const char *filename);

void printPercentStatus(const double actual, const double max, double &old, 
        const std::string &prefix=std::string(""),
        std::ostream &os=std::cerr);



double ttest(const std::vector<double> &a, const std::vector<double> &b);


void makeCube(const int idx, const double size, const double x, const double y, const double z, double p1[3], double p2[3], double p3[3]);

void makeCubeZRot(const int idx, const double zrot, const double size, const double x, const double y, const double z, double p1[3], double p2[3], double p3[3]);


bool collideSpheres(const double x1, const double y1, const double z1, const double r1,
        const double x2, const double y2, const double z2, const double r2);

template<typename Iter>
void print(Iter begin, Iter end, const std::string delim = std::string(" "),
	   const std::string endDelim = std::string(), std::ostream &os = std::cerr) {

	Iter beforEnd = end;
	--beforEnd;

	for(Iter i = begin; i != end; ++i) {
		os << *i;
		if (i != beforEnd) {
			os << delim;
		}
	}
	os << endDelim;
}


template<typename Iter>
std::string printString(Iter begin, Iter end, const std::string delim = std::string(" "),
	   const std::string endDelim = std::string()) {

	std::stringstream ss;
	Iter beforEnd = end;
	--beforEnd;

	for(Iter i = begin; i != end; ++i) {
		ss << *i;
		if (i != beforEnd) {
			ss << delim;
		}
	}
	ss << endDelim;
	return ss.str();
}


template<typename Containter>
std::string printString(const Containter &c, const std::string delim = std::string(" "),
	   const std::string endDelim = std::string()) {
	return printString(c.begin(),c.end(),delim,endDelim);
}

template<typename Containter>
void print(const Containter &c, const std::string delim = std::string(" "),
		const std::string endDelim = std::string(), std::ostream &os = std::cerr) {
	print(c.begin(),c.end(),delim,endDelim,os);
}

template<typename T>
std::string printString(T *array, const int size, const std::string delim = std::string(" "),
	   const std::string endDelim = std::string()) {

	std::stringstream ss;
    for(int i=0;i<size;i++) {
        ss << array[i] << delim;
    }
	ss << endDelim;
	return ss.str();

}

template<typename T>
inline void limit(T &var, const T min, const T max) {
	if (var < min) {
		var = min;
	}
	if (var > max) {
		var = max;
	}
}	

template<typename T>
std::vector<T> lineToNum(const std::string &line) {

	std::vector<T> res;
	std::stringstream iss(line);
	T a;
	while(iss >> a) {
		res.push_back(a);
	}
	return res;
}


template<typename Iter>
std::vector<double> getDimensionI(Iter begin, Iter end) {
	std::vector<double> result(4,0.0);
	bool first = true;

	for(Iter i = begin; i != end; i++) {
		if (first) {
			result[0] = i->x;
			result[1] = i->x;
			result[2] = i->y;
			result[3] = i->y;
			first = end;
		} else {
			result[0] = std::min(result[0], i->x);
			result[1] = std::max(result[1], i->x);
			result[2] = std::min(result[2], i->y);
			result[3] = std::max(result[3], i->y);
		}
	}
	return result;
}


std::vector<double> getDimension(const std::vector<TPoint> &p);

std::vector<double> getDimensionPolygons(const std::vector< std::vector< TPoint > > &p);


template<typename Iter>
std::string toString(Iter begin, Iter end, const std::string delim = std::string(" "), const std::string endDelim = std::string()) {

	std::stringstream ss;

	Iter beforEnd = end;
	--beforEnd;

	for(Iter i = begin; i != end; ++i) {
		ss << *i;
		if (i != beforEnd) {
			ss << delim;
		}
	}
	ss << endDelim;

	return ss.str();
}

template<typename Containter>
std::string toString(const Containter &c, const std::string delim = std::string(" "), const std::string endDelim = std::string()) {
	return toString(c.begin(),c.end(),delim,endDelim);
}

void removeComment(std::string &s, const char commentChar);

double getTime(struct rusage one, struct rusage two);

void getTime(struct rusage *t);


std::string timeStr();

std::vector<double> getStats(const std::vector<double> &data);



template<typename NT>
inline NT normalizeMPP(NT angle) {
	if (angle > 2*M_PI) {
		angle = angle - floor(angle/(2*M_PI))*2*M_PI;
	} else if (angle < -2*M_PI) {
		angle = fabs(angle);
		angle = angle - floor(angle/(2*M_PI))*2*M_PI;
		angle = -angle;
	}
	return angle;
}

template<typename NT>
NT normalize02PI(NT angle) {
    if (angle > 2*M_PI) {
        angle -= 2*M_PI;
    }
    if (angle < 0) {
        angle += 2*M_PI;
    }
    return angle;
}

// we assume that angle is in degrees
template<typename NT>
NT normalize02PI_degrees(NT angle) {
    if (angle >= 360) {
        angle -= 360;
    }
    if (angle < 0) {
        angle += 360;
    }
    return angle;
}


std::vector<double> getLengths(const std::vector<TPoint> &pts, const bool polygon);
std::vector<TPoint> resamplePolygon(const std::vector<TPoint> &polygon, const int numOfSamples, double &delta, const bool isPolygon);


template<typename T>
T multiply(const T &data, const double b) {
	T res;
	for(int i=0;i<(int)data.size();i++) {
		res.push_back(data[i]*b);
	}
	return res;
}

/** add two vectors */
template<typename T>
T add(const T &a, const T &b) {
	T res;
	for(int i=0;i<(int)a.size();i++) {
		res.push_back(a[i]+b[i]);
	}
	return res;
}

/** subtract a - b */
template<typename T>
T subb(const T &a, const T &b) {
	T res(a);
	for(int i=0;i<(int)a.size();i++) {
		res[i] = a[i] - b[i];
	}
	return res;	
}



TPoint3 getGravityCenter(const std::vector< const std::vector< TPoint3 > > &pts);

inline TPoint3 rotateTranslate(const TPoint3 &point, double rotation[3][3], double translation[3]);

std::vector< TPoint3 > rotateTranslate(const std::vector< TPoint3 > &pts, double rotation[3][3], double translation[3]);

std::vector< std::vector<TPoint3 > > rotateTranslate(const std::vector< std::vector<TPoint3> > &triangles, double rotation[3][3], double translation[3]);

double triangleArea(const std::vector<TPoint3> &triangle);
double triangleArea(const TPoint3 &p0, const TPoint3 &p1, const TPoint3 &p2);


double triangle2Area(const std::vector<TPoint> &triangle);

/** return states on straight line made by states s1 and s2. Number of states is 'num' 
  */
template<typename ST>
std::vector< ST > approximateStates2(const ST &s1, const ST &s2, const int num) {

	std::vector<ST> result;
	ST tmp(s1);
	for(int i=0;i<num;i++) {
		const double t = i*1.0/(double)num;
		for(int j=0;j<(int)s1.size();j++) {
			tmp[j] = (1-t)*s1[j] + t*s2[j];
		}
		result.push_back(tmp);
	}
	result.push_back(s2);
	return result;
}






template<typename ST>
void approximateStates(const ST &s1, const ST &s2, const int num, std::vector<ST> &result) {

	ST tmp(s1);
    
    result.clear();
    result.reserve(num+2);

    const int s1size = (int)s1.size();

	for(int i=0;i<num;i++) {
		const double t = i*1.0/(double)num;
		for(int j=0;j<s1size;j++) {
			tmp[j] = (1-t)*s1[j] + t*s2[j];
		}
		result.push_back(tmp);
	}
	result.push_back(s2);
}



void translate(TPoint3 &p, double matrix[3][3], double vector[3]);

TPoint getLargestFreePointInPolygon(const std::vector<TPoint> &pol, const bool tryCenter);


void convertQuaternionToRotation(double &rx, double &ry, double &rz, const double q0, const double q1, const double q2, const double q3);

void rotateVector(double &x, double &y, double &z, const double rx, const double ry, const double rz);
bool file_exists(const char *filename);


/** returns determinan
  *       | a b c |
  * det = | d e f |
  *       | g h i |
  */
template<typename T>
T getDeterminant(const T a, const T b, const T c, const T d, const T e, const T f, const T g, const T h, const T i) {
	return a*e*i+b*f*g+c*d*h-c*e*g-a*f*h-b*d*i;
}


double dist3D_Line_to_Line(
        const TPoint3 &P1, const TPoint3 &P2,
        const TPoint3 &P3, const TPoint3 &P4);


double dist3D_Segment_to_Segment(
        const TPoint3 &P1, const TPoint3 &P2,
        const TPoint3 &P3, const TPoint3 &P4);


void getRotAngles(const TPoint3 &a, const TPoint3 &b, double &rotx, double &roty, double &rotz);

TPoint3 getMiddlePoint(const TPoint3 &a, const TPoint3 &b);

void getMeanDev(const std::vector<double> &data, double &mean, double &dev);
double correlation(const std::vector<double> &a, const std::vector<double> &b);


void createPlane(const TPoint3 &a, const TPoint3 &b, const TPoint3 &c, double &aa, double &bb, double &cc, double &dd);



bool getNextInputPermutation(std::vector<int> &perm, const std::vector<int> &inputSlices);

int getNumberOfCombinations(const std::vector<int> &slices, const int robotInputSize);




// contains integer grid representing the map, for counting coverage of area
struct TGrid {
    
    TGrid(const std::vector<double> &mapDimension, const double res):
        dim(mapDimension), resolution(res) {
        
            sx = (int)lround( (mapDimension[1] - mapDimension[0]) / res);
            sy = (int)lround( (mapDimension[3] - mapDimension[2]) / res);

            std::vector<int> tmpy(std::vector<int>(sy,0));
            grid = std::vector< std::vector<int> >(sx, tmpy);
            //WDEBUG("Created grid of " << sx << " x " << sy << " cells");
        }


    void zero() {
        for(int i=0;i<sx;i++) {
            for(int j = 0; j < sy; j++) {
                grid[i][j] = 0;
            }
        }
    }

    double getPercentage() const {
        int cnt = 0;
        for(int i=0;i<sx;i++) {
            for(int j = 0; j < sy; j++) {
                if (grid[i][j] > 0) {
                    cnt++;
                }
            }
        }
        return 1.0*cnt / (1.0*sx*sy);
    }

    void increment(const double x, const double y, const int val = 1) {
        int cx = (x - dim[0]) / resolution;
        int cy = (y - dim[2]) / resolution;

        if (cx < 0 ) { cx = 0; }
        if (cx >= sx) { cx = sx-1;}
        if (cy < 0) { cy = 0; }
        if (cy >= sy) { cy = sy-1; }
        grid[cx][cy]+=val;
    }

    void saveGrid(const char *filename) const {
        std::ofstream ofs(filename);
        ofs << sx << " " << sx << " " << printString(dim) << "\n";
        for(int i=0;i<(int)grid.size();i++) {
            for(int j =0; j < (int)grid[i].size();j++) {
                if (grid[i][j] != 0) {
                    ofs << i << " " << j << " " << grid[i][j] << "\n";
                }
            }
        }
        ofs.close();
    }

    //normalize to 0,1, where 0=0 and 1=255, normalize so MAX is 1
    void normalizeGridMax(std::vector< std::vector<int> > &result) const {
        result = grid;
        for(int i=0;i<(int)result.size();i++) {
            for(int j = 0; j < (int)result[i].size(); j++) {
                result[i][j] = 0;
            }
        }

        int c = 0;
        int max = -1;
        for(int i = 0; i<(int)grid.size();i++) {
            for(int j=0;j<(int)grid[i].size();j++) {
                c += grid[i][j];
                if (grid[i][j] > max || max == -1) {
                    max = grid[i][j];
                }
            }
        }
        std::cerr << "Normalize grid: c=" << c << ", max=" << max << "\n";

        for(int i = 0; i<(int)result.size();i++) {
            for(int j=0;j<(int)result[i].size();j++) {
                //result[i][j] = (int)lround(255.0*grid[i][j] / (1.0*c));
                result[i][j] = (int)lround(255.0*grid[i][j] / (1.0*max));
                if (result[i][j] < 0) { result[i][j] = 0; }
                if (result[i][j] > 255) { result[i][j] = 255; } 
            }
        }
    }

    //normalize to 0,1, where 0=0 and 1=255, normalize so SUM is 1
    void normalizeGridSum(std::vector< std::vector<int> > &result) const {
        result = grid;
        for(int i=0;i<(int)result.size();i++) {
            for(int j = 0; j < (int)result[i].size(); j++) {
                result[i][j] = 0;
            }
        }

        int c = 0;
        int max = -1;
        for(int i = 0; i<(int)grid.size();i++) {
            for(int j=0;j<(int)grid[i].size();j++) {
                c += grid[i][j];
                if (grid[i][j] > max || max == -1) {
                    max = grid[i][j];
                }
            }
        }
        std::cerr << "Normalize grid: c=" << c << ", max=" << max << "\n";

        for(int i = 0; i<(int)result.size();i++) {
            for(int j=0;j<(int)result[i].size();j++) {
                result[i][j] = (int)lround(255.0*grid[i][j] / (1.0*c));
                //result[i][j] = (int)lround(255.0*grid[i][j] / (1.0*max));
                if (result[i][j] < 0) { result[i][j] = 0; }
                if (result[i][j] > 255) { result[i][j] = 255; } 
            }
        }
    }



    std::vector<double> dim;
    double resolution;
    std::vector< std::vector<int > > grid;
    int sx,sy;
};


/** calcucate input vector to get from state s1 to state s2 over time dt, so that:
  s2 = s1 + input*dt
  it is assumed that input IS CREATED AND HAS PROPER size
  */
template<typename ST, typename IN>
void inputFromStates(const ST &s1, const ST &s2, const double dt, IN &in, const double edgeTime) {
    for(int i=0;i<(int)in.size();i++) {
        in[i] = (s2[i] - s1[i]) / edgeTime;
    }
}


template<typename ST, typename RT>
std::vector<TPoint> states2pts2(const std::vector<ST> &states, const RT &robot) {
    vector<TPoint> pts;
    pts.reserve(states.size());
    for(int i=0;i<(int)states.size();i++) {
        pts.push_back(robot.getRefPoint(states[i]));
    }
    return pts;
}

template<typename ST, typename RT>
std::vector<TPoint3> states2pts3(const std::vector<ST> &states, const RT &robot) {
    vector<TPoint3> pts;
    pts.reserve(states.size());
    for(int i=0;i<(int)states.size();i++) {
        pts.push_back(robot.getRefPoint(states[i]));
    }
    return pts;
}

std::vector< Triangle > oldTriangles2newTriangles(const std::vector< std::vector<TPoint> > &triangles);
std::vector< Triangle > oldTriangles2newTriangles(const std::vector< std::vector<TPoint3> > &triangles);


template<typename State, typename Robot >
std::vector<State> resamplePath(const std::vector< State > &path, Robot &robot, const double delta) {

	vector<double> lengths;
    lengths.reserve(path.size());
    double length;

    for(int i=0;i<(int)path.size()-1;i++) {
        const double d = sqrt(robot.distance( path[i], path[i+1] ));
        lengths.push_back(d);
        length += d;
    }

	const int numOfSamples = (int)lround(length / delta);

    vector<State> result;

	result.reserve(numOfSamples+1);

	int last = 0;
	const int size = path.size();
	double alength = 0;
	double tmpl;
	int lnext;
	for(double l = 0; l < length; l+=delta) {
		while((alength + lengths[last]) < l  && last < size) {
			alength+=lengths[last];
			last++;
		}
		if (last >= size) {
			break;
		}
        lnext = last+1;
        if (lnext >= size) {
            lnext = size-1;
        }

		tmpl = l - alength;

        State tmp(robot.getStateSize(),0);

        for(int i=0;i<(int)tmp.size();i++) {
            const double t = 1.0*tmpl / lengths[last];
            tmp[i] = path[last][i]*(1-t) + path[lnext][i]*t;
        }
        result.push_back(tmp);

	}

	return result;
}

template<typename T>
double avg(const std::vector<T> &data) {
    double sum = 0;
    for(int i=0;i<(int)data.size();i++) {
        sum += data;
    }
    return sum / data.size();
}


std::vector<TPoint> makeRectangle(const TPoint &center, const double width);
std::vector<TPoint> makeRectangle(const TPoint &p1, const TPoint &p2);


void saveTriangles2D(const std::vector< std::vector< TPoint > > &triangles, const char *filename);
void saveTriangles2D(const std::vector< Triangle > &triangles, const char *filename);

void circleFromPoints(const TPoint &pt1, const TPoint &pt2, const TPoint &pt3, double &x, double &y, double &radius);

} // namespace;

#endif


