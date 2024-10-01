#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

#include "CRobot2D.h"
#include "types.h"
#include "ut.h"
#include "rapid/RAPID.H"
#include "WLog.h"

/** general class for handling a 2D robot. 
  x = [x,y,phi]
  * the shape is loaded from TRI file
  */
namespace rrtPlanning {

using namespace std;

static const int _distanceDimension = 3;


CRobot2D::CRobot2D(const char *filename,const double gScale, const State &x0):x(x0) {
    geomScale = gScale;
    rapidModel = NULL;


    WDEBUG("Loading 2D robot from " << filename << ", geom scale is " << geomScale);

    robotName = std::string(filename);	

    if (robotName.find(".tri") != std::string::npos) {
        loadTriangles(filename, _triangles);
        //centerTriangles(_triangles);
        setRapidModel(_triangles);
    } else if (robotName.find(":I") != std::string::npos) {
        int sizex, sizey;
        sscanf(robotName.c_str(), ":I%dx%d", &sizex, &sizey);
        WDEBUG("String is " << robotName << ", sizex=" << sizex << ", sizey = " << sizey);
        robotShapeI(sizex, sizey, _triangles);
        WDEBUG("Building I-shape robot");
        setRapidModel(_triangles);
    } else if (robotName.find(":L") != std::string::npos) {
        int sizex, sizey, thick;
        sscanf(robotName.c_str(), ":L%dx%dx%d", &sizex, &sizey, &thick);
        WDEBUG("String is " << robotName << ", sizex=" << sizex << ", sizey=" << sizey << ", thickness=" << thick);
        robotShapeL(sizex, sizey,thick, _triangles);
        WDEBUG("Building L-shape robot");
        setRapidModel(_triangles);
    } else if (robotName.find(":C") != std::string::npos) {
        int radius, num;
        sscanf(robotName.c_str(), ":C%dx%d", &radius, &num);
        WDEBUG("String is " << robotName << ", radius= " << radius <<", num triangles " << num);
        robotShapeC(radius, num, _triangles);
        WDEBUG("Building Circle-shape robot");
        setRapidModel(_triangles);
    }


    WDEBUG("CRobot2D: _triangles.size=" << _triangles.size());
    
    distanceDimension = _distanceDimension;
    distanceTopology = new int[distanceDimension];
    distanceScale = new MPNN::ANNcoord[distanceDimension];

    for(int i=0;i<distanceDimension;i++) {
        distanceTopology[i] = 1;
        distanceScale[i] = 0.2;
    }
    WDEBUG("robotType: CRobot2D: " << __PRETTY_FUNCTION__ );
}

double CRobot2D::partialDistance(const State &a, const State &b, const int dimension) const {
    return fabs(a[dimension] - b[dimension])*distanceScale[dimension];
}

void CRobot2D::setDimensionScale(const int idx, const double scale) {
    if (idx >= 0 && idx < distanceDimension) {
        distanceScale[idx] = scale;
        WWDEBUG("dimension[ " << idx << "] scale = " << scale);
    } else {
        WWDEBUG("Cannot set scale for dimension " << idx << ", distanceDimension=" << distanceDimension);
    }
}


double CRobot2D::distance(const State &s) const {
	return distance(s,x);
}


double CRobot2D::distance(const State &s1, const State &s2) const {
	// distance betwen two states
	const double dx = s1[0] - s2[0];
	const double dy = s1[1] - s2[1];
    if (_distanceDimension == 2) {
    	return distanceScale[0]*distanceScale[0]*dx*dx + distanceScale[1]*distanceScale[1]*dy*dy; // accordin to mpnn2/multiann.h
    } else {
    	const double dp = s1[2] - s2[2];
    	return distanceScale[0]*distanceScale[0]*dx*dx + distanceScale[1]*distanceScale[1]*dy*dy + distanceScale[2]*distanceScale[2]*dp*dp;
    }
}




CRobot2D::~CRobot2D() {
    x.clear();
    _triangles.clear();
    rapidModel = NULL;
}

void CRobot2D::setRapidModel() {
    setRapidModel(_triangles);
}

void CRobot2D::setRapidModel(const std::vector< Triangle > &triangles) {

    if (rapidModel) {
        delete rapidModel;
    }

    rapidModel = new RAPID_model;
    rapidModel->BeginModel();
    double p1[3], p2[3], p3[3];

    _triangles = triangles;


    for(int i=0;i<(int)triangles.size();i++) {
        p1[0] = triangles[i].a.x;
        p1[1] = triangles[i].a.y;
        p1[2] = 0;				

        p2[0] = triangles[i].b.x;
        p2[1] = triangles[i].b.y;
        p2[2] = 0;

        p3[0] = triangles[i].c.x;
        p3[1] = triangles[i].c.y;
        p3[2] = 0;
        rapidModel->AddTri(p1,p2,p3,i);	
    }
    rapidModel->EndModel();
}

/** count sum of areas of robot's triangles. If they do not overlap,  the sum can be considered as the 'surface' (=2D volume) of the robot
  */
double CRobot2D::getRobotArea() const {
    double s = 0;
    for(int i=0;i<(int)_triangles.size();i++) {
        //s += triangle2Area(_triangles[i]);
        s += _triangles[i].area();
    }
    return s;
}


void CRobot2D::loadTriangles(const char *filename) {
    loadTriangles(filename, _triangles);
}

void CRobot2D::loadTriangles(const char *filename, vector< Triangle > &triangles) {

    WDEBUG("Loading 2D robot from " << filename << ", geom scale is " << geomScale);
    // triangulated version of point above	

    robotName = std::string(filename);	
    std::ifstream ifs(filename);
    string line;
    int triIdx = 0;
    triangles.clear();
    
    int lineIdx = 1;
    while(ifs) {
        std::getline(ifs,line);
        vector<double> vd(lineToNum<double>(line));
        if (vd.size() == 6) {
            TPoint a(geomScale*vd[0],geomScale*vd[1]);
            TPoint b(geomScale*vd[2],geomScale*vd[3]);
            TPoint c(geomScale*vd[4],geomScale*vd[5]);
            Triangle t(a,b,c);

            if (t.area() < 0.001) {
                WDEBUG("Triangle on line " << lineIdx << " has small area: " << t.area());
            } else {
                triangles.push_back(t);
            }
        }
        lineIdx++;
    }
    ifs.close();
    WDEBUG("Loaded " << triangles.size() << " triangles from " << filename << " for robot shape\n");
}

/* change points of all triangles if the points are further than 'maxDist' from the center
   * maxDist is in map units
   */
void CRobot2D::cutTriangles(std::vector< Triangle > &triangles, const TPoint &center, const double maxDist) const {
    WWDEBUG("not implemented yet! ");
    exit(0);
    /*
    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0;j<(int)triangles[i].size();j++) {
            const double dist = pointDistanceEucleid(center, triangles[i][j]);
            if (dist > maxDist) {
                const double t = maxDist / dist;
                triangles[i][j].x = (1-t)*center.x + t*triangles[i][j].x;
                triangles[i][j].y = (1-t)*center.y + t*triangles[i][j].y;
            }
        }
    }
    */
}

/** scale triangles */
void CRobot2D::scaleTriangles(std::vector< Triangle > &triangles, const double scale) const {

    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0;j<3;j++) {
            triangles[i][j].x *= scale;
            triangles[i][j].y *= scale;
        }
    }
}

void CRobot2D::centerTriangles() {
    centerTriangles(_triangles);
}

void CRobot2D::centerTriangles(std::vector< Triangle > &triangles) const {
    WWDEBUG("not implemented yet! ");
    exit(0);
   /* 
    TPoint c( getCenter(triangles ) );

    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0;j<(int)triangles[i].size();j++) {
            triangles[i][j].x -= c.x;
            triangles[i][j].y -= c.y;
        }
    }
    */

}


/* return geometric center of the given triangles */
TPoint CRobot2D::getCenter(const std::vector< Triangle > &triangles) const {

    TPoint c;
    if (triangles.size() == 0) {
        return c;
    }

    int cnt = 0;
    for(int i=0;i<(int)triangles.size();i++) {

        c.x += triangles[i].a.x + triangles[i].b.x + triangles[i].c.x;
        c.y += triangles[i].a.y + triangles[i].b.y + triangles[i].c.y;
        cnt+=3;
    }
    c.x /= cnt;
    c.y /= cnt;
    return c;
}





CRobot2D::CRobot2D(const CRobot2D &rhs):
	x(rhs.x),
		rapidModel(rhs.rapidModel),
		distanceDimension(rhs.distanceDimension),
		_triangles(rhs._triangles)
	{
        distanceTopology = new int[distanceDimension];
        distanceScale = new MPNN::ANNcoord[distanceDimension];
        const int *rhst = rhs.getDistanceTopology();
        const MPNN::ANNcoord *rhss = rhs.getDistanceScale();
        for(int i=0;i<distanceDimension;i++) {
            distanceTopology[i] = rhst[i];
            distanceScale[i] = rhss[i];
        }	
	
}





void CRobot2D::setState(const State &sx) {
	x = sx;
}


CRobot2D::State CRobot2D::getState() const {
	return x;
}

int CRobot2D::getStateSize() const {
	return 3;
}



vector< Triangle > CRobot2D::getShape() const {
	return getShape(x);
}



vector< Triangle > CRobot2D::getShape(const State &s) const {

    vector< Triangle > t;
    t.reserve(_triangles.size());

    for(int i=0;i<(int)_triangles.size();i++) {
        t.push_back(transformTriangle(i, s));
    }
    return t;

}


vector<TPoint> CRobot2D::getShapePoints(const State &s) const {
	return std::vector<TPoint>();
}


vector<TPoint> CRobot2D::getShapePoints() const {
	return getShapePoints(x);
}


TPoint CRobot2D::getRefPoint(const State &s) const {
	return TPoint(s[0],s[1]);
}

std::vector<TPoint> CRobot2D::getRefPoint(const std::vector<State> &states) const {
    std::vector<TPoint> res;
    res.reserve(states.size());
    for(int i=0;i<(int)states.size();i++) {
        res.push_back(getRefPoint(states[i]));
    }
    return res;
}

TPoint CRobot2D::getRefPoint() const {
	return getRefPoint(x);
}



CRobot2D::State CRobot2D::getRandomState(const vector<double> &mapDimension) const {
	State r(getStateSize(),0.0);
	r[0] = getRandom(mapDimension[0],mapDimension[1]);
	r[1] = getRandom(mapDimension[2],mapDimension[3]);
	//r[2] = getRandom(-2*M_PI,,2*M_PI);
	r[2] = getRandom(-M_PI,M_PI);
	return r;
}


CRobot2D::State CRobot2D::getRandomState(const SDimension &mapDimension) const {
	State r(getStateSize(),0.0);
    mapDimension.getRandomPoint(r[0], r[1]);
//	r[2] = getRandom(0,2*M_PI);
	r[2] = getRandom(-M_PI,M_PI);
	return r;
}

void CRobot2D::states2pts(const std::vector<State> &states, vector<TPoint> &pts) const {
    pts.clear();
    pts.reserve(states.size());
    for(int i=0;i<(int)states.size();i++) {
        pts.push_back(getRefPoint(states[i]));
    }
}



bool CRobot2D::isValidState() const {
	return isValidState(x);
}


bool CRobot2D::isValidState(const State &s) const {
	return true;
}



RAPID_model *CRobot2D::getRapidModel(double matrix[3][3], double vector[3]) const {
	return getRapidModel(matrix,vector,x);	
}


RAPID_model *CRobot2D::getRapidModel(double matrix[3][3], double vector[3], const State &s) const {
	const double si = sin(s[2]);
	const double co = cos(s[2]);
	matrix[0][0] = co; matrix[0][1] = -si; matrix[0][2] = 0;
	matrix[1][0] = si; matrix[1][1] = co; matrix[1][2] = 0;
	matrix[2][0] = 0; matrix[2][1] = 0; matrix[2][2] = 1;
	vector[0] = s[0];
	vector[1] = s[1];
	vector[2] = 0;
	
	return rapidModel;
}

Triangle CRobot2D::transformTriangle(const int triangleIdx, const State &s) const {
    double matrix[3][3];

	const double si = sin(s[2]);
	const double co = cos(s[2]);

	matrix[0][0] = co; matrix[0][1] = -si; matrix[0][2] = 0;
	matrix[1][0] = si; matrix[1][1] = co; matrix[1][2] = 0;
	matrix[2][0] = 0; matrix[2][1] = 0; matrix[2][2] = 1;

    Triangle t(_triangles[triangleIdx]);

    for(int j=0;j<3;j++) { // transforming tree points of t
        const double x = matrix[0][0]*t[j].x + matrix[0][1]*t[j].y + s[0];
        const double y = matrix[1][0]*t[j].x + matrix[1][1]*t[j].y + s[1];
        t[j].x = x;
        t[j].y = y;
    }

    return t;	
}





const int *CRobot2D::getDistanceTopology() const {
	return distanceTopology;
}


int CRobot2D::getDistanceDim() const {
	return distanceDimension;
}


const MPNN::ANNcoord *CRobot2D::getDistanceScale() const {
	return distanceScale;
}



void CRobot2D::robotShapeI(const double sizex, const double sizey,  std::vector< Triangle > &triangles) {

    const double x2 = sizex / 2;
    const double y2 = sizey / 2;

    triangles.clear();

    triangles.push_back( Triangle() );
    triangles.back().a = TPoint3(-x2,-y2);
    triangles.back().b = TPoint3(x2,-y2);
    triangles.back().c = TPoint3(x2,y2);
    
    triangles.push_back( Triangle() );
    triangles.back().a = TPoint3(x2,y2);
    triangles.back().b = TPoint3(-x2,y2);
    triangles.back().c = TPoint3(-x2,-y2);




}

void CRobot2D::robotShapeL(const double sizex, const double sizey, const double thickness,  std::vector< Triangle > &triangles) {

    const double x2 = sizex / 2;
    const double y2 = sizey / 2;

    triangles.clear();

    {
        TPoint3 a(0,0);
        TPoint3 b(sizex,0);
        TPoint3 c(sizex,thickness);
        triangles.push_back( Triangle(a,b,c) );
    }

    {
        TPoint3 a(sizex,thickness);
        TPoint3 b(0,thickness);
        TPoint3 c(0,0);
        triangles.push_back( Triangle(a,b,c) );
    }

    {
        TPoint3 a(0,thickness);
        TPoint3 b(thickness,thickness);
        TPoint3 c(thickness,thickness+sizey);
        triangles.push_back( Triangle(a,b,c) );
    }  

    { 
        TPoint3 a(0,thickness);
        TPoint3 b(thickness,thickness+sizey);
        TPoint3 c(0,thickness+sizey);
        triangles.push_back( Triangle(a,b,c) );
    }

    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0;j<3;j++) {
            triangles[i][j].x -= thickness/2;
            triangles[i][j].y -= thickness/2;
        }
    }
}


void CRobot2D::robotShapeC(const double radius, const int num,  std::vector< Triangle > &triangles) {

    triangles.clear();

    vector<TPoint> pts;
    for(int i=0;i<num;i++) {
        const double a = 2*M_PI*i/num;
        const double x = radius*cos(a);
        const double y = radius*sin(a);

        const double a1 = 2*M_PI*(i+1)/num;
        const double x1 = radius*cos(a1);
        const double y1 = radius*sin(a1);

        TPoint3 aa(0,0,0);
        TPoint3 b(x,y,0);
        TPoint3 c(x1,y1,0);
        triangles.push_back( Triangle(aa,b,c) );
    }


}




} // namespace





