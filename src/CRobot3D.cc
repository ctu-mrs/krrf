#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

#include "ut.h"
#include "CRobot3D.h"
#include "types.h"
#include "rapid/RAPID.H"
#include "WLog.h"

/** general class for handlig any 3D robot. The movement is
  * the shape is loaded from TRI3 file
  */
namespace rrtPlanning {

using namespace std;


CRobot3D::CRobot3D(const char *filename, const double scalex, const double scaley, const double scalez):
   _scalex(scalex), 
   _scaley(scaley), 
   _scalez(scalez)
    {

    x = vector<double>(6,0);

    areaThreshold = -0.0001;

    //load triangles
    rapidModel = NULL;
    loadTriangles(filename, areaThreshold, scalex, scaley, scalez,_triangles);
    setRapidModel(_triangles);
/*
    _RWx = 0.4;
    _RWy = 0.4;
    _RWz = 0.4;
  */  
    _RWx = 1;
    _RWy =1;
    _RWz = 1;

    distanceDimension = 3;
    distanceTopology = new int[distanceDimension];
    distanceScale = new MPNN::ANNcoord[distanceDimension];

    if (distanceDimension == 3) {
        _RWx = 0;
        _RWy = 0;
        _RWz = 0;
    }


    for(int i=0;i<distanceDimension;i++) {
        distanceTopology[i] = 1;
        distanceScale[i] = 1.0;
        if (i == 3) {
            distanceScale[i] = _RWx;
        }
        if (i == 4) {
            distanceScale[i] = _RWy;
        }
        if (i == 5) {
            distanceScale[i] = _RWz;
        }
    }
    WDEBUG("rw(x,y,z)="<<_RWx <<","<<_RWy << "," << _RWz);
}

CRobot3D::~CRobot3D() {
    x.clear();
    _triangles.clear();
    rapidModel = NULL;
}

CRobot3D::CRobot3D(const CRobot3D &rhs):
        x(rhs.x),
        rapidModel(rhs.rapidModel),
        baseTPoint3s(rhs.baseTPoint3s),
        distanceDimension(rhs.distanceDimension),
        _triangles(rhs._triangles) {

        distanceTopology = new int[distanceDimension];
        distanceScale = new MPNN::ANNcoord[distanceDimension];
        const int *rhst = rhs.getDistanceTopology();
        const MPNN::ANNcoord *rhss = rhs.getDistanceScale();
        for(int i=0;i<distanceDimension;i++) {
            distanceTopology[i] = rhst[i];
            distanceScale[i] = rhss[i];
        }	
}



double CRobot3D::partialDistance(const State &a, const State &b, const int dimension) const {
    return fabs(a[dimension] - b[dimension])*distanceScale[dimension];
}


void CRobot3D::translateModel(const TPoint3 &newOrigin) {
/* translate triangles so the 'newOrigin' becomes (0,0,0) of the model and rebuild rapid for it */
    
    vector< Triangle > newTriangles(_triangles);

    for(int t=0;t < (int)newTriangles.size();t++) {
        for(int i=0;i<3;i++) {
            newTriangles[t][i].x += newOrigin.x;
            newTriangles[t][i].y += newOrigin.y;
            newTriangles[t][i].z += newOrigin.z;
        }
    }
    setRapidModel(newTriangles);
}

void CRobot3D::setRapidModel(const vector< Triangle > &triangles) {
    if (rapidModel) {
        delete rapidModel;
    }

    rapidModel = new RAPID_model;
    rapidModel->BeginModel();
    double p1[3], p2[3], p3[3];

    for(int i=0;i<(int)triangles.size();i++) {
        p1[0] = triangles[i][0].x;
        p1[1] = triangles[i][0].y;
        p1[2] = triangles[i][0].z;

        p2[0] = triangles[i][1].x;
        p2[1] = triangles[i][1].y;
        p2[2] = triangles[i][1].z;

        p3[0] = triangles[i][2].x;
        p3[1] = triangles[i][2].y;
        p3[2] = triangles[i][2].z;
            
        rapidModel->AddTri(p1,p2,p3,i);	
    }
    rapidModel->EndModel();

    _triangles = triangles;
}





double CRobot3D::getMaxTriangleDistance(const vector< Triangle > &triangles) const {

    double result = -1;
    
    TPoint3 p(0,0,0);

    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0;j<3;j++) {
            const double d = squaredPointDistanceEucleid(triangles[i][j],p);
            if (d > result || result == -1) {
                result = d;
            }
        }
    }
    if (result <= 0) {
        WDEBUG("Cannot compute getMaxTriangleDistance. triangles.size=" << triangles.size());
        exit(0);
    }
    result = sqrt(result);
    return result;

}

/** cut points of the triangles so thay have maximal distance 'd' from the origin of the coordinate system.
  this assumes, that the robot is geometrically centered in the origin 
  */
void CRobot3D::cutToDistance(vector< Triangle > &triangles, const double maxDistance) const {
    
    TPoint3 p(0,0,0);
    double maxd = -1;

    int pts = 0;

    for(int i=0;i<(int)triangles.size();i++) {
        for(int j=0;j<3;j++) {
            const double d = pointDistanceEucleid(triangles[i][j],p);
            if (d > maxd) {
                maxd = d;
            }
            if (d > maxDistance) {
                const double t = maxDistance/d;
                triangles[i][j].x = p.x*(1-t) + t*triangles[i][j].x;
                triangles[i][j].y = p.y*(1-t) + t*triangles[i][j].y;
                triangles[i][j].z = p.z*(1-t) + t*triangles[i][j].z;
                pts++;
            }
        }
    }
    WDEBUG("Max distance of the 3D mesh: " << maxd);
    WDEBUG("Total pts: " << (int)(triangles.size()*3) << ", " << pts << " updated to maxdist=" << maxDistance);
}


void CRobot3D::loadTriangles(const char *filename, 
        const double areaThreshold,
        const double scalex, const double scaley, const double scalez,
        vector < Triangle > &triangles)  {

    WDEBUG("Loading robot shape from " << filename <<", areaThreshold = " << areaThreshold);
    robotName = std::string(filename);	
    string line;
    triangles.clear();
    int lineIdx = 1;
    int discarded = 0;


    ifstream ifs(filename);
    WDEBUG("Robot geom scale is " << scalex << " " << scaley << " " << scalez);

    while(ifs) {
        std::getline(ifs,line);
        vector<double> vd(lineToNum<double>(line));

        if (vd.size() == 9) {
            for(int i=0;i<3;i++) {
                vd[3*i+0] *=scalex;
                vd[3*i+1] *=scaley;
                vd[3*i+2] *=scalez;
            }

            TPoint3 a(vd[0],vd[1],vd[2]);
            TPoint3 b(vd[3],vd[4],vd[5]);
            TPoint3 c(vd[6],vd[7],vd[8]);
            Triangle t(a,b,c);
            if (t.area() < areaThreshold) {
                discarded++;
            } else {
                triangles.push_back(t);
            }
        } else if (vd.size() == 12) {
            for(int i=0;i<4;i++) {
                vd[3*i+0] *=scalex;
                vd[3*i+1] *=scaley;
                vd[3*i+2] *=scalez;
            }
            // we have 4-tupple as input
            {
                TPoint3 a(vd[0],vd[1],vd[2]);
                TPoint3 b(vd[3],vd[4],vd[5]);
                TPoint3 c(vd[6],vd[7],vd[8]);
                Triangle t(a,b,c);
                if (t.area() < areaThreshold) {
                    discarded++;
                } else {
                    triangles.push_back(t);
                }
            }
            {
                TPoint3 a(vd[3],vd[4],vd[5]);
                TPoint3 b(vd[6],vd[7],vd[8]);
                TPoint3 c(vd[9],vd[10],vd[11]);
                Triangle t(a,b,c);
                if (t.area() < areaThreshold) {
                    discarded++;
                } else {
                    triangles.push_back(t);
                }
            }
        } else if (vd.size() != 0) {
            WWDEBUG("Line " << lineIdx << " in " << filename << " contains " << vd.size() << " number, but it should have 9 or 12 numbers!");
            exit(0);
        }
        lineIdx++;
        vd.clear();
    }
    WDEBUG("Loaded " << triangles.size() << " triangles from " << (lineIdx-1) << " for robot shape, discarded " << discarded << " triangles");
}








void CRobot3D::setState(const State &sx) {
    x = sx;
    }


CRobot3D::State CRobot3D::getState() const {
    return x;
    }


int CRobot3D::getStateSize() const {
    return 6;
}

TPoint3 CRobot3D::getRefPoint(const State &s) const {
    return TPoint3(s[0],s[1],s[2]);
}


TPoint3 CRobot3D::getRefPoint() const {
    return getRefPoint(x);
}



CRobot3D::State CRobot3D::getRandomState(const vector<double> &mapDimension) const {
    State r(6,0.0);
    // normal
    r[0] = getRandom(mapDimension[0],mapDimension[1]);
    r[1] = getRandom(mapDimension[2],mapDimension[3]);
    r[2] = getRandom(mapDimension[4],mapDimension[5]);
    r[3] = getRandom(-M_PI,M_PI);
    r[4] = getRandom(-M_PI,M_PI);
    r[5] = getRandom(-M_PI,M_PI);
    
    return r;
}


CRobot3D::State CRobot3D::getRandomState(const SDimension &mapDimension) const {
    State r(6,0.0);
    // normal
    mapDimension.getRandomPoint(r[0], r[1], r[2]);
    r[3] = getRandom(-M_PI,M_PI);
    r[4] = getRandom(-M_PI,M_PI);
    r[5] = getRandom(-M_PI,M_PI);
    
    return r;
}



double CRobot3D::distance(const State &s) const {
    return distance(s,x);
}


double CRobot3D::distance(const State &s1, const State &s2) const {
    // distance betwen two states
    const double dx = s1[0] - s2[0];
    const double dy = s1[1] - s2[1];
    const double dz = s1[2] - s2[2];
    const double da = s1[3] - s2[3];
    const double db = s1[4] - s2[4];
    const double dc = s1[5] - s2[5];
    //	return dx*dx + dy*dy + dz*dz;
    return dx*dx + dy*dy + dz*dz + _RWx*_RWx*da*da + _RWy*_RWy*db*db + _RWz*_RWz*dc*dc;
}


bool CRobot3D::isValidState() const {
    return isValidState(x);
}


bool CRobot3D::isValidState(const State &s) const {
    return 1;
}


RAPID_model *CRobot3D::getRapidModel(double matrix[3][3], double vector[3]) const {
    return getRapidModel(matrix,vector,x);	
}

// p is in local coordinates of the robot (i..e, relatively to the reference point of the robot which is 0,0
TPoint3 CRobot3D::getTransformedPoint(const TPoint3 &p, const State &s) const {
    double m[3][3];
    double v[3];
    RAPID_model *mm = getRapidModel(m,v,s);
    const double x = m[0][0]*p.x + m[0][1]*p.y + m[0][2]*p.z + v[0];
    const double y = m[1][0]*p.x + m[1][1]*p.y + m[1][2]*p.z + v[1];
    const double z = m[2][0]*p.x + m[2][1]*p.y + m[2][2]*p.z + v[2];
    return TPoint3(x,y,z);
}


RAPID_model *CRobot3D::getRapidModel(double matrix[3][3], double vector[3], const State &s) const {

    // rotace jsou: 
    // 1. roll (rotx, x[3],gamma), 
    // 2. pitch (tory, x[4],beta),
    // 3. yaw (rotz, x[5],alpha) 
    // v tomto poradi !!


    const double ca = cos(s[5]);
    const double sa = sin(s[5]);
    const double cb = cos(s[4]);
    const double sb = sin(s[4]);
    const double cg = cos(s[3]);
    const double sg = sin(s[3]);
    // normal
    matrix[0][0] = ca*cb;
    matrix[0][1] = ca*sb*sg-sa*cg;
    matrix[0][2] = ca*sb*cg+sa*sg;

    matrix[1][0] = sa*cb;
    matrix[1][1] = sa*sb*sg+ca*cg;
    matrix[1][2] = sa*sb*cg-ca*sg;

    matrix[2][0] = -sb;
    matrix[2][1] = cb*sg;
    matrix[2][2] = cb*cg;

    /*
    // for euler angles
    matrix[0][0] = ca*cg-sa*cb*sg;
    matrix[0][1] = -ca*sg-sa*cb*cg;
    matrix[0][2] = sb*sa;

    matrix[1][0] = sa*cg+ca*cb*sg;
    matrix[1][1] = -sa*sg+ca*cb*cg;
    matrix[1][2] = -sb*ca;

    matrix[2][0] = sb*sg;
    matrix[2][1] = sb*cg;
    matrix[2][2] = cb;
     */

    vector[0] = s[0];
    vector[1] = s[1];
    vector[2] = s[2];	
    return rapidModel;
}


const int *CRobot3D::getDistanceTopology() const {
    return distanceTopology;
}


int CRobot3D::getDistanceDim() const {
    return distanceDimension;
}


const MPNN::ANNcoord *CRobot3D::getDistanceScale() const {
    return distanceScale;
}


}

