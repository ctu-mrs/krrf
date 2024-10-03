#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>


#include "types.h"
#include "ut.h"
#include "rapid/RAPID.H"
#include "WLog.h"
#include "CRobotCarLike.h"


/* model of a car like robot
 */

namespace rrtPlanning {

using namespace std;


// stavy jsou:
	// x,y,phi,xx,yy,pphi,oldv,a
	// kde x,y,phi je poloha v mape, xx,yy,pphi jsou jejich derivace
	// oldv je starra rychlost, v je nova rychlsot. v = sqrt(xx^2 + yy^2);
	// zrychleni je a = (v-oldv) / dt
	// input 0 .. forward speed
	// input 1 .. turning speed

CRobotCarLike::CRobotCarLike(const State &x0):x(x0) {

		robotWidth = 15; //15 bylo 20
		robotHeight = 30; //40 bylo 20
		robotL = 30; //30// L in model - distance between front and rear wheels


		minForward = -50; 
		minTurn = -45.0*M_PI/180.0;

		maxForward= 50; 
		maxTurn = 45.0*M_PI/180.0;
		
		const double w2 = robotWidth/2.0;
		const double h2 = robotHeight/2.0;
		basicShape.reserve(4);
		basicShape.push_back(TPoint(w2,h2));
		basicShape.push_back(TPoint(-w2,h2));
		basicShape.push_back(TPoint(-w2,-h2));
		basicShape.push_back(TPoint(w2,-h2));

		rapidModel = new RAPID_model;	
		double p1[3], p2[3], p3[3];

		rapidModel->BeginModel();
		
		p1[0] = -w2; p1[1] = -h2; p1[2] = 0;
		p2[0] = w2; p2[1] = -h2; p2[2] = 0;
		p3[0] = w2; p3[1] = h2; p3[2] = 0;
		rapidModel->AddTri(p1,p2,p3,0);
		//rapidModel->AddTri(p3,p2,p1,2);
		p1[0] = w2; p1[1] = h2; p1[2] = 0;
		p2[0] = -w2; p2[1] = h2; p2[2] = 0;
		p3[0] = -w2; p3[1] = -h2; p3[2] = 0;
		
		rapidModel->AddTri(p1,p2,p3,1);
		rapidModel->EndModel();

		distanceDimension = 3;
		distanceTopology = new int[distanceDimension];
		distanceScale = new MPNN::ANNcoord[distanceDimension];

		for(int i=0;i<distanceDimension;i++) {
			distanceTopology[i] = 1;
			distanceScale[i] = 1.0;
		}
        WDEBUG("robotType: CRobotCarLike: " << __PRETTY_FUNCTION__ );
}

void CRobotCarLike::enableBackward(const bool v) {
    if (v) {
        minForward = -50;
    } else {
        minForward = 0;
    }
}



CRobotCarLike::~CRobotCarLike() {
    x.clear();
    rapidModel = NULL;
    delete [] distanceTopology;
    delete [] distanceScale;
}

CRobotCarLike::CRobotCarLike(const CRobotCarLike &rhs):
		x(rhs.x),robotL(rhs.robotL),
		robotWidth(rhs.robotWidth),robotHeight(rhs.robotHeight),
		minForward(rhs.minForward),maxForward(rhs.maxForward),
		minTurn(rhs.minTurn),maxTurn(rhs.maxTurn),
		basicShape(rhs.basicShape), distanceDimension(rhs.distanceDimension),
		rapidModel(rhs.rapidModel)
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



void CRobotCarLike::setState(const State &sx) {
	x = sx;
}


CRobotCarLike::State CRobotCarLike::getState() const {
	return x;
}


int CRobotCarLike::getInputSize() const {
	return 2;
}


int CRobotCarLike::getStateSize() const {
	return 8;
}

/** inputs: input[0] .. forward speed
  *         input[1] .. turn speed 
  *
  * State is described in .h file
  */

vector< CRobotCarLike::State > CRobotCarLike::control(
		const Input &input, const double dt, const int n) {
	
	vector<State> res;
	res.reserve(n);
	double v =0;
	const double oldv = x[6];
	for(int i=0;i<n;i++) {

		// derivation of state variables - car like motion model
		x[3] = cos(x[2])*input[0];
		x[4] = sin(x[2])*input[0];
		x[5] = input[0]/robotL * tan(input[1]);

		// new state x = x + dt * xd;
		for(int j=0;j<3;j++) {
			x[j] += x[j+3]*dt;
		}


		while(x[2] < -2*M_PI) {
			x[2]+=2*M_PI;
		}

		while(x[2] > 2*M_PI) {
			x[2]-=2*M_PI;
		}
		v = input[0];
			
		res.push_back(x);
	}
	res.back()[7] =  (v - oldv)/dt;
	res.back()[6] = v;
	return res;
}



CRobotCarLike::Input CRobotCarLike::getMinInputValues() const {
	Input in(2);
	in[0] = minForward;
	in[1] = minTurn;
	return in;
}	


CRobotCarLike::Input CRobotCarLike::getMaxInputValues() const {
	Input in(2);
	in[0] = maxForward;
	in[1] = maxTurn;
	return in;
}


vector< vector<TPoint> > CRobotCarLike::getShape() const {
	return getShape(x);
}



vector< vector<TPoint> > CRobotCarLike::getShape(const State &s) const {
	vector<TPoint> shape(basicShape);
	transformRT(shape.begin(), shape.end(), s[0],s[1],s[2]);
	return vector< vector<TPoint> >(1,shape);

}

// return shape as set of triangles. WHere we assume, that each shape is rectangle */
vector< vector<TPoint> > CRobotCarLike::getShapeTriangles(const State &s) const {
	vector<TPoint> shape(basicShape);
	transformRT(shape.begin(), shape.end(), s[0],s[1],s[2]);

	if (shape.size() != 4) {
		WDEBUG("Cannot created triangle model of rectangle, as shape.size=" << shape.size() << "!");
		exit(0);
	}
	
	vector< vector<TPoint> > res;
	
	res.push_back( vector<TPoint>());
	res.back().push_back(shape[0]);
	res.back().push_back(shape[1]);
	res.back().push_back(shape[2]);
   
	res.push_back( vector<TPoint>());
	res.back().push_back(shape[0]);
	res.back().push_back(shape[2]);
	res.back().push_back(shape[3]);
	return res;
}


vector<TPoint> CRobotCarLike::getShapePoints(const State &s) const {
	vector<TPoint> res(basicShape);
	transformRT(res.begin(), res.end(), s[0],s[1],s[2]);
	return res;
}


vector<TPoint> CRobotCarLike::getShapePoints() const {
	return getShapePoints(x);
}


TPoint CRobotCarLike::getRefPoint(const State &s) const {
	return TPoint(s[0],s[1]);
}


TPoint CRobotCarLike::getRefPoint() const {
	return getRefPoint(x);
}


/*
CRobotCarLike::State CRobotCarLike::getRandomState(const vector<double> &mapDimension) const {
	State r(8,0.0);
	r[0] = getRandom(mapDimension[0], mapDimension[1]);
	r[1] = getRandom(mapDimension[2], mapDimension[3]);
	r[2] = getRandom(0,2*M_PI);
	return r;
}
*/

CRobotCarLike::State CRobotCarLike::getRandomState(const SDimension &mapDimension) const {
	State r(8,0.0);
	r[0] = getRandom(mapDimension.min.x, mapDimension.max.x);
	r[1] = getRandom(mapDimension.min.y, mapDimension.max.y);
	r[2] = getRandom(0,2*M_PI);
	return r;
}

double CRobotCarLike::distance(const State &s) const {
	return distance(s,x);
}


double CRobotCarLike::distance(const State &s1, const State &s2) const {
	const double dx = s1[0] - s2[0];
	const double dy = s1[1] - s2[1];
	const double dphi = s1[2] - s2[2];
	return dx*dx + dy*dy;
	//return dx*dx + dy*dy + dphi*dphi;
}

bool CRobotCarLike::isValidState() const {
	return isValidState(x);
}


bool CRobotCarLike::isValidState(const State &s) const {
	return true;
}


bool CRobotCarLike::isInputValid(const Input &i) const {
    /*
    if (i[1] >= 0) {
	    return true;
    } else {
        return false;
    }
    */

    return true;
}


RAPID_model *CRobotCarLike::getRapidModel(double matrix[3][3], double vector[3], const State &s) const{
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


RAPID_model *CRobotCarLike::getRapidModel(double matrix[3][3], double vector[3]) const {
	return getRapidModel(matrix,vector,x);
}

	

const int *CRobotCarLike::getDistanceTopology() const {
	return distanceTopology;
}


int CRobotCarLike::getDistanceDim() const {
	return distanceDimension;
}


const MPNN::ANNcoord *CRobotCarLike::getDistanceScale() const {
	return distanceScale;
}



} // namespace



