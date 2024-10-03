#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <map>


#include "CRobotDiff.h"
#include "types.h"
#include "ut.h"
#include "rapid/RAPID.H"
#include "WLog.h"

/** minEdge time for this robot is:
  * tmin = 2*pi*L / (r * v_max)
  * where v_max is: min{v_r-v_l} 
  */

namespace rrtPlanning {

using namespace std;


// stavy jsou:
// x,y,phi,xx,yy,pphi,oldv,a
// kde x,y,phi je poloha v mape, xx,yy,pphi jsou jejich derivace
// oldv je starra rychlost, v je nova rychlsot. v = sqrt(xx^2 + yy^2);
// zrychleni je a = (v-oldv) / dt
// input 0 .. left
// input 1 .. right wheel speed
CRobotDiff::CRobotDiff(const State &x0):x(x0) {

    wheelRadius = 5;
    wheelDistance = 20;

    //		wheelRadius = 4.8; // pro g2bota
    //		wheelDistance = 48.5;

    robotWidth = 20; // bylo 27
    robotHeight = 20;


    minLeft = -2; 
    minRight = -2;
    maxLeft = 2; 
    maxRight = 2; 


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
    p1[0] = w2; p1[1] = h2; p1[2] = 0;
    p2[0] = -w2; p2[1] = h2; p2[2] = 0;
    p3[0] = -w2; p3[1] = -h2; p3[2] = 0;
    rapidModel->AddTri(p1,p2,p3,1);
    rapidModel->EndModel();

    distanceDimension = 2;
    distanceTopology = new int[distanceDimension];
    distanceScale = new MPNN::ANNcoord[distanceDimension];

    for(int i=0;i<distanceDimension;i++) {
        distanceTopology[i] = 1;
        distanceScale[i] = 1.0;
    }

    WDEBUG("robotType: CRobotDiff: " << __PRETTY_FUNCTION__ );
}

void CRobotDiff::enableBackward(const bool v) {
    if (v) {
        minLeft = -2;
        minRight = -2;
    } else {
        minLeft = 0;
        minRight = 0;
    }
}


CRobotDiff::~CRobotDiff() {
    x.clear();
    rapidModel = NULL;
    delete [] distanceTopology;
    delete [] distanceScale;
}



void CRobotDiff::setState(const State &sx) {
	x = sx;
}


CRobotDiff::State CRobotDiff::getState() const {
	return x;
}


int CRobotDiff::getInputSize() const {
	return 2;
}


int CRobotDiff::getStateSize() const {
	return 8;
}


vector< CRobotDiff::State > CRobotDiff::control(
		const Input &input, const double dt, const int n) {
		
	vector<State> res;
	res.reserve(n);
	double v =0;
	const double oldv = x[6];
	for(int i=0;i<n;i++) {
		// derivation of state variables
		x[3] = (wheelRadius/2.0)*cos(x[2])*(input[0]+input[1]);
		x[4] = (wheelRadius/2.0)*sin(x[2])*(input[0]+input[1]);
		x[5] = ((double)1.0*wheelRadius/wheelDistance)*(input[1] - input[0]);

		// new state x = x + dt * xd;
		for(int j=0;j<3;j++) {
			x[j] += x[j+3]*dt;
		}

		/*
		std::cerr << "Q1";
		while(x[2] < -2*M_PI) {
			x[2]+=2*M_PI;
		}
		std::cerr << "Q2";

		while(x[2] > 2*M_PI) {
			x[2]-=2*M_PI;
		}
		std::cerr << "Q3";
		*/
		x[2] = normalizeMPP(x[2]);
		v = sqrt(x[3]*x[3]+x[4]*x[4]);
			
		res.push_back(x);
	}
	res.back()[7] =  (v - oldv)/dt;
	res.back()[6] = v;
	return res;
}


/** controls robot's movement and assumes that input was originaly computed for car like robot
  * hence it must be changed in order to navigate the robot on the same trjectory.
  * we assumes that input is: input[0].. fwd speed, input[1] .. curvature
  */

vector< CRobotDiff::State > CRobotDiff::controlCL(
		const Input &input, const double dt, const int n) {
		
	vector<State> res;
	res.reserve(n);
	double v =0;
	const double oldv = x[6];
	for(int i=0;i<n;i++) {
		// derivation of state variables
//		x[3] = (wheelRadius/2.0)*cos(x[2])*(input[0]+input[1]);
//		x[4] = (wheelRadius/2.0)*sin(x[2])*(input[0]+input[1]);
//		x[5] = ((T)1.0*wheelRadius/wheelDistance)*(input[1] - input[0]);

//		x[3] = (wheelRadius/2.0)*cos(x[2])*(input[0]);
//		x[4] = (wheelRadius/2.0)*sin(x[2])*(input[0]);
	//	x[5] = (wheelDistance*input[0]*input[1])/wheelRadius;


		// funguje ok		
//		x[3] = cos(x[2])*(input[0]*2.0/wheelRadius);
//		x[4] = sin(x[2])*(input[0]*2.0/wheelRadius);
//		x[5] = wheelDistance*(input[0]*input[1]) / wheelRadius;

		x[3] = cos(x[2])*(input[0]*1.0/wheelRadius);
		x[4] = sin(x[2])*(input[0]*1.0/wheelRadius);
		x[5] = wheelDistance*(input[0]*input[1]) / wheelRadius;


		// new state x = x + dt * xd;
		for(int j=0;j<3;j++) {
			x[j] += x[j+3]*dt;
		}

		x[2] = normalizeMPP(x[2]);
		v = sqrt(x[3]*x[3]+x[4]*x[4]);
			
		res.push_back(x);
	}
	res.back()[7] =  (v - oldv)/dt;
	res.back()[6] = v;
	return res;
}




CRobotDiff::Input CRobotDiff::getMinInputValues() const {
	Input in(2);
	in[0] = minLeft;
	in[1] = minRight;
	return in;
}	


CRobotDiff::Input CRobotDiff::getMaxInputValues() const {
	Input in(2);
	in[0] = maxLeft;
	in[1] = maxRight;
	return in;
}


vector< vector<TPoint> > CRobotDiff::getShape() const {
	return getShape(x);
}



vector< vector<TPoint> > CRobotDiff::getShape(const State &s) const {
	vector<TPoint> shape(basicShape);
	transformRT(shape.begin(), shape.end(), s[0],s[1],s[2]);
	return vector< vector<TPoint> >(1,shape);

}


vector<TPoint> CRobotDiff::getShapePoints(const State &s) const {
	vector<TPoint> res(basicShape);
	transformRT(res.begin(), res.end(), s[0],s[1],s[2]);
	return res;
}


vector<TPoint> CRobotDiff::getShapePoints() const {
	return getShapePoints(x);
}


TPoint CRobotDiff::getRefPoint(const State &s) const {
	return TPoint(s[0],s[1]);
}


TPoint CRobotDiff::getRefPoint() const {
	return getRefPoint(x);
}



CRobotDiff::State CRobotDiff::getRandomState(const SDimension &mapDimension) const {
	State r(8,0.0);
	r[0] = getRandom(mapDimension.min.x, mapDimension.max.x);
	r[1] = getRandom(mapDimension.min.y, mapDimension.max.y);
	r[2] = getRandom(0,2*M_PI);
	return r;
}


double CRobotDiff::distance(const State &s) const {
	return distance(s,x);
}


double CRobotDiff::distance(const State &s1, const State &s2) const {

	const double dx = s1[0] - s2[0];
	const double dy = s1[1] - s2[1];
	//const double dp = s1[2] - s2[2];

	return dx*dx + dy*dy;
//	return dx*dx + dy*dy + dp*dp;


	// shortest distance between robot rounding the cicle
	// ang goal stat
/*
	T k1 = x[5]*wheelDistance/wheelRadius;
	if (sin(x[3]) != 0) {
		k2 = x[4]*2/wheelRadius/sin(x[3]);
	} else {
		k2 = x[4]*2/wheelRadius/cos(x[3]);
	}
	const T dx = s1[0] - s2[0];
	const T dy = s1[1] - s2[1];

	if (k1 == 0) k1 == 1;
	return sqrt(dx*dx) - k2/k1*wheelDistance/2.0;
*/
}


bool CRobotDiff::isValidState() const {
	return isValidState(x);
}


bool CRobotDiff::isValidState(const State &s) const {
//	return (s[3] < 2 && s[4] < 2);
//	return ( fabs(x[7]) < 100 );
	return true;
}


bool CRobotDiff::isInputValid(const Input &i) const {
	 // zataci jen doprava
    /*
	if ((i[0] >= 0 && i[1] >= 0) && 
//		((i[0] == i[1]) ||
		 (i[0] > i[1]+0.1)) {
		return true;
	} 
	return false;
	*/
/*
	// zataci jen doleva
	if ((i[0] >= 0 && i[1] >= 0) && 
		(// (i[0] == i[1]) ||
		 (i[0] < i[1]))) {
		return true;
	} 
	return false;
*/
    return true;
}


RAPID_model *CRobotDiff::getRapidModel(double matrix[3][3], double vector[3], const State &s) const{
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


RAPID_model *CRobotDiff::getRapidModel(double matrix[3][3], double vector[3]) const {
	return getRapidModel(matrix,vector,x);
}

	

const int *CRobotDiff::getDistanceTopology() const {
	return distanceTopology;
}


int CRobotDiff::getDistanceDim() const {
	return distanceDimension;
}


const MPNN::ANNcoord *CRobotDiff::getDistanceScale() const {
	return distanceScale;
}


} // namespace



