#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <map>


#include "CRobotBike.h"
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

// from article
// Numerically Stable Dynamic Bicycle Model for Discrete-time ControlNumerically Stable Dynamic Bicycle Model for Discrete-time Control
// stavy jsou:
// x,y,phi,xx,yy,pphi
// kde x,y,phi je poloha v mape, xx,yy,pphi jsou jejich derivace
// oldv je starra rychlost, v je nova rychlsot. v = sqrt(xx^2 + yy^2);
// zrychleni je a = (v-oldv) / dt
// input 0 .. bike longitude acceleration
// input 1 .. delta - steer angle

CRobotBike::CRobotBike(const State &x0):x(x0) {

    robotWidth = 10; // bylo 27
    robotHeight = 20;

	Iz = 1536.7; // yaw inertia in kg*m^2
	kf = -128916; // front axle equivalent sideslip stiffness N/rad
	kr = -85944; // rear axle equivalent sideslip stiffness N/rad
	lf =  1.06;//1.06; // distance between C.G. and front axle m
	lr =  1.85;//1.85; // distance between C.G. and rear axle m
	m = 1412; //mass of the vehicle kg

	//map_modifier = 10.0;

    min_a = -5;  //was 0
    min_delta = -M_PI/4;
    max_a = 2; 
    max_delta = M_PI/4; 


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

    distanceDimension = 3;
    distanceTopology = new int[distanceDimension];
    distanceScale = new MPNN::ANNcoord[distanceDimension];

    for(int i=0;i<distanceDimension;i++) {
        distanceTopology[i] = 1;
        distanceScale[i] = 1.0;
    }

    WDEBUG("robotType: CRobotBike: " << __PRETTY_FUNCTION__ );
}

/*void CRobotBike::enableBackward(const bool v) { // for bike it means enable breaking
    if (v) {
        min_a = -5;
    } else {
        min_a = 0;
    }
}*/


CRobotBike::~CRobotBike() {
    x.clear();
    rapidModel = NULL;
    delete [] distanceTopology;
    delete [] distanceScale;
}



void CRobotBike::setState(const State &sx) {
	x = sx;
}


CRobotBike::State CRobotBike::getState() const {
	return x;
}


int CRobotBike::getInputSize() const {
	return 2;
}


int CRobotBike::getStateSize() const {
	return 8;
}


//TODO
vector< CRobotBike::State > CRobotBike::control(
		const Input &input, const double dt, const int n) {
		
	vector<State> res;
	res.reserve(n);
	double v =0;
	double old_u = 0, old_v = 0, old_om = 0;
	const double oldv = x[6];
	for(int i=0;i<n;i++) {
		// derivation of state variables
		old_u = x[3];
		old_v = x[4];
		old_om = x[5];
		x[0] += dt*(old_u*cos(x[2]) - old_v*sin(x[2]));
		x[1] += dt*(old_v*cos(x[2]) + old_u*sin(x[2]));
		x[2] += dt*old_om;
		
		x[3] = max(0.0, min(x[3] + dt*input[0], 20.0));
		x[4] = max(-4.0, min(4.0,(m*old_u*old_v + dt*(lf*kf - lr*kr)*old_om - dt*kf*input[1]*old_u - dt*m*old_u*old_u*old_om)/(m*old_u - dt*(kf + kr))));
		x[5] = max(-3.0, min(3.0, (Iz*old_u*old_om + dt*(lf*kf - lr*kr)*old_v - dt*lf*kf*input[1]*old_u)/(Iz*old_u - dt*(lf*lf*kf + lr*lr*kr))));

		x[2] = normalizeMPP(x[2]);
		v = sqrt(x[3]*x[3]+x[4]*x[4]);

		res.push_back(x);
	}
	res.back()[7] =  (v - oldv)/dt;
	res.back()[6] = v;
	return res;
}


//TODO
CRobotBike::Input CRobotBike::getMinInputValues() const {
	Input in(2);
	in[0] = min_a;
	in[1] = min_delta;
	return in;
}	

//TODO
CRobotBike::Input CRobotBike::getMaxInputValues() const {
	Input in(2);
	in[0] = max_a;
	in[1] = max_delta;
	return in;
}


vector< vector<TPoint> > CRobotBike::getShape() const {
	return getShape(x);
}



vector< vector<TPoint> > CRobotBike::getShape(const State &s) const {
	vector<TPoint> shape(basicShape);
	transformRT(shape.begin(), shape.end(), s[0],s[1],s[2]);
	return vector< vector<TPoint> >(1,shape);

}


vector<TPoint> CRobotBike::getShapePoints(const State &s) const {
	vector<TPoint> res(basicShape);
	transformRT(res.begin(), res.end(), s[0],s[1],s[2]);
	return res;
}


vector<TPoint> CRobotBike::getShapePoints() const {
	return getShapePoints(x);
}


TPoint CRobotBike::getRefPoint(const State &s) const {
	return TPoint(s[0],s[1]);
}


TPoint CRobotBike::getRefPoint() const {
	return getRefPoint(x);
}



CRobotBike::State CRobotBike::getRandomState(const SDimension &mapDimension) const {
	State r(8,0.0);
	r[0] = getRandom(mapDimension.min.x, mapDimension.max.x);
	r[1] = getRandom(mapDimension.min.y, mapDimension.max.y);
	r[2] = getRandom(0,2*M_PI);
	return r;
}


double CRobotBike::distance(const State &s) const {
	return distance(s,x);
}


double CRobotBike::distance(const State &s1, const State &s2) const {

	const double dx = s1[0] - s2[0];
	const double dy = s1[1] - s2[1];
	//const double dp = s1[2] - s2[2];

	return dx*dx + dy*dy;
//	return dx*dx + dy*dy + dp*dp;
}


bool CRobotBike::isValidState() const {
	return isValidState(x);
}


bool CRobotBike::isValidState(const State &s) const {
//	return (s[3] < 2 && s[4] < 2);
//	return ( fabs(x[7]) < 100 );
	return true;
}


bool CRobotBike::isInputValid(const Input &i) const {
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


RAPID_model *CRobotBike::getRapidModel(double matrix[3][3], double vector[3], const State &s) const{
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


RAPID_model *CRobotBike::getRapidModel(double matrix[3][3], double vector[3]) const {
	return getRapidModel(matrix,vector,x);
}

	

const int *CRobotBike::getDistanceTopology() const {
	return distanceTopology;
}


int CRobotBike::getDistanceDim() const {
	return distanceDimension;
}


const MPNN::ANNcoord *CRobotBike::getDistanceScale() const {
	return distanceScale;
}


} // namespace



