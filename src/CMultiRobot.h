#ifndef _CMULTI_ROBOT_H__
#define _CMULTI_ROBOT_H__

#include <vector>
#include <list>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

#include "types.h"


/* Model for robot pulling trailers
 * model state transition equations derived from LaValle Motion Plnning book, page 603
 *
 *
 * State x desc:
 * x[0] .. x position of car
 * x[1] .. y position of car
 * x[2] .. phi position of car
 * x[3] .. track 0 angle
 * x[4] .. track 1 angle 
 * ...
 * x[k] .. last track angle
 * x[k+1] .. x derivation
 * x[k+2] .. y derivation
 * x[k+3] .. phi drivation
 * x[k+4] .. derivation of angle of trailer 0 
 * ...
 *
 * input: 
 *    input[0] .. speed
 *    input[1] .. turn angle 
 */

namespace rrtPlanning {

using std::vector;
using std::list;
using std::ostream;
using std::stringstream;
using std::string;



template<typename T>
class CMultiRobot {
	public:
	typedef vector<T> State;
	typedef vector<T> Input;

	CMultiRobot(const int numRobot):
		x(numRobot*3*2),numRobots(numRobot)
   	{
		wheelRadius = 15.0;
		wheelDistance = 15.0;
		robotWidth = 2*wheelRadius;
		robotHeight = 2*wheelRadius;
		minTurn = -2;
		maxTurn = 2;
		minSpeed = -2;
		maxSpeed = 2;
		wheelWidth = 5.0;
	}

	~CMultiRobot() {
		x.clear();
	}

	CMultiRobot(const CMultiRobot<T> &rhs):
		x(rhs.x)
	{}


	void setState(const State &sx);
	State getState() const;
	int getInputSize() const;
	int getStateSize() const;
	vector<State> control(const Input &input, const T dt, const int n);
	Input getMinInputValues() const;
	Input getMaxInputValues() const;
	vector< vector<TPoint> > getShape() const;
	vector< vector<TPoint> > getShape(const State &s) const;
	vector<TPoint> getShapePoints(const State &s) const;
	vector<TPoint> getShapePoints() const;
	TPoint getRefPoint(const State &s) const;
	TPoint getRefPoint() const;
	State getRandomState(const vector<double> &mapDimension) const;
	T distance(const State &s) const;
	T distance(const State &s1, const State &s2) const;
	bool isValidState(const State &s) const;
	bool isValidState() const;
	bool isInputValid(const Input &in) const;

	string getString() const;

	template<typename RT>
	friend ostream &operator<<(ostream &os, const CMultiRobot<RT> &robot);

	private:
	State x;
	const int numRobots;
	double wheelRadius, wheelDistance, robotWidth, robotHeight, minTurn, maxTurn,
			minSpeed, maxSpeed, wheelWidth;

	vector< TPoint > makeBox(const T w, const T h, const T refx, const T refy) const;
	vector< vector< TPoint > > makeVehicle(const T widthR, const T heightH,
			const T dx, const T dy, const T alpha, const int wheelMask) const;
};

template<typename T>
void CMultiRobot<T>::setState(const State &sx) {
	x = sx;
}

template<typename T>
typename CMultiRobot<T>::State CMultiRobot<T>::getState() const {
	return x;
}

template<typename T>
int CMultiRobot<T>::getInputSize() const {
	return numRobots*2;
}

template<typename T>
int CMultiRobot<T>::getStateSize() const {
	return 2*(3*numRobots);
}

template<typename T>
vector< typename CMultiRobot<T>::State > CMultiRobot<T>::control(
		const Input &input, const T dt, const int n) {
		
	vector<State> res;
	res.reserve(n);

	const int k = numRobots*3;
	for(int i=0;i<n;i++) {

		for(int r = 0; r < numRobots; r++) {
		// derivation of state variables
			x[k+0+r*3] = (wheelRadius/2.0)*cos(x[3*r+2])*(input[2*r+0]+input[2*r+1]);
			x[k+1+r*3] = (wheelRadius/2.0)*sin(x[3*r+2])*(input[2*r+0]+input[2*r+1]);
			x[k+2+r*3] = (wheelRadius/wheelDistance)*(input[2*r+1] - input[2*r+0]);
		}

		// new state x = x + dt * xd;
		for(int j=0;j<k;j++) {
			x[j] += x[j+k]*dt;
		}

		for(int r=0;r<numRobots;r++) {
			while(x[r*3+2] < -2*M_PI) {
				x[r*3+2]+=2*M_PI;
			}

			while(x[r*3+2] > 2*M_PI) {
				x[r*3+2]-=2*M_PI;
			}
		}

		res.push_back(x);
	}

	return res;
}


template<typename T>
typename CMultiRobot<T>::Input CMultiRobot<T>::getMinInputValues() const {
	Input in(2*numRobots);
	for(int i=0;i<numRobots;i++) {
		in[i*2+0] = minSpeed;
		in[i*2+1] = minTurn;
	}
	return in;
}	

template<typename T>
typename CMultiRobot<T>::Input CMultiRobot<T>::getMaxInputValues() const {
	Input in(2*numRobots);
	for(int i=0;i<numRobots;i++) {
		in[i*2+0] = maxSpeed;
		in[i*2+1] = maxTurn;
	}
	return in;
}

template<typename T>
vector< vector< TPoint> > CMultiRobot<T>::getShape() const {
	return getShape(x);
}


template<typename T>
vector< TPoint > 
CMultiRobot<T>::makeBox(const T w, const T h, const T refx, const T refy) const
{
	vector<TPoint> result;

	result.push_back(TPoint(0,0));
	result.push_back(TPoint(w,0));
	result.push_back(TPoint(w,h));
	result.push_back(TPoint(0,h));

	transformRT(result.begin(), result.end(), -refx, -refy,(T)0);
	return result;
}


template<typename T>
vector< vector< TPoint > > 
CMultiRobot<T>::makeVehicle(const T widthR, const T heightH,
		const T dx, const T dy, const T alpha, const int wheelMask) const{

	const T h2 = heightH / 2.0;
	const T wh = wheelWidth;
	const T R2 = wheelRadius/2.0;
	const T wh2 = wh/2.0;

	vector< vector< TPoint> > result;
	
	vector< TPoint  > t;

	t = makeBox(widthR,heightH,widthR/2.0,heightH/2.0);
	transformRT(t.begin(), t.end(), dx,dy,alpha);
	result.push_back(t);
	t.clear();

	TPoint q;
	if (wheelMask & 1) {
		t = makeBox(wheelRadius,wh,R2,wh2);
		q = transformPointRT_R(TPoint(0,h2),dx,dy,alpha);
		transformRT(t.begin(),t.end(), q.x,q.y,alpha);
		result.push_back(t);
		t.clear();
	}

	if (wheelMask & 2) {
		t = makeBox(wheelRadius,wh,R2,wh2);
		q = transformPointRT_R(TPoint(0,-h2),dx,dy,alpha);
		transformRT(t.begin(),t.end(), q.x,q.y,alpha);
		result.push_back(t);
		t.clear();
	}
	
	return result;

}

template<typename T>
vector< vector< TPoint> > CMultiRobot<T>::getShape(const State &s) const {

	vector< vector<TPoint> > result;

	for(int i=0;i<numRobots;i++) {
		vector< vector<TPoint> > tmp = makeVehicle(robotWidth,robotHeight,
				s[3*i+0],s[3*i+1],s[3*i+2],3);
		std::copy(tmp.begin(),tmp.end(),std::back_inserter(result));
		tmp.clear();
	}

	return result;

}

template<typename T>
vector<TPoint> CMultiRobot<T>::getShapePoints(const State &s) const {
//	const T w2 = robotWidth/2.0;
//	const T h2 = robotHeight/2.0;


	vector<TPoint> result;
	
	for(int i=0;i<numRobots;i++) {
		vector< vector<TPoint> > tmp = makeVehicle(robotWidth,robotHeight,
				s[3*i+0],s[3*i+1],s[3*i+2],0);
		for(int a=0;a<(int)tmp.size();a++) {
			for(int b=0;b<(int)tmp[a].size();b++) {
				result.push_back(tmp[a][b]);
			}
		}
		tmp.clear();
	}

	return result;
}

template<typename T>
vector<TPoint> CMultiRobot<T>::getShapePoints() const {
	return getShapePoints(x);
}

template<typename T>
TPoint CMultiRobot<T>::getRefPoint(const State &s) const {
	return TPoint(s[0],s[1]);
}

template<typename T>
TPoint CMultiRobot<T>::getRefPoint() const {
	return getRefPoint(x);
}


template<typename T>
typename CMultiRobot<T>::State CMultiRobot<T>::getRandomState(const vector<double> &mapDimension) const {
	State r((3*numRobots)*2,0.0);

	for(int i=0;i<numRobots;i++) {
		r[3*i+0] = getRandom(mapDimension[0],mapDimension[1]);
		r[3*i+1] = getRandom(mapDimension[2],mapDimension[3]);
		r[3*i+2] = getRandom(0,2*M_PI);
		
//		r[3*i+0] = (double)rand()/(double)RAND_MAX * (mapDimension[1] - mapDimension[0]) + mapDimension[0];
//		r[3*i+1] = (double)rand()/(double)RAND_MAX * (mapDimension[3] - mapDimension[2]) + mapDimension[2];
//		r[3*i+2] = (double)rand()/(double)RAND_MAX * 2*M_PI;
	}
	return r;
}

template<typename T>
T CMultiRobot<T>::distance(const State &s) const {
	return distance(s,x);
}

template<typename T>
T CMultiRobot<T>::distance(const State &s1, const State &s2) const {
	// distance betwen two states

/*	T dx,dy,d = -1;
	for(int i=0;i<numRobots;i++) {
		dx = s1[3*i+0] - s2[3*i+0];
		dy = s1[3*i+1] - s2[3*i+1];
		if ( (dx * dx + dy * dy) < d || d == -1) {
			d = dx*dx + dy*dy;
		}
	}
*/

	T dx,dy,d = 0;
	for(int i=0;i<numRobots;i++) {
		dx = s1[3*i+0] - s2[3*i+0];
		dy = s1[3*i+1] - s2[3*i+1];
		d += dx*dx + dy*dy;
	}

	return d;
}

template<typename T>
string CMultiRobot<T>::getString() const {
	string result;

	stringstream ss;

	ss << wheelRadius << " " << wheelDistance << " " << robotWidth << " " << robotHeight << " ";
	ss << numRobots << " " << getStateSize() << " " << getInputSize() << " ";
	result = ss.str();
	
	result += toString(getMinInputValues()," "," ");
	result += toString(getMaxInputValues()," "," ");

	return result;
}

template<typename RT>
ostream &operator<<(ostream &os, const CMultiRobot<RT> &robot) {
	os << "MultiRobot(S=(";
	print(robot.getState()," ",")",os);
	os <<")";
	return os;
}

template<typename T>
bool CMultiRobot<T>::isValidState() const {
	return isValidState(x);
}

template<typename T>
bool CMultiRobot<T>::isValidState(const State &s) const {
	return true;
}

template<typename T>
bool CMultiRobot<T>::isInputValid(const Input &in) const {
	return true;
}

} // namespace 

#endif


