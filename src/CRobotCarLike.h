#ifndef _CROBOT_CAR_LIKE_H__
#define _CROBOT_CAR_LIKE_H__

#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>


#include "types.h"
#include "ut.h"
#include "libs/rapid/RAPID.H"
#include "WLog.h"

/* model of a car like robot
 */

namespace rrtPlanning {

class CRobotCarLike {
	public:
	typedef std::vector<double> State;
	typedef std::vector<double> Input;


    // robot state:
    // [0] = x
    // [1] = y
    // [2] = phi (radians)
    // [3] = dot x (derivation of x)
    // [4] = dot y (derivation of y)
    // [5] = dot phi (derivation of phi)
    // [6] = old velocity
    // [7] = acceleration 
    // only [0,1,2] are needed in fact. The rest is here for various tests..

	CRobotCarLike(const State &x0 = State(8,0.0));
	~CRobotCarLike();
	CRobotCarLike(const CRobotCarLike &rhs);

	void setState(const State &sx);
	State getState() const;
	int getInputSize() const;
	int getStateSize() const;
	std::vector<State> control(const Input &input, const double dt, const int n);


	Input getMinInputValues() const;
	Input getMaxInputValues() const;
	std::vector< std::vector<TPoint> > getShape() const;
	std::vector< std::vector<TPoint> > getShape(const State &s) const;
	std::vector<TPoint> getShapePoints(const State &s) const;
	std::vector<TPoint> getShapePoints() const;
	std::vector< std::vector<TPoint> > getShapeTriangles(const State &s) const;
	TPoint getRefPoint(const State &s) const;
	TPoint getRefPoint() const;
	//State getRandomState(const std::vector<double> &mapDimension) const;
	State getRandomState(const SDimension &mapDimension) const;
	double distance(const State &s) const;
	double distance(const State &s1, const State &s2) const;
	bool isValidState(const State &s) const;
	bool isValidState() const;
	bool isInputValid(const Input &i) const;

	RAPID_model *getRapidModel(double matrix[3][3], double vector[3], const State &s) const;
	RAPID_model *getRapidModel(double matrix[3][3], double vector[3]) const;

	const int *getDistanceTopology() const;
	int getDistanceDim() const;
	const MPNN::ANNcoord *getDistanceScale() const;

    
    void setMinInputValues(const double fwd, const double turn) { minForward = fwd; minTurn = turn; }
    void setMaxInputValues(const double fwd, const double turn) { maxForward = fwd; maxTurn = turn; }
    void enableBackward(const bool v);

	private:
	State x;
	double robotL, robotWidth, robotHeight;
	double minForward, maxForward, minTurn, maxTurn;
	std::vector<TPoint> basicShape;
	int distanceDimension;
	RAPID_model *rapidModel;
	int *distanceTopology;
    MPNN::ANNcoord *distanceScale;

};


} // namespace

#endif


