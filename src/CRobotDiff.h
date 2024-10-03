#ifndef _CROBOT_DIFF_H__
#define _CROBOT_DIFF_H__

#include <vector>
#include <math.h>
#include <iostream>
#include <string>


#include "types.h"
#include "ut.h"
#include "libs/rapid/RAPID.H"
#include "WLog.h"

/** minEdge time for this robot is:
  * tmin = 2*pi*L / (r * v_max)
  * where v_max is: min{v_r-v_l} 
  */

namespace rrtPlanning {

class CRobotDiff {
	public:
	typedef std::vector<double> State;
	typedef std::vector<double> Input;


    CRobotDiff(const State &x0 = State(8,0.0));

	~CRobotDiff();

	CRobotDiff(const CRobotDiff &rhs):
		x(rhs.x),wheelRadius(rhs.wheelRadius),wheelDistance(rhs.wheelDistance),
		robotWidth(rhs.robotWidth),robotHeight(rhs.robotHeight),minLeft(rhs.minLeft),maxLeft(rhs.maxLeft),
		minRight(rhs.minRight),maxRight(rhs.maxRight),basicShape(rhs.basicShape), distanceDimension(rhs.distanceDimension),
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


	void setState(const State &sx);
	State getState() const;
	int getInputSize() const;
	int getStateSize() const;
	std::vector<State> control(const Input &input, const double dt, const int n);
	std::vector<State> controlCL(const Input &input, const double dt, const int n);

	Input getMinInputValues() const;
	Input getMaxInputValues() const;
	std::vector< std::vector<TPoint> > getShape() const;
	std::vector< std::vector<TPoint> > getShape(const State &s) const;
	std::vector<TPoint> getShapePoints(const State &s) const;
	std::vector<TPoint> getShapePoints() const;
	TPoint getRefPoint(const State &s) const;
	TPoint getRefPoint() const;
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

    void setMinInputValues(const double left, const double right) { minLeft = left; minRight = right; }
    void setMaxInputValues(const double left, const double right) { maxLeft = left; maxRight = right; }

    void enableBackward(const bool v);

	private:
	State x;
	double wheelRadius, wheelDistance, robotWidth, robotHeight;
	double minLeft, maxLeft, minRight, maxRight;
	std::vector<TPoint> basicShape;
	int distanceDimension;
	RAPID_model *rapidModel;
	int *distanceTopology;
    MPNN::ANNcoord *distanceScale;

};


} // namespace

#endif


