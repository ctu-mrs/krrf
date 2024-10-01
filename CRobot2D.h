#ifndef _CROBOT_2D_H
#define _CROBOT_2D_H

#include <vector>
#include <math.h>
#include <iostream>
#include <string>

#include "types.h"
#include "ut.h"
#include "libs/rapid/RAPID.H"
#include "WLog.h"

/** general class for handling a 2D robot. 
  The movement is
  * x[i] = input[i]
  * the shape is loaded from TRI file
  */
namespace rrtPlanning {

class CRobot2D {

	public:
	typedef std::vector<double> State;

	CRobot2D(const char *filename,const double gScale = 1.0, const State &x0 = State(3,0.0));

	~CRobot2D();
	CRobot2D(const CRobot2D &rhs);

	void setState(const State &sx);
	State getState() const;
	int getStateSize() const;
    std::vector< Triangle > getShape() const;
    std::vector< Triangle > getShape(const State &s) const;
    std::vector<TPoint> getShapePoints(const State &s) const;
    std::vector<TPoint> getShapePoints() const;

	TPoint getRefPoint(const State &s) const;
    std::vector<TPoint> getRefPoint(const std::vector<State> &states) const;
	TPoint getRefPoint() const;

    double getRobotArea() const;

	State getRandomState(const std::vector<double> &mapDimension) const;
    State getRandomState(const SDimension &mapDimension) const;


    void states2pts(const std::vector<State> &states, std::vector<TPoint> &pts) const;


	double distance(const State &s) const;
	double distance(const State &s1, const State &s2) const;

	double distance2D(const State &s1, const State &s2) const {
        return pointDistanceEucleid( getRefPoint(s1), getRefPoint(s2));
    }

	bool isValidState(const State &s) const;
	bool isValidState() const;

    double partialDistance(const State &a, const State &b, const int dimension) const;

	std::vector< Triangle > getTriangles() const { return _triangles; }
	RAPID_model *getRapidModel(double matrix[3][3], double vector[3]) const;
	RAPID_model *getRapidModel(double matrix[3][3], double vector[3], const State &s) const;

	const int *getDistanceTopology() const;
	int getDistanceDim() const;
	const MPNN::ANNcoord *getDistanceScale() const;

    void setDimensionScale(const int idx, const double scale);


    void loadTriangles(const char *filename, std::vector< Triangle > &triangles);
    void loadTriangles(const char *filename);


    TPoint getCenter(const std::vector< Triangle > &triangles) const;

    void cutTriangles(std::vector< Triangle > &triangles, const TPoint &center, const double maxDist) const;
    void scaleTriangles(std::vector< Triangle > &triangles, const double maxDist) const;
    void centerTriangles(std::vector< Triangle > &triangles) const;
    
    void centerTriangles();

    void setRapidModel(const std::vector< Triangle > &triangles);
    void setRapidModel();
    Triangle transformTriangle(const int triangleIdx, const State &s) const;
	
    private:

	State x;

	std::string robotName; // name of file from which the robot shape was loaded

	RAPID_model *rapidModel;
	int distanceDimension;
	int *distanceTopology;
	MPNN::ANNcoord *distanceScale;
	double geomScale;

	std::vector< Triangle > _triangles;

    void robotShapeI(const double sizex, const double sizey,  std::vector< Triangle > &triangles);
    void robotShapeL(const double sizex, const double sizey, const double thickness,  std::vector< Triangle > &triangles);
    void robotShapeC(const double radius, const int num, std::vector< Triangle > &triangles);

};


} // namespace

#endif




