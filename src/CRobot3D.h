#ifndef _CROBOT_3D_H
#define _CROBOT_3D_H

#include <vector>
#include <math.h>
#include <iostream>
#include <string>


#include "types.h"
#include "ut.h"
#include "rapid/RAPID.H"
#include "WLog.h"
#include "ANN.h"

/** general class for handling any 3D robot. The movement is
  * x[i] = input[i]
  * the shape is loaded from TRI3 file
  */
namespace rrtPlanning {

class CRobot3D {

	public:
	typedef std::vector<double> State;

    CRobot3D(const char *filename, const double scalex = 1.0, const double scaley = 1.0, const double scalez = 1.0);

	~CRobot3D();
	CRobot3D(const CRobot3D &rhs);

	void setState(const State &sx);
	State getState() const;
	int getStateSize() const;
    
	TPoint3 getRefPoint(const State &s) const;
	TPoint3 getRefPoint() const;
	
    State getRandomState(const std::vector<double> &mapDimension) const;
	State getRandomState(const SDimension &mapDimension) const;


	double distance(const State &s) const;
	double distance(const State &s1, const State &s2) const;
	bool isValidState(const State &s) const;
	bool isValidState() const;

    double partialDistance(const State &a, const State &b, const int dimension) const;


	const std::vector< Triangle > &getTriangles() const { return _triangles; }
	
	RAPID_model *getRapidModel(double matrix[3][3], double vector[3]) const;
	RAPID_model *getRapidModel(double matrix[3][3], double vector[3], const State &s) const;

	const int *getDistanceTopology() const;
	int getDistanceDim() const;
	const MPNN::ANNcoord *getDistanceScale() const;

	void setAreaThreshold(const double th) { areaThreshold = th; }
	double getAreaThreshold() const { return areaThreshold; }
    
    std::string getRobotFile() const { return robotName; }

    void setRobotName(const char *name) { 
        robotName = string(name);
    }

    double getScaleX() const { return _scalex; }
    double getScaleY() const { return _scaley; }
    double getScaleZ() const { return _scalez; }

    void loadTriangles(const char *filename, const double areaThreshold, const double scalex, const double scaley, const double scalez, 
            std::vector< Triangle > &triangles) ;
    void setRapidModel(const std::vector< Triangle > &triangles);
    void cutToDistance(std::vector< Triangle > &triangles, const double maxDistance) const;

    double getMaxTriangleDistance(const vector< Triangle > &triangles) const;


    TPoint3 getTransformedPoint(const TPoint3 &p, const State &s) const;
    void translateModel(const TPoint3 &newOrigin);

	private:
	State x;
	std::string robotName; // name of file from which the robot shape was loaded
	double _scalex, _scaley, _scalez;
    double _RWx, _RWy, _RWz; //rotation scaling for distance metric

	RAPID_model *rapidModel;
    std::vector<TPoint3> baseTPoint3s;
	int distanceDimension;
	int *distanceTopology;
    MPNN::ANNcoord *distanceScale;
	double areaThreshold;

	std::vector< Triangle > _triangles;

};


} // namespace

#endif




