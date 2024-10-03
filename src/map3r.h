
#ifndef _MAP3_rapid_H__
#define _MAP3_rapid_H__

#include "types.h"
#include "ut.h"
#include "polygonUtils.h"
#include "rapid/RAPID.H"
#include "multiann.h"
#include "ANN.h"
#include "WLog.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <typeinfo>

namespace rrtPlanning {

/** for handling 3D map made from triangles */
class Map3R {
	public:

		Map3R():sourceName("_emptyMap3R_")
		{
			mapModel = NULL;
			scale = 1.0;
			areaThreshold = 0.000005;
		}
		

		~Map3R() {
			sourceName.erase();
			triangles.clear();
			if (mapModel != NULL) {
				delete mapModel;
				mapModel = NULL;
			}
		}

		void loadMapTriangles(const char *filename, const double mscale=1.0, const bool addBox = false);

		double getScale() const;
		void saveMap(const char *filename) const;
		

		void setMapTriangles(const std::vector< Triangle > &_triangles, const bool addBox = false);
		const SDimension &getDimension() const { return _dimension; }

		RAPID_model *getRapidModel(double matrix[3][3], double vector[3]) const;
		double getNearestObstacleDistance(const TPoint3 &point) const;

		const std::vector< Triangle > &getObstacles() const;
		const std::vector< Triangle > &getTriangles() const;

		double getAreaThreshold() const { return areaThreshold; }
		void setAreaThreshold(const double th) { areaThreshold = th; }
        int getNumTriangles() const;
        void saveFilteredTriangles(const char *filename, const std::vector<bool> &saveTriangle);
        
		void getTopCorners(std::vector<TPoint3> &pts);

		double getMeanHeight(const TPoint3 &point, const double radius) const;
		double getMaxHeight(const TPoint3 &point, const double radius) const;
		double getSurfaceHeight(const TPoint3 &point) const;

        const string& getFilename() const { return sourceName; }


	private:

        SDimension _dimension;

		string sourceName;

		double rr[3][3],tr[3], rm[3][3],tm[3];

		std::vector< Triangle > triangles;

		RAPID_model *mapModel;
		double scale;
		double areaThreshold;

		int getIndex(vector<TPoint3> &pts, const TPoint3 &p);
		int getIndexKDTree(vector<TPoint3> &pts, const TPoint3 &p, MPNN::MultiANN<int> *kdTree);
};


} 


#endif


