
#ifndef _MAP_rapid_H__
#define _MAP_rapid_H__

#include "types.h"
#include "ut.h"
#include "polygonUtils.h"
#include "rapid/RAPID.H"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <typeinfo>

namespace rrtPlanning {

class MapR {
	public:
		MapR():sourceName("_emptyMapR_")
		{
			mapModel = NULL;
		}
		

		~MapR() {
			sourceName.erase();
			triangles.clear();
			if (mapModel != NULL) {
				delete mapModel;
				mapModel = NULL;
			}
		}

		void loadMapTriangles(const char *filename);
		void setMapTriangles(const std::vector< Triangle > &_triangles);
        const SDimension &getDimension() const { return _dimension; }

		RAPID_model *getRapidModel(double matrix[3][3], double vector[3]) const;
		double getNearestObstacleDistance(const TPoint &point) const;
		const std::vector< Triangle > &getTriangles() const;

	private:

        SDimension _dimension;

		string sourceName;

		double rr[3][3],tr[3], rm[3][3],tm[3];

		std::vector< Triangle > triangles;

		RAPID_model *mapModel;
};


} 


#endif


