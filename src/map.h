
#ifndef _MAP2_H__
#define _MAP2_H__

#include "types.h"
#include "ut.h"
#include "polygonUtils.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <typeinfo>

namespace rrtPlanning {


class Map {
	public:

		Map():dimension(4,0.0),sourceName("_emptyMap_")
		{
			map.clear();
			obstacles.clear();
			obstaclesPolygon.clear();
		}
		

		~Map() {
			dimension.clear();
			sourceName.erase();
			map.clear();
			mapPolygon.clear();
			obstacles.clear();
			obstaclesPolygon.clear();
		}

		void loadPolygon(const char *filename);
		void loadIsoMap(const char *filename);
		void saveMapPolygon(const char *filename) const;
		void saveIsoMap(const char *filename) const;
		void setMapPolygon(const vector<TPoint> &pts);

        /** translate map so all coordinate are not negative */
        void normalizeMap();
		void update();

		bool isFree(const TPoint &p) const;

		vector<double> getDimension() const;

		const vector<TPoint> &getMap() const;
		const vector< vector<TPoint> > &getObstacles() const;

		template<typename Iter>
		bool isFree(Iter begin, Iter end) const;
		
		bool isFreePts(const vector<TPoint> &pts) const;
		bool isFreePolygon(const vector<TPoint> &pts) const;
		bool isFreePolygons(const vector< vector<TPoint> > &pgs) const;


		double getNearestObstacleDistance(const TPoint &point) const;


	private:


		// first and last point qual 
		vector<TPoint> map; // border

		// first and last point does not equal
		vector<TPoint> mapPolygon; // border

		// last point != first point
		vector< vector<TPoint> > obstacles;

		// last point == first point
		vector< vector<TPoint> > obstaclesPolygon;

		vector<double> dimension;
		string sourceName;


};


}


#endif


