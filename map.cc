#include "types.h"
#include "ut.h"
#include "polygonUtils.h"
#include "map.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <typeinfo>

namespace rrtPlanning {

using namespace std;


bool Map::isFree(const TPoint &p) const {

	/*
	bool res = pointInPolygon(p,mapPolygon);

	if (res == false) {
		return false;
	} else {
		res = false;
		for(int i=0;i<obstacles.size() && !res;i++) {
			res = pointInPolygon(p,obstacles[i]); 
		}
		return !res;
	}
	*/
	
	// return true, if point p is in polygon (map)

	if (wn_PnPoly(p, map) == 0) {
		return false;
	} else {
		bool res = false;
		for(int i=0;i<(int)obstaclesPolygon.size() && !res;i++) {
			res = wn_PnPoly(p,obstaclesPolygon[i]);
		}
		return !res;
	}
	
}


// return true if ALL points are free

template<typename Iter>
bool Map::isFree(Iter begin, Iter end) const {
	bool t = true;
	for(Iter i = begin; i != end && t; i++) {
		if (isFree(*i) == false) {
			t = false;
		}
	}
	return t;
}

 
bool Map::isFreePts(const vector<TPoint> &pts) const {
	return isFree(pts.begin(),pts.end());
}


// assume pts are polygon of robot
 
bool Map::isFreePolygon(const vector<TPoint> &pts) const {
	bool res = robotMapCollision(pts, map);
	

	if (res == false) {

		// check all obstacles
		res = false;
		for(int i=0;i<(int)obstacles.size() && !res;i++) {
			res = robotObstacleCollision(pts,obstaclesPolygon[i]);
		}

		return !res;
	} 

	return false;
}


bool Map::isFreePolygons(const vector< vector<TPoint> > &pgs) const {

	for(int i=0;i<(int)pgs.size();i++) {
		if (isFreePolygon(pgs[i]) == false) {
			return false;
		}
	}

	return true;
}



void Map::loadPolygon(const char *filename) {
	sourceName = string(filename);

	ifstream ifs(filename);

	double x,y;
	vector<TPoint> tmp;


	while(ifs) {
		if (ifs >> x >> y) {
			tmp.push_back(TPoint(x,y));
		} else {
			ifs.ignore(10000,'\n');
		}
	}
	ifs.close();

	map.clear();
	map = tmp;
	tmp.clear();

	update();	
}




void Map::loadIsoMap(const char *filename) {
	sourceName = string(filename);	

	ifstream ifs(filename);

	int s = 0;

	map.clear();
	mapPolygon.clear();
	obstacles.clear();
	obstaclesPolygon.clear();

	vector<TPoint> tmpPolygon;
	double x = 0;
	double y = 0;

	string line;
	while(ifs) {
		switch(s) {
			case 0: {
				std::getline(ifs,line);
				for(int i=0;i<(int)line.size();i++) {
					line[i] = std::tolower(line[i]);
				}
				if (line.find("[obstacle]") != std::string::npos) {
					s = 1;
				} 
				if (line.find("[border]") != std::string::npos) {
					s = 2;
				}
				break;
			}
			case 1: {
				if (ifs >> x >> y) {
					tmpPolygon.push_back(TPoint(x,y));
				} else {
					ifs.clear();
					obstacles.push_back(tmpPolygon);
					tmpPolygon.clear();
					s = 0;
				}
				break;
			}

			case 2: {
				if (ifs >> x >> y) {
					tmpPolygon.push_back(TPoint(x,y));
				} else {
					ifs.clear();
					s = 0;
					map = tmpPolygon;
					tmpPolygon.clear();
				}
				break;
			}
			default: break;
		}
	}
	ifs.close();



	update();
}


void Map::saveMapPolygon(const char *filename) const {
	ofstream ofs(filename);

	for(int i=0;i<(int)map.size();i++) {
		ofs << map[i].x << " " << map[i].y << "\n";
	}
	ofs.close();
}


void Map::saveIsoMap(const char *filename) const {
	ofstream ofs(filename);

	ofs << "[border]\n";
	for(int i=0;i<(int)map.size();i++) {
		ofs << mapPolygon[i].x << " " << mapPolygon[i].y << "\n";
	}

	for(int i=0;i<(int)obstacles.size();i++) {
		ofs << "[obstacle]\n";
		for(int j=0;j<(int)obstacles[i].size();j++) {
			ofs << obstacles[i][j].x << " "<< obstacles[i][j].y << "\n";
		}
	}
	ofs.close();

}


void Map::update() {

	// dimension of map is compute only from border (map[]) not from obstacles


	if (map.size() == 0) {

		for(int i=0;i<4;i++) {
			dimension[i] = (double)0;
		}

	} else {

		double minx = map[0].x;
		double maxx = minx;
		double miny = map[0].y;
		double maxy = miny;

		for(int i=0;i<(int)map.size();i++) {
			minx = std::min(minx, map[i].x);
			maxx = std::max(maxx, map[i].x);
			miny = std::min(miny, map[i].y);
			maxy = std::max(maxy, map[i].y);
		}

		dimension[0] = minx;
		dimension[1] = maxx;
		dimension[2] = miny;
		dimension[3] = maxy;

		mapPolygon.clear();
		mapPolygon = map;

		if (mapPolygon.back().x == mapPolygon.front().x &&
			mapPolygon.back().y == mapPolygon.front().y) {
			mapPolygon.pop_back();
		}

		if (map.front().x != map.back().x || 
			map.front().y != map.back().y) {
			map.push_back(map.front());
		}
	}

	obstaclesPolygon = obstacles;

	for(int i=0;i<(int)obstaclesPolygon.size(); i++) {
			
		if (obstaclesPolygon[i].size() > 0) {
			if (obstaclesPolygon[i].front().x != obstaclesPolygon[i].back().x ||
				obstaclesPolygon[i].front().y != obstaclesPolygon[i].back().y) {
				obstaclesPolygon[i].push_back(obstaclesPolygon[i].front());
			}
		}
	}

}


vector<double> Map::getDimension() const {
	return dimension;
}


const vector<TPoint> &Map::getMap() const {
	return map;
}

// find a obstacle that is nearest to a given point and return distance
// to this obstacle

double Map::getNearestObstacleDistance(const TPoint &point) const {
	
	double minDist = -1;
	double d;

	for(int i=0;i<(int)map.size()-1; i++) {
		d = dist_Point_to_Segment(point,map[i],map[i+1]);
		if (d < minDist || minDist == -1) {
			minDist = d;
		}
	}

	if (minDist < 0) {
		minDist = 0;
	}	

	return sqrt(minDist);

}

		
void Map::setMapPolygon(const vector<TPoint> &pts) {
	sourceName = "set from pts";
	map.clear();
	map = pts;
	update();
}



const vector< vector<TPoint> > &Map::getObstacles() const {
	return obstacles;
}

/** transform map so all coordinates are not negative */

void Map::normalizeMap() {
    update();

    std::vector<TPoint> tmp;
    for(int i=0;i<(int)map.size();i++) {
        tmp.push_back(TPoint(map[i].x-dimension[0],map[i].y-dimension[2]));
    }
    map = tmp;

    tmp.clear();
    for(int i=0;i<(int)mapPolygon.size();i++) {
        tmp.push_back(TPoint(mapPolygon[i].x-dimension[0],mapPolygon[i].y-dimension[2]));
    }
    mapPolygon = tmp;


    tmp.clear();
    for(int i=0;i<(int)obstacles.size();i++) {
        for(int j=0;j<(int)obstacles[i].size();j++) {
            tmp.push_back(TPoint(obstacles[i][j].x-dimension[0],obstacles[i][j].y-dimension[2]));
        }
        obstacles[i] = tmp;
        tmp.clear();
    }

    for(int i=0;i<(int)obstaclesPolygon.size();i++) {
        for(int j=0;j<(int)obstaclesPolygon[i].size();j++) {
            tmp.push_back(TPoint(obstaclesPolygon[i][j].x - dimension[0], obstaclesPolygon[i][j].y - dimension[2]));
        }
        obstaclesPolygon[i] = tmp;
        tmp.clear();
    }
    tmp.clear();

    update();
}



}




