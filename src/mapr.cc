
#include "types.h"
#include "ut.h"
#include "polygonUtils.h"
#include "rapid/RAPID.H"
#include "mapr.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <typeinfo>

namespace rrtPlanning {

using namespace std;



void MapR::loadMapTriangles(const char *filename) {
	if (mapModel != NULL) {
		delete mapModel;
	}

	std::ifstream ifs(filename);
	triangles.clear();

	string line;

    vector< Triangle > tris;

	while(ifs) {
		getline(ifs,line);
		std::vector<double> vd(lineToNum<double>(line));
		if (vd.size() == 6) {
            TPoint3 a(vd[0],vd[1],0);
            TPoint3 b(vd[2],vd[3],0);
            TPoint3 c(vd[4],vd[5],0);
            Triangle t(a,b,c);
			tris.push_back(t);
		}
		vd.clear();
	}
	ifs.close();

    setMapTriangles(tris);
	sourceName = std::string(filename);

    WDEBUG("Loaded " << tris.size() << " from " << filename);

}


/** directly set triangles forming the obstacles in map */

void MapR::setMapTriangles(const std::vector< Triangle > &_triangles) {
	if (mapModel != NULL) {
		delete mapModel;
	}

	triangles.clear();
	triangles = _triangles;

	sourceName = std::string("directSet");

	bool first = true;
	double p1[3], p2[3], p3[3];
	mapModel = new RAPID_model;
	mapModel->BeginModel();
	for(int i=0;i<(int)triangles.size();i++) {
        const Triangle &t(triangles[i]);

        if (first) {
            _dimension.setToPoint(t.a);
            first = false;
        }
        _dimension.update(t);

		p1[0] = t.a.x;
		p1[1] = t.a.y;
		p1[2] = 0;	
		p2[0] = t.b.x;
		p2[1] = t.b.y;
		p2[2] = 0;
		p3[0] = t.c.x;
		p3[1] = t.c.y;
		p3[2] = 0;
		mapModel->AddTri(p1,p2,p3,i);
	
	}	
	mapModel->EndModel();

	// transformation matrix is constant identity matrix
	rm[0][0] = 1; rm[0][1] = 0;rm[0][2] = 0;
	rm[1][0] = 0; rm[1][1] = 1;rm[1][2] = 0;
	rm[2][0] = 0; rm[2][1] = 0;rm[2][2] = 1;

	tm[0] =0; tm[1] = 0; tm[2] = 0;

}



RAPID_model *MapR::getRapidModel(double matrix[3][3], double vector[3]) const {
	for(int i=0;i<3;i++) {
		for(int j=0;j<3;j++) {
			matrix[i][j] = rm[i][j];
		}
		vector[i] = tm[i];
	}
	return mapModel;
}




const std::vector< Triangle > &MapR::getTriangles() const {
	return triangles;
}



} 

