#include "map3r.h"
#include "types.h"
#include "ut.h"
#include "polygonUtils.h"
#include "rapid/RAPID.H"
#include "WLog.h"
#include "painting.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>
#include <typeinfo>

namespace rrtPlanning {
using namespace std;



int Map3R::getNumTriangles() const {
    return triangles.size();
}



/** filename: .raw format
    mscale: 
addBox: if true, than extra triangles representing aabb bounding box are added (only!) to rapid collision detection.
when reading map.getTriangles(), these extra triangles are not stored there, so they are not visible in visualization!
*/
void Map3R::loadMapTriangles(const char *filename, const double mscale, const bool addBox) {
	
    
    scale = mscale;

	sourceName = std::string(filename);
	std::ifstream ifs(filename);
	//triangles.clear();

	string line;
	int lineIdx = 1;
    int discarded = 0;
    areaThreshold = -1;
	WDEBUG("Loading map from " << filename << ", areaThreshold is " << areaThreshold);

    vector< Triangle > tris;

	while(ifs) {
		getline(ifs,line);
		std::vector<double> vd(lineToNum<double>(line));
		if (vd.size() == 9) {
            Triangle t;
			for(int i=0;i<3;i++) {
				t[i].x = mscale*vd[3*i+0];
                t[i].y = mscale*vd[3*i+1];
                t[i].z = mscale*vd[3*i+2];
			}
            if (t.area() > areaThreshold) {
				tris.push_back(t);
			} else {
                discarded++;
			}
		} else if (vd.size() == 12) {
            Triangle t;
			for(int i=0;i<3;i++) {
                t[i].x = mscale*vd[3*i+0];
                t[i].y = mscale*vd[3*i+1];
                t[i].z = mscale*vd[3*i+2];
			}
            if (t.area() > areaThreshold) {
				tris.push_back(t);
			} else {
                discarded++;
			}
			for(int i=0;i<3;i++) {
                t[i].x = mscale*vd[3*i+3];
                t[i].y = mscale*vd[3*i+4];
                t[i].z = mscale*vd[3*i+5];
			}
            if (t.area() > areaThreshold) {
				tris.push_back(t);
			} else {
                discarded++;
			}
        }

		vd.clear();
		lineIdx++;
	}
	ifs.close();
	WDEBUG("Loaded " << tris.size() << " triangles from " << (lineIdx-1) << " for map, discarded=" << discarded << " triangles");

    setMapTriangles(tris, addBox);

    WDEBUG("Map dimension is " << _dimension);
    WDEBUG("Map has " << triangles.size() << " triangles");


}


double Map3R::getScale() const {
	return scale;
}


void Map3R::saveMap(const char *filename) const {
    saveTriangles(triangles, filename);
}

/** directly set triangles forming the obstacles in map */

void Map3R::setMapTriangles(const std::vector< Triangle > &_triangles, const bool addBox) {
	if (mapModel != NULL) {
		delete mapModel;
	}


	triangles.clear();
	triangles = _triangles;

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
        _dimension.update(t.a);
        _dimension.update(t.b);
        _dimension.update(t.c);

		p1[0] = t.a.x;
		p1[1] = t.a.y;
		p1[2] = t.a.z;

		p2[0] = t.b.x;
		p2[1] = t.b.y;
		p2[2] = t.b.z;

		p3[0] = t.c.x;
		p3[1] = t.c.y;
		p3[2] = t.c.z;
		mapModel->AddTri(p1,p2,p3,i);
	}	

    
    if (addBox) {
//	    std::vector<TPoint3> triangle;

        const bool doCheck = false;
        int ii = triangles.size();

        const vector<double> dimension(_dimension.getVector());
        // top face
        p1[0] = dimension[0]; // minx
        p1[1] = dimension[2]; // miny
        p1[2] = dimension[5]; //maxz
        p2[0] = dimension[1]; // maxx
        p2[1] = dimension[2]; // miny
        p2[2] = dimension[5]; //maxz
        p3[0] = dimension[1]; // maxx
        p3[1] = dimension[3]; // maxy
        p3[2] = dimension[5]; //maxz
        mapModel->AddTri(p1,p2,p3,ii++);
        if (doCheck) {
            TPoint3 a(p1[0], p1[1], p1[2]);
            TPoint3 b(p2[0], p2[1], p2[2]);
            TPoint3 c(p3[0], p3[1], p3[2]);
            Triangle t(a,b,c);
            triangles.push_back(t);
        }

        p1[0] = dimension[0]; // minx
        p1[1] = dimension[2]; // miny
        p1[2] = dimension[5]; //maxz

        p2[0] = dimension[1]; // maxx
        p2[1] = dimension[3]; // maxy
        p2[2] = dimension[5]; //maxz

        p3[0] = dimension[0]; // minx
        p3[1] = dimension[3]; // maxy
        p3[2] = dimension[5]; //maxz
        mapModel->AddTri(p1,p2,p3,ii++);
        if (doCheck) {
            TPoint3 a(p1[0], p1[1], p1[2]);
            TPoint3 b(p2[0], p2[1], p2[2]);
            TPoint3 c(p3[0], p3[1], p3[2]);
            Triangle t(a,b,c);
            triangles.push_back(t);
        }


        // bottom/top (side) face
        for(int i=0;i<2;i++) {
            p1[0] = dimension[0]; // minx
            p1[1] = dimension[2+i]; 
            p1[2] = dimension[4]; //minz
            p2[0] = dimension[1]; // maxx
            p2[1] = dimension[2+i]; 
            p2[2] = dimension[4]; //minz
            p3[0] = dimension[1]; // maxx
            p3[1] = dimension[2+i]; 
            p3[2] = dimension[5]; //maxz
            mapModel->AddTri(p1,p2,p3,ii++);
            if (doCheck) {
                TPoint3 a(p1[0], p1[1], p1[2]);
                TPoint3 b(p2[0], p2[1], p2[2]);
                TPoint3 c(p3[0], p3[1], p3[2]);
                Triangle t(a,b,c);
                triangles.push_back(t);
            }

            p1[0] = dimension[0]; // minx
            p1[1] = dimension[2+i]; 
            p1[2] = dimension[4]; //minz
            p2[0] = dimension[1]; // maxx
            p2[1] = dimension[2+i]; 
            p2[2] = dimension[5]; //maxz
            p3[0] = dimension[0]; // minx
            p3[1] = dimension[2+i]; 
            p3[2] = dimension[5]; //maxz
            mapModel->AddTri(p1,p2,p3,ii++);
            if (doCheck) {
                TPoint3 a(p1[0], p1[1], p1[2]);
                TPoint3 b(p2[0], p2[1], p2[2]);
                TPoint3 c(p3[0], p3[1], p3[2]);
                Triangle t(a,b,c);
                triangles.push_back(t);
            }
        }

        // left/right faces
        for(int i=0;i<2;i++) {
            p1[0] = dimension[0+i]; 
            p1[1] = dimension[2]; //miny
            p1[2] = dimension[4]; //minz
            p2[0] = dimension[0+i]; 
            p2[1] = dimension[3]; //maxy
            p2[2] = dimension[4]; //minz
            p3[0] = dimension[0+i]; 
            p3[1] = dimension[3]; //maxy
            p3[2] = dimension[5]; //maxz
            mapModel->AddTri(p1,p2,p3,ii++);
            if (doCheck) {
                TPoint3 a(p1[0], p1[1], p1[2]);
                TPoint3 b(p2[0], p2[1], p2[2]);
                TPoint3 c(p3[0], p3[1], p3[2]);
                Triangle t(a,b,c);
                triangles.push_back(t);
            }




            p1[0] = dimension[0+i];
            p1[1] = dimension[2]; //miny
            p1[2] = dimension[4]; //minz
            p2[0] = dimension[0+i]; 
            p2[1] = dimension[3]; // maxy
            p2[2] = dimension[5]; //maxz
            p3[0] = dimension[0+i]; // minx
            p3[1] = dimension[2]; //miny 
            p3[2] = dimension[5]; //maxz
            mapModel->AddTri(p1,p2,p3,ii++);

            if (doCheck) {
                TPoint3 a(p1[0], p1[1], p1[2]);
                TPoint3 b(p2[0], p2[1], p2[2]);
                TPoint3 c(p3[0], p3[1], p3[2]);
                Triangle t(a,b,c);
                triangles.push_back(t);
            }




        }

        
        WDEBUG("Added " << (ii-triangles.size()) << " triangles of extra bounding box to collision detection data structure. ");

    }
    
    
    mapModel->EndModel();




	// transformation matrix is constant identity matrix
	rm[0][0] = 1; rm[0][1] = 0;rm[0][2] = 0;
	rm[1][0] = 0; rm[1][1] = 1;rm[1][2] = 0;
	rm[2][0] = 0; rm[2][1] = 0;rm[2][2] = 1;

	tm[0] =0; tm[1] = 0; tm[2] = 0;

}



RAPID_model *Map3R::getRapidModel(double matrix[3][3], double vector[3]) const {
	for(int i=0;i<3;i++) {
		for(int j=0;j<3;j++) {
			matrix[i][j] = rm[i][j];
		}
		vector[i] = tm[i];
	}
	return mapModel;
}



const std::vector< Triangle > &Map3R::getObstacles() const {
	return triangles;
}


const std::vector< Triangle > &Map3R::getTriangles() const {
	return triangles;
}



/** find a point p in given point array. if such a points does not exists, add it to the end of the array
  * and return its index */

int Map3R::getIndex(vector<TPoint3> &pts, const TPoint3 &p) {
	double mind = -1;
	int minidx = -1;
	double d;
	const double distanceThreshold = 0.01;
	for(int i=0;i<(int)pts.size();i++) {
		d = squaredPointDistanceEucleid(pts[i],p);
		if (d < mind || mind == -1) {
			mind = d;
			minidx = i;
		}
	}

	if (minidx == -1 || mind > distanceThreshold*distanceThreshold) {
		pts.push_back(p);
		minidx = (int)pts.size()-1;
	}
	return minidx;
}


/** find a point p in given point array. if such a points does not exists, add it to the end of the array
  * and return its index
  * the searching is done using KDTree structure 
  */

int Map3R::getIndexKDTree(vector<TPoint3> &pts, const TPoint3 &p, MPNN::MultiANN<int> *kdTree) {

	double dann = INFINITY;
	const double distanceThreshold = 0.01;

	MPNN::ANNpoint query = MPNN::annAllocPt(3);
	query[0] = p.x;
	query[1] = p.y;
	query[2] = p.z;


	int idx = 0;
	if (kdTree->size > 0) {
		int nidx;
		idx = kdTree->NearestNeighbor(query,nidx,dann);
	}
	MPNN::annDeallocPt(query);

	if (dann > distanceThreshold || kdTree->size == 0) {
		MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,3);
		annpt[0][0] = p.x;
		annpt[0][1] = p.y;
		annpt[0][2] = p.z;
		idx = kdTree->size;
		kdTree->AddPoint(annpt[0],idx);
		MPNN::annDeallocPts(annpt);
		pts.push_back(p);
	}

	return idx;
}

/** save only triangles whose indices in 'saveTriangle[]' is true */

void Map3R::saveFilteredTriangles(const char *filename, const std::vector<bool> &saveTriangle) {
    std::ofstream ofs(filename);
    for(int i=0;i<(int)triangles.size();i++) {
        if (saveTriangle[i]) {
            for(int j=0;j<3;j++) {
                ofs << triangles[i][j].x << " "<< triangles[i][j].y << " "<<triangles[i][j].z << " ";
            }
            ofs << "\n";
        }
    }
    ofs.close();
}

		
void Map3R::getTopCorners(std::vector<TPoint3> &pts) {
	pts.clear();
	vector<double> d(_dimension.getVector());

	pts.push_back(TPoint3(d[0],d[2],d[5]));
	pts.push_back(TPoint3(d[1],d[2],d[5]));
	pts.push_back(TPoint3(d[1],d[3],d[5]));
	pts.push_back(TPoint3(d[0],d[3],d[5]));
}


/** return mean height at given point (only its 2d coordinates are considered) in given radius */
double Map3R::getMeanHeight(const TPoint3 &point, const double radius) const {
	double s = 0;
	int c = 0;
	const double radius2 = radius*radius;
	for(int i=0;i<(int)triangles.size();i++) {
        const Triangle &t(triangles[i]);
		for(int j=0;j<3;j++) {
			const double dx = t[j].x - point.x;
			const double dy = t[j].y - point.y;
			const double dist = dx*dx + dy*dy;
			if (dist <= radius2) {
				s += t[j].z;
				c++;
			}		
		}
	}
	if (c > 0) {
		s /= c;
	}
	return s;
}

double Map3R::getMaxHeight(const TPoint3 &point, const double radius) const {
	double maxh = 0;
	int c = 0;
	const double radius2 = radius*radius;
	for(int i=0;i<(int)triangles.size();i++) {
        const Triangle &t(triangles[i]);
		for(int j=0;j<3;j++) {
			const double dx = t[j].x - point.x;
			const double dy = t[j].y - point.y;
			const double dist = dx*dx + dy*dy;
			if (dist <= radius2) {
				if (t[j].z > maxh || c == 0) {
					c++;
					maxh = t[j].z;
				}
			}		
		}
	}
	return maxh;
}

/** computes which triangle corresponds to the 2d position of the point and the make the height as it approcimation */
double Map3R::getSurfaceHeight(const TPoint3 &point) const {

	double mmap[3][3];
	double vmap[3];
	RAPID_model *m = getRapidModel(mmap,vmap);

	RAPID_model *o = new RAPID_model;
	//WDEBUG("mapmodel = " << (int)m);

    const vector<double> dimension(_dimension.getVector());

	double p1[3], p2[3], p3[3];
	o->BeginModel();
	p1[0] = point.x; p1[1] = point.y; p1[2] = dimension[4]-0.1;
	p2[0] = point.x+0.01; p2[1] = point.y; p2[2] = dimension[5]+0.1;
	p3[0] = point.x; p3[1] = point.y+0.01; p3[2] = dimension[5]+0.1;
	o->AddTri(p1,p2,p3,0);
	o->EndModel();
	RAPID_Collide(mmap,vmap,m,mmap,vmap,o,RAPID_ALL_CONTACTS);

//	WDEBUG("Num contacts: " << RAPID_num_contacts);
 //   for(int i=0;i<RAPID_num_contacts;i++) {
//		WDEBUG("c[" << i << "]=" << RAPID_contact[i].id1 << "-" << RAPID_contact[i].id2);
//	}	
	if (RAPID_num_contacts == 0) {
		WDEBUG("Cannot determine surface height using rapid identification of the triangle.");
		delete o;
		return getMaxHeight(point,1);
	}

	double s = 0;
	double max = 0;
    const Triangle &t(triangles[RAPID_contact[0].id1]);
	for(int i=0;i<3;i++) {
		double height = t[i].z;
		s += height;
		if (height > max || i == 0) {
			max = height;
		}
	}
	s /= 3;
	delete o;
	return max;
}


} 


