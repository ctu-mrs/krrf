#include "types.h"
#include "ut.h"
#include <string>
#include <iostream>
#include <sstream>

using namespace std;

namespace rrtPlanning {


std::ostream &operator<<(std::ostream &os, const SDimension &dim) {
    os << "DIM{x=" << dim.min.x << ":" << dim.max.x << ", y=" << dim.min.y << ":" << dim.max.y << ", z=" << dim.min.z << ":" << dim.max.z << ", vol=" << dim.volume() << "}";
    return os;
}



ostream &operator<<(ostream &os, const TPoint &r) {
	os << '(' << r.x  << ',' << r.y <<')';
	return os;
}

ostream &operator<<(ostream &os, const TPoint3 &r) {
	os << '(' << r.x  << ',' << r.y << ',' << r.z << ')';
	return os;
}

TPoint rotatePointR(const TPoint &p, const double alpha) {
	const double s = sin(alpha);
	const double c = cos(alpha);

	return TPoint(p.x*c - p.y*s, p.x*s + p.y*c);
}

TPoint translatePointR(const TPoint &p, const double tx, const double ty) {
	return TPoint(p.x + tx, p.y+ ty);
}

void rotatePoint(TPoint &p, const double alpha) {
	const double s = sin(alpha);
	const double c = cos(alpha);

	const double xx = p.x*c - p.y*s;
	const double yy = p.x*s + p.y*c;

	p.x = xx;
	p.y = yy;
}


void rotatePoint(TPoint &p, const double alpha, const TPoint &center) {
	const double s = sin(alpha);
	const double c = cos(alpha);

	const double xx = p.x*c - p.y*s - center.x*c + center.y*s + center.x;
	const double yy = p.x*s + p.y*c - center.x*s - center.y*c + center.y;

	p.x = xx;
	p.y = yy;
}

double distancePointEucleidSquared(const TPoint &a, const TPoint &b) {
	return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
}

double distancePointEucleid(const TPoint &a, const TPoint &b) {
	return sqrt(distancePointEucleidSquared(a,b));
}

void translatePoint(TPoint &p, const double tx, const double ty) {
	p.x += tx;
	p.y += ty;
}

TPoint transformPointRT_R(const TPoint &p, const double dx, const double dy, const double alpha) {
	return translatePointR(rotatePointR(p,alpha),dx,dy);
}


TPoint transformPointTR_R(const TPoint &p, const double dx, const double dy, const double alpha) {
	return rotatePointR(translatePointR(p,dx,dy),alpha);
}


/** return angle of segment from p1 to p2 */
double getAnglePoints(const TPoint &p1, const TPoint &p2) {
	return atan2(p2.y-p1.y,p2.x-p1.x);
}


double getAngleRad(const TPoint &p1, const TPoint &p2, const TPoint &p3) {

	const double a = sqrt( 
		((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)));
	
	const double b = sqrt(
			( p2.x - p3.x)*( p2.x -p3.x ) +	( p2.y - p3.y )*( p2.y - p3.y )	);
			
	const double c = sqrt(
			( p1.x - p3.x)*(p1.x - p3.x ) +	( p1.y - p3.y )*( p1.y - p3.y ));


	if (a == 0 || b == 0) {
		return 0;
	}	

	double aco =  acos((a*a+b*b-c*c)/(2*a*b));

	if (isnan(aco)) {
		const double bn = p3.x - p1.x;
		const double an = p1.y - p3.y;
		const double cn = -an*p1.x - bn*p1.y;
		const double dist = fabs(an*p2.x+bn*p2.y+cn)/sqrt(an*an + bn*bn);
		if (dist < 0.05) {
			aco = 0;
		} else {
			std::cerr << "Point near collinera: d="<<dist<<"\n";
			exit(0);
		}
	}
	return aco;
}



std::string getCPGName(const TCPG cpgtype) {
	switch(cpgtype) {
		case SINUSCPG: return std::string("SinusCPG"); break;
		case NONLINEARCPG: return std::string("NonlinearCPG"); break;
        case HOPFCPG: return std::string("HopfCPG"); break;
		default: return string("unknownCPG"); break;						   
	}
	return string("unknownCPG");
}

std::string getCPGAllNames() {
	std::stringstream ss;
	for(int i = SINUSCPG; i < CPG_NONE; i++) {
		ss << i << "=" << getCPGName((TCPG)i) << ",";
	}
	return ss.str();
}

vector<TPoint> getVoronoiSegments(Voronoi::VoronoiDiagramGenerator &vdg) {
	vdg.resetIterator();

	Voronoi::GraphEdge e;
	vector<TPoint> result;

	while(vdg.getNext(e)) {
			result.push_back(TPoint(e.x1,e.y1));
			result.push_back(TPoint(e.x2,e.y2));
	}
	return result;
}

std::vector< std::vector<TPoint > > getVoronoiCells(Voronoi::VoronoiDiagramGenerator &vdg) {
    // for each input site, return list of segments making its voronoi cell
    using std::vector;
    int maxPoint = 0;
	vdg.resetIterator();
	Voronoi::GraphEdge e;
	while(vdg.getNext(e)) {
            maxPoint = std::max(maxPoint, e.point1);
            maxPoint = std::max(maxPoint, e.point2);
	}

    vector< vector<TPoint > > cells;
    maxPoint++;
    for(int i=0;i<maxPoint;i++) {
        cells.push_back( vector<TPoint>());
    }
    std::cerr << "nump=" << maxPoint << "\n";

	vdg.resetIterator();
	while(vdg.getNext(e)) {
            if (e.point1 >=0 && e.point1 < maxPoint) {
                cells[ e.point1 ].push_back( TPoint(e.x1,e.y1) );
                cells[ e.point1 ].push_back( TPoint(e.x2,e.y2) );
            }
            if (e.point2 >=0 && e.point2 < maxPoint) {
                cells[ e.point2 ].push_back( TPoint(e.x1,e.y1) );
                cells[ e.point2 ].push_back( TPoint(e.x2,e.y2) );
            }
	}

    vector< vector<TPoint > > cells2;
    cells.reserve(cells.size());
    for(int ci = 0; ci < (int)cells.size();ci++ ) {
        const vector<TPoint> &pts(cells[ci]);
        std::cerr << " pts[" << ci << "]: ";
        for(int i=0;i<(int)pts.size()/2;i++) {
            std::cerr << pts[2*i+0].x << ", " << pts[2*i+0].y << " | ";
            std::cerr << pts[2*i+1].x << ", " << pts[2*i+1].y << "\n";
        }
        std::cerr << "\n";

        vector<bool> isUsed( pts.size(), false );
        int actual = 0;
        cells2.push_back( vector<TPoint>() );
        cells2.back().push_back(pts[actual]);
        cells2.back().push_back(pts[actual+1]);
        isUsed[0] = true;
        isUsed[1] = true;
        while(1) {
            double mind = -1;
            int next = -1;
            for(int i=0; i <(int)pts.size()/2;i++) {
                if (isUsed[2*i] == false) {
                    const double dx = pts[2*actual+1].x - pts[2*i+0].x;
                    const double dy = pts[2*actual+1].y - pts[2*i+0].y;
                    const double d = dx*dx + dy*dy;
                    if (d < mind || mind == -1) {
                        mind = d;
                        next = i;
                    }
                }
            }
            if (next != -1) {
                std::cerr << "Site " << ci << " a=" << actual << ",n=" << next << ", d=" << mind << "\n";
                std::cerr << printString(isUsed) << "\n";
                cells2.back().push_back(pts[2*next+0]);
                cells2.back().push_back(pts[2*next+1]);
                isUsed[2*next+0] = true;
                isUsed[2*next+1] = true;
                actual = next;
            } else {
                break;
            }
        }
    }
    return cells2;

}




/** for given border points (e.g. from SDimension.borderLine), creates extra 8 triangles of the given width 
assumption: pts.size() == 4 and their organiation is:
^y
|
| p3  p2
|
| p0  p1
|-------> x


*/
void makeBorderTriangles(const vector<TPoint> &pts, vector< vector< TPoint > > &triangles, const double borderWidth) {

    triangles.clear();

    //bottom triangles p0 to p1
    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(pts[0]);
    triangles.back().push_back( TPoint(pts[0].x, pts[0].y-borderWidth));
    triangles.back().push_back(pts[1]);

    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back( TPoint(pts[0].x, pts[0].y-borderWidth));
    triangles.back().push_back(pts[1]);
    triangles.back().push_back( TPoint(pts[1].x, pts[1].y-borderWidth));

    
    //right p1 to p2
    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(pts[1]);
    triangles.back().push_back(TPoint(pts[1].x+borderWidth, pts[1].y));
    triangles.back().push_back(pts[2]);

    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(TPoint(pts[1].x+borderWidth, pts[1].y));
    triangles.back().push_back(pts[2]);
    triangles.back().push_back(TPoint(pts[2].x+borderWidth, pts[2].y));

    //top p2 to p3
    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(pts[2]);
    triangles.back().push_back(pts[3]);
    triangles.back().push_back(TPoint(pts[3].x, pts[3].y+borderWidth));

    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(pts[2]);
    triangles.back().push_back(TPoint(pts[3].x, pts[3].y+borderWidth));
    triangles.back().push_back(TPoint(pts[2].x, pts[2].y+borderWidth));

    // left p0 to p3
    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(pts[0]);
    triangles.back().push_back(pts[3]);
    triangles.back().push_back(TPoint(pts[0].x-borderWidth, pts[0].y));
    
    triangles.push_back( vector< TPoint > ());
    triangles.back().push_back(TPoint(pts[0].x-borderWidth, pts[0].y));
    triangles.back().push_back(pts[3]);
    triangles.back().push_back(TPoint(pts[3].x-borderWidth, pts[3].y));

}


/** recursively subdivide all triangles until maximum area of each triangle is maxArea */
void subdivide(const std::vector<Triangle> &triangles, std::vector<Triangle> &result, const double maxArea) {

    result.clear();
    for(int i=0;i<(int)triangles.size();i++) {
        if (triangles[i].area() > maxArea) {
            Triangle a,b;
            triangles[i].divide(a,b);
            result.push_back(a);
            result.push_back(b);
        } else {
            result.push_back(triangles[i]);
        }
    }

}




} // namespace



