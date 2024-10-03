
#include "painting.h"
#include "ut.h"

#include <sstream>

namespace rrtPlanning {

	
const char *painterNames[] = {
	"gui",
	"png",
	"pdf",
	"xfig",
    "gnuplot"
};

gPoint tgp(const TPoint &p, const bool resizeable) {
	return gPoint(p.x, p.y);
}


void drawTriangle(const Triangle &t,  const CColor &lineColor, const CColor &fillColor, CPainterBase *pa) {
    vector<TPoint> pts;
    pts.reserve(3);
    pts.push_back(TPoint(t.a.x, t.a.y));
    pts.push_back(TPoint(t.b.x, t.b.y));
    pts.push_back(TPoint(t.c.x, t.c.y));
    drawPlg(pts,0,2,lineColor, fillColor, pa);
}

void drawRobot(const std::vector< Triangle > &pts, CPainterBase *pa, const int type) {
    for(int i=0;i<(int)pts.size();i++) {
        drawTriangle(pts[i], CColor("red"), CColor("coral"), pa);
    }
}






void drawColorMap(CColorMap &c, CPainterBase *pa) {

    const int w = pa->getWidth();
    const int h = pa->getHeight();

    const int size = c.getNumColors();

    int sx = w / size;
    int sy = h / size;

    pa->begin();

    

    for(int i=0;i<size;i++) {
        vector<TPoint> pts;
        pts.push_back(TPoint(i*sx, 0));
        pts.push_back(TPoint((i+1)*sx, 0));
        pts.push_back(TPoint((i+1)*sx, h));
        pts.push_back(TPoint(i*sx, h));
        drawPlg(pts,0,1,c.getColorIdx(i), c.getColorIdx(i), pa);
    }


    pa->close();


}

void drawDim(const std::vector<double> &dim, const int ps, const int lw, const CColor &cLine, const CColor &cFill, CPainterBase *pa) {
    if (pa) {
        
        vector<TPoint> pts;
        pts.push_back(TPoint(dim[0], dim[2]));
        pts.push_back(TPoint(dim[1], dim[2]));
        pts.push_back(TPoint(dim[1], dim[3]));
        pts.push_back(TPoint(dim[0], dim[3]));
        drawPlg(pts, ps, lw, cLine, cFill, pa);

    }
}



void drawPl_2(const TPoint &p1, const TPoint &p2, const int lw, const CColor &cc, CPainterBase *pa) {
    std::vector<gPoint> l;
    l.push_back(tgp(p1));
    l.push_back(tgp(p2));
    if (pa) {
        pa->draw(l,lw,cc);
    }
    l.clear();
}


int xyz2idx(const TPoint3 &p, const vector<double> &mapDimension, const double resolution, 
        const int nx, const int ny, const int nz ) {
    const int x = (int)lround(fabs((p.x - mapDimension[0])/ resolution));
    const int y = (int)lround(fabs((p.y - mapDimension[2])/ resolution));
    const int z = (int)lround(fabs((p.z - mapDimension[4])/ resolution));
    return z * nx*ny + y*nx + x;
}



void drawRobot(const std::vector<TShape> &s, CPainterBase *pa) {

    for(int i=0;i<(int)s.size();i++) {
        if (s[i].type == PTS_ONLY) {
            drawPts(s[i].pts, s[i].ptsSize,s[i].ptsLineWidth,s[i].ptsColor,s[i].ptsFillColor, pa);
        } else if (s[i].type == LINE_ONLY) {
            drawPl(s[i].pts,s[i].lineWidth,s[i].lineColor,pa);
        } else if (s[i].type == PTS_AND_LINE) {
            drawPl(s[i].pts,s[i].lineWidth,s[i].lineColor,pa);
            drawPts(s[i].pts, s[i].ptsSize,s[i].ptsLineWidth,s[i].ptsColor,s[i].ptsFillColor, pa);
        } else if (s[i].type == POLY_ONLY) {
            drawPlg(s[i].pts, s[i].ptsSize, s[i].lineWidth, s[i].lineColor, s[i].fillColor, pa);
        } else if (s[i].type == PTS_AND_POLY) {
            drawPlg(s[i].pts, s[i].ptsSize, s[i].lineWidth, s[i].lineColor, s[i].fillColor, pa);
            drawPts(s[i].pts, s[i].ptsSize,s[i].ptsLineWidth,s[i].ptsColor,s[i].ptsFillColor, pa);
        } 
    }

}



bool equalPoints(const std::vector<TPoint> &pts) { 
	bool same = true;
	for(int i=0;i<(int)pts.size() && same;i++) {
		same = (pts[0].x == pts[i].x && pts[0].y == pts[i].y);
	}
	return same;
}


/** draw point described by 'pts'.
  * if drawEdges == 1, also drawedges between these poits whose distance mm[i][j] > 0
  * size of a matrix mm is pts.size() x pts.size()
  */
void drawPointGraph(double **mm, const std::vector<TPoint> &pts, const int drawEdges, CPainterBase *pa) {
	if (pa) {
		vector<TPoint> pp;
        pp.reserve(3);
		if (drawEdges && mm != NULL) {
			for(int i=0;i<(int)pts.size()-1;i++) {
				for(int j=i+1;j<(int)pts.size();j++) {
					if (mm[i][j] > 0) {
						pp.push_back(pts[i]);
						pp.push_back(pts[j]);
						drawPl(pp,1,CColor("darkslategrey"),pa);
						pp.clear();	
					}
				}
			}
		}
//		drawPts(pts.begin(),pts.end(),2,1,CColor("black"),CColor("black"),pa);
	}
}


void draw(const TPoint &p, const int ps, const int lw, const CColor &c, 
		const CColor &cf, CPainterBase *pa, const bool resizeable) {
	
	if (pa) {
		pa->draw(tgp(p,resizeable),ps,lw,c,cf);
	}
}




/** make painter according to given painter type */
CPainterBase *initPainter(enum _TPainter type, const std::vector<double> &dimension, const char *fileMask, 
		CPainterBase *dump,const double scale) {
	CPainterBase *painter = NULL;
	if (type == GUI) {
		painter = new CPainterGui(400,400,dimension,1,dump);
	} else if (type == PNG) {
		painter = new CPainterCairo(fileMask,CPainterCairo::PNG,dimension,dump,scale);
	} else if (type == PDF) {
		painter = new CPainterCairo(fileMask,CPainterCairo::PDF,dimension,dump,scale);
	} else if (type == XFIG) {
		painter= new CPainterXFig(fileMask,dimension,dump);
	} else if (type == GNUPLOT) {
		painter= new CPainterGnuplot(dump);
	} else {
		WDEBUG("painter is NULL");
	}
	return painter;
}


std::string printPainterHelp() {
	stringstream s;
	s << GUI << "=" << painterNames[GUI] << "," <<
	     PNG << "=" << painterNames[PNG] << "," <<
	 	 PDF << "=" << painterNames[PDF] << "," <<	 
		 XFIG << "=" << painterNames[XFIG] << "," << 
		 GNUPLOT << "=" << painterNames[GNUPLOT];
	return s.str();
}



// drawExpensiveEdges = -1 -> normal graph
//                    = X>=0 -> edges with cost hiher than this value are painter in diffrerent color
void drawMesh(const Mesh &mesh, CPainterBase *pa, const double drawExpensiveEdges) {
        
    vector<TPoint> pts;

    if (mesh.getMeshType() == Mesh::TYPE_MESH) {
        for(int i=0;i<(int)mesh.getTriangles().size();i++) {
            vector< TPoint > pl(mesh.getTriangles()[i]);
            pl.push_back(pl.front());
            drawPl(pl, 1, CColor("darkslategrey"),pa);
        }
    } else {
        // draw vorooni edge
        for(int i=0;i<(int)mesh.getEdges().size();i++) {
            const SEdge &e(mesh.getEdges()[i]);
                pts.push_back(mesh.getPoint(e.from));
                pts.push_back(mesh.getPoint(e.to));
                drawPl(pts, 1, CColor("darkslategrey"),pa);
                pts.clear();
            }
    }

    if (drawExpensiveEdges > 0) {
        for(int i=0;i<(int)mesh.getEdges().size();i++) {
            const SEdge &e(mesh.getEdges()[i]);
            if (e.cost >= drawExpensiveEdges) {
                pts.push_back(mesh.getPoint(e.from));
                pts.push_back(mesh.getPoint(e.to));
                drawPl(pts, 1, CColor("red"),pa);
                pts.clear();
            }
        }
    }

}

/** for map ISO or TRI 
void drawMap(const Map &m, CPainterBase *pa) {
	const std::vector<TPoint> points(m.getMap());

	const int alpha = 255; // 255 for normal map, 0 for transparent map

	//drawPlg(points.begin(),points.end(),0,3,CColor("royalBlue1"),CColor("azure2"),pa);
	if (pa) {
		const std::vector<double> dim(m.getDimension());

        std::vector<TPoint> pts;
		pts.push_back(TPoint(dim[0],dim[2]));
		pts.push_back(TPoint(dim[1],dim[2]));
		pts.push_back(TPoint(dim[1],dim[3]));
		pts.push_back(TPoint(dim[0],dim[3]));

		pa->setParam("depth","56");
#ifdef BW
		drawPlg(pts,0,3,CColor("grey60"),CColor("grey60"),pa);
#else
		drawPlg(pts,0,3,CColor("royalBlue1",alpha),CColor("slateblue1",alpha),pa);
#endif

		pa->setParam("depth","54");
#ifdef BW
		drawPlg(points,0,3,CColor("grey60"),CColor("white"),pa);
#else
		drawPlg(points,0,3,CColor("royalBlue1",alpha),CColor("white",alpha),pa);
#endif
		pa->setParam("depth","52");
		const std::vector< std::vector<TPoint> > &obstacles(m.getObstacles());
//		const vector<double> dimpl(getDimensionPolygons(obstacles));
//		char name[200];
//		snprintf(name,sizeof(name)," %lf %lf %lf %lf",dimpl[0], dimpl[1], dimpl[2], dimpl[3]);
//		pa->setParam("groupb",std::string(name));

		for(int i=0;i<(int)obstacles.size();i++) {
#ifdef BW
			drawPlg(obstacles[i],0,3,CColor("grey50"),CColor("grey60"),pa);
#else
			drawPlg(obstacles[i],0,3,CColor("royalBlue1",alpha),CColor("slateblue1",alpha),pa);
#endif
		}
//		pa->setParam("groupe","");
		pa->setParam("depth","50");
	}
}

*/



void drawMap(const MapR &m, CPainterBase *pa) {
	if (pa) {

        const SDimension &dim(m.getDimension());
        std::vector<TPoint> pts;

        dim.borderLine(pts);
		pa->setParam("depth","54");	
		drawPlg(pts,0,3,CColor("royalBlue1"),CColor("white"),pa);

		const std::vector< Triangle > &obstacles(m.getTriangles());

		pa->setParam("depth","52");
		for(int i=0;i<(int)obstacles.size();i++) {
            drawTriangle(obstacles[i],CColor("slateblue1"),CColor("slateblue1"),pa);
		}
		pa->setParam("depth","50");
	}
}



void createCylinder(std::vector< std::vector< TPoint3 > > &cyl, const double length, const double radius, const int n) {
	cyl.clear();
	for(int i=0;i<n;i++) {
		const double a1 = i*2*M_PI/n;
		const double a2 = (i+1)*2*M_PI/n;
		const TPoint3 a(-length/2,radius*cos(a1),radius*sin(a1));
		const TPoint3 b(-length/2,radius*cos(a2),radius*sin(a2));
		const TPoint3 c(length/2,radius*cos(a1),radius*sin(a1));
		const TPoint3 d(length/2,radius*cos(a2),radius*sin(a2));
		
		cyl.push_back(vector<TPoint3>());
		cyl.back().push_back(a);
		cyl.back().push_back(b);
		cyl.back().push_back(c);
		cyl.push_back(vector<TPoint3>());
		cyl.back().push_back(b);
		cyl.back().push_back(c);
		cyl.back().push_back(d);
	}
}

/* draw 3D cylinder defined by two points and represented by 3D triangles 
   resolution is number of faces */
void raw_draw_cylinder(const TPoint3 &p1in, const TPoint3 &p2in, const double radius, const int resolution, ofstream &ofs) {
	
	std::vector< vector<TPoint3 > > triangles;
	const double l = pointDistanceEucleid(p1in,p2in);
    if (l < 0.001) {
        return;
    }
	createCylinder(triangles,l,radius,resolution);
	TPoint3 p1(p1in);
	TPoint3 p2(p2in);
    
	const double a = atan2(p2.y-p1.y,p2.x-p1.x);
	const double b = atan2(p2.z-p1.z,p2.x-p1.x);

//    std::cerr << "a=" << a << "\nb="<<b << "\n\n";

    TPoint3 ps(getMiddlePoint(p1in,p2in));
    double rotx,roty,rotz;
    rotx = 0;
    roty = -b;
    rotz = a;
//    getRotAngles(p1in,p2in,rotz,roty,rotx);

//    std::cerr << "rx="<<rotx <<", ry=" << roty << ", rz="<< rotz << "\n";
    const double ca = cos(rotz);
    const double sa = sin(rotz);
    const double cb = cos(roty);
    const double sb = sin(roty);
    const double cg = cos(rotx);
    const double sg = sin(rotx);
    // normal
    double matrix[3][3];
    matrix[0][0] = ca*cb;
    matrix[0][1] = ca*sb*sg-sa*cg;
    matrix[0][2] = ca*sb*cg+sa*sg;

    matrix[1][0] = sa*cb;
    matrix[1][1] = sa*sb*sg+ca*cg;
    matrix[1][2] = sa*sb*cg-ca*sg;

    matrix[2][0] = -sb;
    matrix[2][1] = cb*sg;
    matrix[2][2] = cb*cg;
    

	for(int i=0;i<(int)triangles.size();i++) {
		for(int j=0;j<(int)triangles[i].size();j++) {
            /*
			const double nx = triangles[i][j].x*cos(a) - triangles[i][j].y*sin(a);
			const double ny = triangles[i][j].x*sin(a) + triangles[i][j].y*cos(a);
			triangles[i][j].x = nx;
			triangles[i][j].y = ny;

			const double nx2 = triangles[i][j].x*cos(b) - triangles[i][j].z*sin(b);
			const double nz = triangles[i][j].x*sin(b) + triangles[i][j].z*cos(b);
			triangles[i][j].x = ps.x + nx2;
			triangles[i][j].z = ps.z + nz;
			triangles[i][j].y += ps.y; 
            */
            const TPoint3 &p(triangles[i][j]);
            const double nx = matrix[0][0]*p.x + matrix[0][1]*p.y + matrix[0][2]*p.z;
            const double ny = matrix[1][0]*p.x + matrix[1][1]*p.y + matrix[1][2]*p.z;
            const double nz = matrix[2][0]*p.x + matrix[2][1]*p.y + matrix[2][2]*p.z;
            triangles[i][j].x = nx+ps.x;
            triangles[i][j].y = ny+ps.y;
            triangles[i][j].z = nz+ps.z;
		}
	}

	for(int i=0;i<(int)triangles.size();i++) {
		for(int j=0;j<(int)triangles[i].size();j++) {
			ofs << triangles[i][j].x << " " << triangles[i][j].y << " " << triangles[i][j].z << " ";
		}
		ofs << "\n";
	}
	triangles.clear();
}



/** draw 3D line consisting of cylinder represented by 3D triangles .. */
void raw_drawPl(const vector<TPoint3> &pts, ofstream &ofs, const double radius) {
	for(int i=0;i<(int)pts.size()-1;i++) {
		raw_draw_cylinder(pts[i],pts[i+1],radius,10,ofs);
	}
}

} // namespace rrtPlanning

