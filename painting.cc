
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

gPoint3 tgp3(const TPoint3 &p) {
	return gPoint3(p.x,p.y,p.z);
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




std::vector<gPoint3> tgp3(const std::vector<TPoint3> &pts) {
	std::vector<gPoint3> res;
	res.reserve(pts.size());
	for(int i=0;i<(int)pts.size();i++) {
		res.push_back(tgp3(pts[i]));
	}
	return res;
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



std::string povPoint(const double &x, const double &y, const double &z) {
	std::stringstream ss;
	ss << "<"<<x<<","<<y<<","<<z<<">";
	return ss.str();
}

std::string povPoint(const TPoint3 &point) {
	return povPoint(point.x, point.y, point.z);
}


void draw(const TPoint &p, const int ps, const int lw, const CColor &c, 
		const CColor &cf, CPainterBase *pa, const bool resizeable) {
	
	if (pa) {
		pa->draw(tgp(p,resizeable),ps,lw,c,cf);
	}
}

void povIfObject(std::ofstream &ofs, const std::string &name) {
    ofs << "#ifdef (" << name << ")\n";
    ofs << "  object { " << name << " }\n";
    ofs << "#end\n";
}

void povInclude(std::ofstream &ofs, const std::string &name) {
    ofs << "#include \"" << name << "\"\n";
}

void povDeclareMap(const std::vector< Triangle > &triangles, std::ofstream &ofs) {
	ofs << "#include \"textures.inc\"\n";
	ofs << "#declare map=\n";
	ofs << "mesh{ // num triangles=" << triangles.size() << "\n";
	ofs << printPOVtriangles(triangles) << "\n";
//	ofs << " pigment { rgb " << povColor(CColor("royalBlue1")) << "}";
//	ofs << " pigment { brick rgb " << povColor(CColor("blue")) << ", rgb " << povColor(CColor("ivory3")) << " scale 0.1}";
	ofs << "texture {New_Brass}\n";
    ofs << "#if (mapOpacity=-1)\n";
    CColor mcolor;
    mcolor.setFloatRGB(0.8,0.8,0.8);
    ofs << "pigment { " << povColor(mcolor) << "}\n";
//    ofs << "pigment { " << povColor(CColor("royalBlue1")) << "}\n";	
    ofs << "#else\n";
    CColor c("royalBlue1");
    ofs << "pigment { color rgbf <"<<c.getRedf() << "," << c.getGreenf() <<","<<c.getBluef() << ",mapOpacity> }\n";
    ofs << "#end\n";
    ofs << "\n} // map\n";	
}

/*
void povDrawRobot(const std::vector<TPoint3> &triangles, std::ofstream &ofs) {
	ofs << "// robot\n";
	ofs << "mesh{\n";
	ofs << printPOVtriangle(triangles) << "\n";
//	ofs << " pigment {checks rgb " << povColor(CColor("red1")) << "}";
	ofs << " pigment {brick rgb " << povColorRaw(CColor("red1")) << ", rgb " << povColorRaw(CColor("green")) << " scale 0.08}";
    ofs << "\n} // robot\n";	
}
*/

void povDrawPoint(const TPoint3 &point, const double radius, const CColor &color, std::ofstream &ofs) {
	ofs << "sphere {" << povPoint(point) << "," << radius << " pigment { " << povColor(color) << "} }\n";
}

void povDrawPoint(const TPoint3 &point, const string &radius, const CColor &color, std::ofstream &ofs) {
	ofs << "sphere {" << povPoint(point) << "," << radius << " pigment { " << povColor(color) << "} }\n";
}


void writeLights(const vector<TLight> &lights, ofstream &ofs) {
    for(int i=0;i<(int)lights.size();i++) {
		ofs << "light(" << lights[i].p.x << "," << lights[i].p.y << "," << lights[i].p.z << "," 
            << lights[i].shadowless << ", " << lights[i].energy << ")\n";
    }
    ofs << "\n\n";    
}


void writeMacros(ofstream &ofs) {
	const char *sep = "//--------------------------------------------------------------------\n";
	ofs << "//declare treeNoshadow=1;\n";
	ofs << "//declare trajNoshadow=1;\n\n";
    ofs << "// general treeSegment\n";
    ofs << "#macro treeSegment(a,b,c,d,e,f,segmentColor)\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,rrtWidth pigment { color segmentColor }\n";
	ofs << "#ifdef (treeNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";
	ofs << sep;	


    ofs << "#macro ars(arsLength)\n";
    ofs << "union{\n";
	ofs << "cylinder { <0,0,0>, <arsLength,0,0>, 0.1 pigment {color rgb <1,0,0> } } \n";
	ofs << "cylinder { <0,0,0>, <0,arsLength,0>, 0.1 pigment {color rgb <0,1,0> } } \n";
	ofs << "cylinder { <0,0,0>, <0,0,arsLength>, 0.1 pigment {color rgb <0,0,1> } } \n";
    ofs << "}\n";
    ofs << "#end // macro ars\n\n";
    ofs << "// ars(5)\n\n";





    ofs << "#macro treeSegmentNS(a,b,c,d,e,f,segmentColor)\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,rrtWidth pigment { color segmentColor }\n";
	ofs << "no_shadow\n";
	ofs << "}\n";
    ofs << "#end\n\n";
	ofs << sep;	


    ofs << "// treeSegment for RANDANGLES\n";
    ofs << "#macro treeRA(a,b,c,d,e,f)\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,rrtWidth pigment { color treeColorRANDANGLES }\n";
	ofs << "#ifdef (treeNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";	
	ofs << sep;	
    
    ofs << "// treeSegment for RANDSINUS\n";
    ofs << "#macro treeRS(a,b,c,d,e,f)\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,rrtWidth pigment { color treeColorRANDSINUS }\n";
	ofs << "#ifdef (treeNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";	
	ofs << sep;	

    ofs << "// treeSegment for motion primitives\n";
    ofs << "#macro treeMP(a,b,c,d,e,f)\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,rrtWidth pigment { color treeColorMP }\n";
	ofs << "#ifdef (treeNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";	
	ofs << sep;	

    ofs << "// treeSegment for motion primitives, color is sucked from segmentColor\n";
    ofs << "#macro treeMP2(a,b,c,d,e,f,idx)\n";
    ofs << "#declare useColor = segmentColor[idx];\n";
    ofs << "#ifdef (treeOneColor)\n";
    ofs << "   #declare useColor = treeColorMP;\n";
    ofs << "#end\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,rrtWidth pigment { color useColor }\n";
	ofs << "#ifdef (treeNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";	
	ofs << sep;	


    ofs << "// treeTrajectory for motion primitives\n";
    ofs << "#macro traj(a,b,c,d,e,f)\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,trajectoryWidth pigment { color trajectoryColor }\n";
	ofs << "#ifdef (trajNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";	
	ofs << sep;	

    ofs << "// treeTrajectory for motion primitives, using index\n";
    ofs << "#macro traj2(a,b,c,d,e,f,index)\n";
    ofs << "#declare useColor = segmentColor[index];\n";
    ofs << "#ifdef (trajectoryOneColor)\n";
    ofs << "   #declare useColor = trajectoryColor;\n";
    ofs << "#end\n";
    ofs << "cylinder {<a,b,c>,<d,e,f>,trajectoryWidth pigment { color useColor }\n";
	ofs << "#ifdef (trajNoshadow)\n";
	ofs << "no_shadow\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end\n\n";	
	ofs << sep;	


	ofs << "#macro light(lx,ly,lz,noshadow,value)\n";
 	ofs << "light_source { <_LX*lx,_LY*ly,lz> color rgb <value,value,value>\n";
	ofs << "#if (noshadow=1)\n";
    ofs << "    shadowless\n";
	ofs << "#end\n";
    ofs << "#ifndef (numAreaLights)\n";
    ofs << "   #declare numAreaLights=16;\n";
    ofs << "#end\n";
	ofs << "#ifdef (hifi)\n";
    ofs << "  #if (hifi=1)\n";
    ofs << "     area_light <14,0,0> <0,14,0> numAreaLights numAreaLights\n";
    ofs << "     adaptive 1\n";
    ofs << "     jitter\n";
    ofs << "  #end\n";
	ofs << "#end\n";
	ofs << "}\n";
    ofs << "#end//macro light\n\n";
	ofs << sep;	

    ofs << " #declare CloudArea = texture { pigment { agate turbulence 0.4 lambda 0.01 frequency 0.5 color_map { [0.0 color rgbf <1, 1, 1, 1>] [0.8 color rgbf <1, 1, 1, .35>] [1.0 color rgbf <1, 1, 1, 1>] } } }\n\n";


    ofs << "#macro TSPHERE(xx,yy,zz,rr,cc)\n";
    ofs << "  sphere { <xx,yy,zz>,rr \n";
    ofs << "  texture { pigment {color cc filter 0.97 } finish { phong 0.11 } }\n}\n";
    ofs << "#end // macro TSPHERE\n\n"; 

    ofs << "#macro TSPHERECM(xx,yy,zz,rr,intvalue)\n";
    ofs << "  sphere { <xx,yy,zz>,rr \n";
    ofs << "  texture { pigment {color cmColor(intvalue) filter 0.97 } finish { phong 0.11 } }\n}\n";
    ofs << "#end // macro TSPHERE\n\n"; 

    CColorMap::saveCMStoPovray(ofs);

}





void drawLine(std::ofstream &ofs, const TPoint3 &p1, const TPoint3 &p2, const double radius, const CColor &color) {
	ofs << "cylinder {"<<povPoint(p1) <<','<<povPoint(p2)<<','<<radius<<" pigment { " << povColor(color) << "}}\n";
}


void drawLine(std::ofstream &ofs, const TPoint3 &p1, const TPoint3 &p2, const char *radius, const CColor &color) {
	ofs << "cylinder {"<<povPoint(p1) <<','<<povPoint(p2)<<','<<radius<<" pigment { " << povColor(color) << "}}\n";
}


void drawTriangle(std::ofstream &ofs,const TPoint3 &p1, const TPoint3 &p2, const TPoint3 &p3, const CColor &color) {
	ofs << "triangle { " << povPoint(p1) << ","<<povPoint(p2) << "," << povPoint(p3) << " ";
	ofs << "pigment { " << povColor(color) << " } }\n";
}


void drawTriangle(std::ofstream &ofs,const TPoint3 &p1, const TPoint3 &p2, const TPoint3 &p3) {
	ofs << "triangle { " << povPoint(p1) << ","<<povPoint(p2) << "," << povPoint(p3) << "}";
}

void drawPoint(std::ofstream &ofs, const TPoint3 &pts, const double size, const CColor &color) {
	ofs << "sphere { " << povPoint(pts) << "," << size << " pigment { " << povColor(color) << "}}\n";
}

void drawMesh(ofstream &ofs, const vector<TPoint3> &pts, const int indicesSize, int *indices, 
		const bool drawEdges, const bool drawColorHeightField, const char *colorMapName) {


	if (drawColorHeightField) {
		// triangle colors are defined according to its height
		double minz = 0;
		double maxz = 0;
		bool f=true;
		for(int i=0;i<(int)pts.size();i++) {
			if (pts[i].z < minz || f) {
				minz = pts[i].z;
			}
			if (pts[i].z > maxz || f) {
				maxz = pts[i].z;
			}
			f = false;
		}
		//CColorMap cm;
		CColorMap cm(colorMapName,true);
//CColorMap cm("autumn");
//		CColorMap cm("copper",true); // hneda
//		CColorMap cm("bone",true); // modra
		cm.setRange(minz,1.5*maxz);
		ofs << "// drawed using colormap " << cm.getName() << "\n";
		ofs << "union {\n";

		double aa;
		int idx = 0;
		for(int i=0;i<(int)indicesSize/3;i++) {
			aa = (pts[indices[3*idx+0]].z + pts[indices[3*idx+1]].z + pts[indices[3*idx+2]].z)/3.0;
			ofs << "mesh { ";
			drawTriangle(ofs,pts[indices[3*idx+0]], pts[indices[3*idx+1]], pts[indices[3*idx+2]]);
			ofs << " pigment {" << povColor(cm.getColor(aa)) << "}";
			// mars surface - you have to include "mars.inc" from /scripts
			//ofs << " finish { marsfinish } normal { marsnormal } ";
			ofs << "}\n ";
			idx++;
		}

		ofs << "}\n";
	} else {
        /*
		ofs << "mesh { \n";
		int idx = 0;
		for(int i=0;i<(int)indicesSize/3;i++) {
			drawTriangle(ofs,pts[indices[3*idx+0]], pts[indices[3*idx+1]], pts[indices[3*idx+2]]);
			ofs << "\n";
			idx++;
		}	
		ofs << "pigment {" << povColor(CColor("burlywood3")) << "\n}}";
		
		idx = 0;
		if (drawEdges) {
			const double edgeWidth = 0.005;
			for(int i=0;i<(int)indicesSize/3;i++) {
				drawLine(ofs,pts[indices[3*i+0]], pts[indices[3*i+1]],edgeWidth,CColor("brown"));
				drawLine(ofs,pts[indices[3*i+0]], pts[indices[3*i+2]],edgeWidth,CColor("brown"));
				drawLine(ofs,pts[indices[3*i+1]], pts[indices[3*i+2]],edgeWidth,CColor("brown"));
			}
		}
        */
        // draw mesh2 object
        ofs << "mesh2 {\n";
        ofs << "vertex_vectors { " << pts.size() << ",\n";
        for(int i=0;i<(int)pts.size();i++) {
            ofs << "<" << pts[i].x << "," << pts[i].y << "," << pts[i].z << ">";
            if (i < (int)pts.size()-1) {
                ofs << ",";
            }
            if ((i % 3) == 0) {
                ofs << "\n";
            }
        }
        ofs << "}\n";
        ofs << "face_indices { " << indicesSize/3 << ",\n";
        for(int i=0;i<(int)indicesSize/3;i++) {
           ofs << "<" << indices[3*i+0] << "," << indices[3*i+1] << "," << indices[3*i+2] << ">";
           if (i < indicesSize/3-1) {
                ofs << ",";
           } 
           if ((i % 10) == 0) {
                ofs << "\n";
           }
        }
        ofs << "}\n";
        ofs << "pigment { " << povColor(CColor("burlywood3")) << "}\n";
        ofs << "}\n";
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

string povColor(const CColor &color) {
	stringstream ss;
	ss << "color rgb";
	if (color.getA() == 255) {
		ss << " <" << color.getRedf() << "," << color.getGreenf() << "," << color.getBluef() << ">";
	} else {
		ss << "f <" << color.getRedf() << "," << color.getGreenf() << "," << color.getBluef() << "," << (1.0-color.getAf()) << ">";
	}
	return ss.str();
}

string povColorRaw(const CColor &color) {
	stringstream ss;
	ss << " <" << color.getRedf() << "," << color.getGreenf() << "," << color.getBluef() << ">";
	return ss.str();
}



void povDeclareCoord(std::ofstream &ofs) {
    /*
	ofs << "#declare arsLength = 2;\n";
	ofs << "#declare ars=\nunion{\n";
	ofs << "cylinder { <0,0,0>, <arsLength,0,0>, 0.1 pigment {color rgb <1,0,0> } } \n";
	ofs << "cylinder { <0,0,0>, <0,arsLength,0>, 0.1 pigment {color rgb <0,1,0> } } \n";
	ofs << "cylinder { <0,0,0>, <0,0,arsLength>, 0.1 pigment {color rgb <0,0,1> } } \n";
	ofs << "}\n";
	ofs << "// object { ars }\n";
    */
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


/** for map ISO or TRI */
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



/** colorize robot according to size of the triangles */
void povDeclareRobotColorMap(const std::vector< std::vector< TPoint3> > &triangles, std::ofstream &ofs) {

    double min = 0;
    double max = 0;
    for(int i=0;i<(int)triangles.size();i++) {
        const double area = triangleArea(triangles[i]);
        if (area < min || i == 0) {
            min = area;
        }
        if (area > max || i == 0) {
            max = area;
        }
    }
    CColorMap cm("jet");
    cm.setRange(min,max);

	ofs << "#include \"textures.inc\"\n";
	ofs << "#include \"glass.inc\"\n";
	ofs << "#declare robot=\n";
	ofs << "union{ // robot\n";
    for(int i=0;i<(int)triangles.size();i++) {
        ofs << "mesh { triangle { ";
		for(int j=0;j<3;j++) {
			ofs << povPoint(triangles[i][j]);
			if (j < 2) {
				ofs << ",";
			}
		}
        ofs << " } pigment { color rgb " << povColorRaw(cm.getColor(triangleArea(triangles[i]))) << "}}\n";
    }
    ofs << "\n} // robot\n";	
}



/** also print rotation and translation of the robot*/
void povDeclareRobot(const std::vector< Triangle > &triangles, std::ofstream &ofs, const CColor &color) {
	ofs << "#include \"textures.inc\"\n";
	ofs << "#include \"glass.inc\"\n";
	ofs << "#declare robot=\n";
	ofs << "mesh{ // robot\n";
	ofs << printPOVtriangles(triangles) << "\n";
	ofs << " //pigment {checker rgb " << povColorRaw(CColor("blue1")) << ", rgb " << povColorRaw(CColor("green2")) << " scale 0.08}\n";
	//ofs << " texture { pigment { color rgb " << povColorRaw(color) << "}\n";
	ofs << " texture { pigment { color robotColor }\n";
	ofs << "           finish { diffuse 0.9 phong 0.5 } }\n";
    ofs << "\n} // robot\n";	
}


std::string printPOVtriangleDeformable(const std::vector< std::vector<TPoint3> > &triangles, const double scaleX, const double scaleY, const double scaleZ) {
	std::stringstream ss;
	for(int i=0;i<(int)triangles.size();i++) {
		ss << "triangle{";
		for(int j=0;j<3;j++) {
            TPoint3 aa(triangles[i][j].x*scaleX, triangles[i][j].y*scaleY, triangles[i][j].z*scaleZ);
			ss << povPoint(aa);
			if (j < 2) {
				ss << ",";
			}
		}
		ss << "}\n";
	}
	return ss.str();
}



std::string printPOVtriangle(const Triangle &triangle) {
    stringstream ss;
    ss << "triangle {" << povPoint(triangle.a) << "," << povPoint(triangle.b) << "," << povPoint(triangle.c) << "}\n";
    return ss.str();
}

std::string printPOVtriangles(const std::vector< Triangle > &triangles) {
	std::stringstream ss;
	for(int i=0;i<(int)triangles.size();i++) {
        ss << printPOVtriangle(triangles[i]);
	}
	return ss.str();
}

std::string dbl2str(const double d) {
    stringstream ss;
    ss << d;
    return ss.str();
}


/* draw 'polyline' using cylindres. The radius of the cilinder is given */
void povDrawPl(const std::vector<TPoint3> &pts, const double width, const CColor &color, std::ofstream &ofs, const bool noshadow) {
    povDrawPl(pts, dbl2str(width), color ,ofs, noshadow);
}

/* draw 'polyline' using cylindres. The radius of the cylinder is given as a variable name which
 * must be declaret previously in .inc or .pov file */
void povDrawPl(const std::vector<TPoint3> &pts, const std::string &width, const CColor &color, std::ofstream &ofs, const bool noshadow) {
	const double eps = 0.001;
    const string ns=noshadow?"no_shadow ":"";
	for(int i=0;i<(int)pts.size()-1;i++) {
		if (fabs(pts[i].x-pts[i+1].x) > eps ||
			fabs(pts[i].y-pts[i+1].y) > eps ||
			fabs(pts[i].z-pts[i+1].z) > eps) {
			ofs << "cylinder { " << povPoint(pts[i]) << "," << povPoint(pts[i+1]) << "," << width << " ";
			ofs << "pigment { rgb " << povColorRaw(color) << "} " << ns << "}\n";
		}
	}
}


/* draw 'polyline' using cylindres. The radius of the cylinder is given as a variable name which
 * must be declaret previously in .inc or .pov file 
 * color is given as a string, and must be declared in pov file like #declare color=rgb<1,0,0> */
void povDrawPl2(const std::vector<TPoint3> &pts, const std::string &width, const std::string &color, std::ofstream &ofs, const bool noshadow) {
	const double eps = 0.001;
    const string ns=noshadow?"no_shadow ":"";

	for(int i=0;i<(int)pts.size()-1;i++) {
		if (fabs(pts[i].x-pts[i+1].x) > eps ||
			fabs(pts[i].y-pts[i+1].y) > eps ||
			fabs(pts[i].z-pts[i+1].z) > eps) {
			ofs << "cylinder { " << povPoint(pts[i]) << "," << povPoint(pts[i+1]) << "," << width << " ";
			ofs << "pigment { color " << color << "} " << ns << " }\n";
		}
	}
}

void povDrawPlTrajSegment(const std::vector<TPoint3> &pts, std::ofstream &ofs, const int index) {
	const double eps = 0.001;
 //   const string ns=noshadow?"no_shadow ":"";

	for(int i=0;i<(int)pts.size()-1;i++) {
		if (fabs(pts[i].x-pts[i+1].x) > eps ||
			fabs(pts[i].y-pts[i+1].y) > eps ||
			fabs(pts[i].z-pts[i+1].z) > eps) {
            ofs << "traj2(" << pts[i].x << "," << pts[i].y << "," << pts[i].z << ",";
            ofs << pts[i+1].x << "," << pts[i+1].y << "," << pts[i+1].z << "," << index << ")\n";
//			ofs << "cylinder { " << povPoint(pts[i]) << "," << povPoint(pts[i+1]) << "," << width << " ";
//			ofs << "pigment { color " << color << "} " << ns << " }\n";
		}
	}
}


void povDrawTreeSegments(const std::vector<TPoint3> &pts, const std::string &color, std::ofstream &ofs, const bool noshadow) {
	const double eps = 0.001;
	for(int i=0;i<(int)pts.size()-1;i++) {
		if (fabs(pts[i].x-pts[i+1].x) > eps ||
			fabs(pts[i].y-pts[i+1].y) > eps ||
			fabs(pts[i].z-pts[i+1].z) > eps) {
            if (noshadow) {
				ofs << "treeSegmentNS("<<pts[i].x << ","<<pts[i].y << ","<<pts[i].z<<","<<pts[i+1].x<<","<<pts[i+1].y<<","<<pts[i+1].z<<"," << color<<")\n";
            } else {
				ofs << "treeSegment("<<pts[i].x << ","<<pts[i].y << ","<<pts[i].z<<","<<pts[i+1].x<<","<<pts[i+1].y<<","<<pts[i+1].z<<"," << color<<")\n";
            }
		}
	}
}

void povDrawLineMacro(const std::vector<TPoint3> &pts, const std::string &macroname, std::ofstream &ofs) {
	const double eps = 0.001;
	for(int i=0;i<(int)pts.size()-1;i++) {
		if (fabs(pts[i].x-pts[i+1].x) > eps ||
			fabs(pts[i].y-pts[i+1].y) > eps ||
			fabs(pts[i].z-pts[i+1].z) > eps) {
				ofs << macroname << "("<<pts[i].x << ","<<pts[i].y << ","<<pts[i].z<<","<<pts[i+1].x<<","<<pts[i+1].y<<","<<pts[i+1].z<<")\n";
		}
	}
}

void povDrawLineMacro2(const std::vector<TPoint3> &pts, const int idx, const std::string &macroname, std::ofstream &ofs) {
	const double eps = 0.001;
	for(int i=0;i<(int)pts.size()-1;i++) {
		if (fabs(pts[i].x-pts[i+1].x) > eps ||
			fabs(pts[i].y-pts[i+1].y) > eps ||
			fabs(pts[i].z-pts[i+1].z) > eps) {
				ofs << macroname << "("<<pts[i].x << ","<<pts[i].y << ","<<pts[i].z<<","<<pts[i+1].x<<","<<pts[i+1].y<<","<<pts[i+1].z<<"," << idx << ")\n";
		}
	}
}


void povDeclareConvexHull(const std::vector< std::vector<TPoint3> > &triangles, std::ofstream &ofs, const std::string &name) {
    ofs << "// convex hull\n";
    ofs << "#declare " << name << "= mesh{\n";
    for(int i=0;i<(int)triangles.size();i++) {
        ofs << "   triangle {";
        for(int j=0;j<(int)triangles[i].size();j++) {
            ofs << "<" << triangles[i][j].x << "," << triangles[i][j].y << "," << triangles[i][j].z << ">";
            if (j < 2) {
                ofs << ",";
            }
        }
        ofs << "}\n";
    }
    ofs << "pigment { color rgbf colorHull }\n";
    ofs << "}\n";
}



void draw3Map(CPainter3D *pa, const std::vector< std::vector<TPoint3 > > &tr) {
	if (pa) {
		for(int i=0;i<(int)tr.size();i++) {
			pa->drawTriangle(tgp3(tr[i][0]),tgp3(tr[i][1]),tgp3(tr[i][2]),CColor("royalBlue1"));
		}
	}
}


void draw3Surface(CPainter3D *pa, const std::vector<TPoint3> &pts, const int indicesSize, int *indices) {
	if (pa) {
		int idx = 0;
		for(int i=0;i<indicesSize/3;i++) {
            CColor a,b,c;
            a.setFloatRGB(0.9,0.8,0.66);
            b.setFloatRGB(0.6,0.86,0.3);
            c.setFloatRGB(0.6,0.3,1);
			pa->drawTriangle(tgp3(pts[indices[3*idx+0]]),
					tgp3(pts[indices[3*idx+1]]),
					tgp3(pts[indices[3*idx+2]]),
					a,b,c);
			idx++;
		}
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

void blenderIntro(std::ostream &os) {

    os << "import Blender\n";
    os << "import math\n";
    os << "from Blender import *\n";
    os << "sc = Scene.GetCurrent()\n";
    os << "scale = 1\n";
    os << "rScale = scale\n";
    os << "sphereSegments = 16\n";
    os << "sphereRings = 16\n";
    os << "vertices=6\n";
    os << "radius = 0.1\n\n#-------------------------------------------------\n";
    os << "sc.objects.selected=[]\n";
    os << "scene = Scene.GetCurrent()\n\n";
    os << "radiusTree = 0.1;\n";
    os << "radiusPath = 0.12;\n\n";
    os << "radiusGPath = 0.14;\n\n";
    os << "def create_or_get_material(name,r,g,b):\n";
    os << "    try:\n";
    os << "        mat = Material.Get(name)\n";
    os << "    except:\n";
    os << "        mat = Material.New(name)\n";
    os << "        mat.rgbCol = [r,g,b]\n";
    os << "        mat.setAlpha(1)\n";
    os << "    mat = Material.Get(name)\n";
    os << "    return mat\n\n";

    os << "def getRotAngles(a,b):\n";
    os << "    #a,b, are points\n";
    os << "    rotz = -math.atan2(b[0] - a[0], b[1]-a[1])\n";
    os << "    roty = 0\n";
    os << "    rotx = -math.atan2(b[1]-a[1],b[2]-a[2])\n";
    os << "    return [rotx, roty, rotz]\n\n";

    os << "def makeCylinder(a,b):\n";
    os << "    length = math.sqrt( (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]))\n";
    os << "    me = Mesh.Primitives.Cylinder(vertices, radius, length)\n";
    os << "    ob = scene.objects.new(me,'cyl')\n";
    os << "    r = getRotAngles(a,b);\n";
    os << "    ob.LocX = (a[0]+b[0])/2;	ob.LocY = (a[1]+b[1])/2; ob.LocZ = (a[2]+b[2])/2;\n";
    os << "    ob.RotX = r[0]; 	ob.RotY = r[1]; ob.RotZ = r[2]\n";
    os << "    return ob;\n\n";

    os << "#a,b are points\n";
    os << "def makeSegment(a,b,name,mat,red,green,blue,rad):\n";
    os << "    l = math.sqrt( (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2);\n";
    os << "    me = Mesh.Primitives.UVsphere(8,8,rad)\n";
    os << "    steps = math.ceil(1.8*l/rad);\n";
    os << "    mat = create_or_get_material(mat,red,green,blue);\n";
    os << "    for i in range(0,steps):\n";
    os << "        t = 1.0*i/steps;\n";
    os << "        ob = scene.objects.new(me,name)\n";
    os << "        ob.setLocation((1-t)*a[0]+t*b[0],(1-t)*a[1]+t*b[1],(1-t)*a[2]+t*b[2]);\n";
    os << "        n = ob.getData();\n";
    os << "        n.materials = [ mat ];\n";
    os << "        n.update()\n\n";


	
}

void saveSegmentBlender(std::ofstream &ofs, const TPoint3 &a, const TPoint3 &b, const char *cname, const char *matname,const char *radname) {
    ofs << "makeSegment([" << a.x <<","<<a.y <<","<<a.z <<"],[" <<b.x <<","<<b.y <<","<<b.z << "]," << cname << "," << matname << "," << radname << ")\n";
    /*
    const double l = pointDistanceEucleid(a,b);
    const double dx = 
    const int steps = (int)ceil(l/dx);
    for(int k = 0; k < steps;k++) {
        TPoint3 mpoint;
        const double t = k*1.0/(double)steps;
        mpoint.x = (1-t)*a.x + t*b.x;
        mpoint.y = (1-t)*a.y + t*b.y;
        mpoint.z = (1-t)*a.z + t*b.z;

        ofsr << "me = Mesh.Primitives.UVsphere(16,16,radius)\n";
        ofsr << "ob = scene.objects.new(me,"<<cname << ")\n";
        ofsr << "ob.setLocation(" << mpoint.x << "," << mpoint.y << "," << mpoint.z << ")\n";
        ofsr << "mat = create_or_get_material(" << matname << ",0.1,0.9,0.5)\n";
        ofsr << "n = ob.getData(); ";
        ofsr << "n.materials = [ mat ]; ";
        ofsr << "n.update()\n\n";
    }
    */
}
   
void saveTreeBlender(ofstream &ofsr, const std::vector<TPoint3> &pts, const int offset) {
#if 0
    blenderIntro(ofsr);
    ofsr << "scene = Scene.GetCurrent()\n";
    char cname[30];
    char matname[30];
    snprintf(matname,sizeof(matname),"'mTree%04d'",offset);
    double dx = 0.1;

    int cidx = 0;
    for(int i=0;i<(int)pts.size()-1;i++) {
        snprintf(cname,sizeof(cname),"'s%d:%d'",offset,cidx++);
        saveSegmentBlender(ofsr,pts[i],pts[i+1],cname,matname, radname);
       /*
        ofsr << "ob = makeCylinder([" << pts[i].x << "," << pts[i].y << "," << pts[i].z << "],[" << pts[i+1].x << "," << pts[i+1].y << "," << pts[i+1].z << "])\n";
        ofsr << "mat = create_or_get_material(" << matname << ",0.1,0.9,0.5)\n";
        ofsr << "n = ob.getData()\n";
        ofsr << "n.materials = [ mat ]\n";
        ofsr << "n.update()\n\n";
        */
        /*
        ofsr << "me = Mesh.Primitives.Cylinder(vertices,radius," << pointDistanceEucleid(pts[i],pts[i+1]) << ")\n";
        ofsr << "ob = scene.objects.new(me,"<<cname << ")\n";
        ofsr << "ob.setLocation(" << mpoint.x << "," << mpoint.y << "," << mpoint.z << ")\n";
        ofsr << "ob.setEuler(" << rx <<"," << ry << "," << rz << ")\n";
        */
        
    }
#endif 
}

void povDraw3DNodesColor( const std::vector< TNode *> &tree,  CRobot3D *robot, std::ofstream &ofs, const char *colorName) {

    std::vector< TPoint3 > pts;
    int idx = 0;
    for(int i=0;i<(int)tree.size();i++) {
        if (tree[i]->pstate.size() != 0) {
            pts.push_back(robot->getRefPoint(tree[i]->pstate));
            pts.push_back(robot->getRefPoint(tree[i]->state));
            povDrawTreeSegments(pts,colorName,ofs,true);
            pts.clear();
        }
        idx++;
    }
}






} // namespace rrtPlanning

