#include "CPainter3DPovray.h"
#include <sstream>
#include <string>
#include <string.h>


namespace Painter3D {

typedef CPainters::CColor CColor;


std::string printCoord(const gPoint3 &p);
std::string printCoord(const double x, const double y, const double z);
std::string printColor(const CColor &col);

CPainter3DPovray::CPainter3DPovray(const char *filemask):CPainter3D() {

	index = 0;
	strncpy(mask,filemask,500);
//	std::cerr << "mask is " << mask << "\n";
	sprintf(camName,"%s.camera.inc",mask);
	ofs.clear();

	ofs.open(camName);
	include("colors.inc");
	include("textures.inc");
	ofs << "camera { location " << printCoord(-20,-5,15) << " sky " << printCoord(0,0,1) << " ";
	ofs << "look_at " << printCoord(0,0,0) << "}\n";
	ofs << "light_source { " << printCoord(12,12,7) << " color White}\n";
	ofs << "light_source { " << printCoord(1,12,7) << " color White}\n";
	ofs << "light_source { " << printCoord(-5,-12,7) << " color White}\n";
	ofs << "light_source { " << printCoord(-50,50,70) << " color White}\n";
	ofs << "light_source { " << printCoord(0,-50,70) << " color White}\n";
	ofs << "light_source { " << printCoord(0,52,70) << " color White}\n";
	ofs << "light_source { " << printCoord(0,0,40) << " color White}\n";
	ofs << "#declare trajectoryWidth= 0.03;\n";
	ofs << "#declare rrtWidth = 0.02;\n";

	ofs << "#declare arsLength = 2;\n";
	ofs << "#declare ars = union { \n";
	ofs << "cylinder { <0,0,0>, <arsLength,0,0>, 0.1 pigment { color rgb <1,0,0> } }\n";
	ofs << "cylinder { <0,0,0>, <0,arsLength,0>, 0.1 pigment { color rgb <0,1,0> } }\n";
	ofs << "cylinder { <0,0,0>, <0,0,arsLength>, 0.1 pigment { color rgb <0,0,1> } }\n";
	ofs << "}\n\n";
	ofs << "// object { ars }\n\n";
	ofs.close();
	ofs.clear();
}

CPainter3DPovray::~CPainter3DPovray() {
	ofs.close();
	ofs.clear();
}

void CPainter3DPovray::begin() {

	char name[200];
	sprintf(name,"%s.%06d.pov",mask,index++);
	ofs.open(name);

	include(camName);

}

void CPainter3DPovray::close() {

	ofs.close();
	ofs.clear();

}

//void CPainter3DPovray::drawBox(const gPoint3 &position, const double rx, const double ry, const double rz,
//		const double size, const CColor &color) {
//
//}

void CPainter3DPovray::drawPoint(const gPoint3 &position, const double size, const CColor &color){

	ofs << "sphere {" << printCoord(position) <<"," << size << " pigment { " << printColor(color) << "}}\n";

}

void CPainter3DPovray::drawLine(const gPoint3 &p1, const gPoint3 &p2, const double width, const CColor &color) {
	ofs << " cylinder { " << printCoord(p1) << "," << printCoord(p2) << "," << width << " ";
	ofs << "pigment { " << printColor(color) << "}}\n";
}

void CPainter3DPovray::drawLine(const std::vector<gPoint3> &pts, const double width, const CColor &color) {
	for(int i=0;i<(int)pts.size()-1;i++) {
		drawLine(pts[i],pts[i+1],width,color);
	}
}

void CPainter3DPovray::drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CColor &color) {
	ofs << "triangle {" << printCoord(p1) << "," << printCoord(p2) << "," << printCoord(p3) << " ";
	ofs << "pigment { " << printColor(color) << "}}\n";
}

void CPainter3DPovray::drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &color) {
	for(int i=0;i<(int)triangles.size();i++) {
		drawTriangle(triangles[i][0],triangles[i][1],triangles[i][2],color);
	}
}

	
void CPainter3DPovray::drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3) {
	drawTriangle(p1,p2,p3,c1);
}

void CPainter3DPovray::drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3) {
	drawTriangles(triangles,c1);
}

int CPainter3DPovray::getType() {
	return 2;
}

void CPainter3DPovray::include(const char *file) {
	ofs << "#include \"" << file << "\"\n";
}

std::string printCoord(const gPoint3 &p) {
	return printCoord(p.x,p.y,p.z);
}

std::string printCoord(const double x, const double y, const double z) {
	std::stringstream ss;
	ss << "<" << x << "," << y << "," << z << ">";
	return ss.str();
}

std::string printColor(const CColor &col) {
	std::stringstream ss;
	ss << "color rgb";
	if (col.getA() == 255) {
		ss << " <" << (double)col.getRed()/255.0 << "," << (double)col.getGreen()/255.0 << ","<<
			(double)col.getBlue()/255.0 << "> ";
	} else {
		ss << "f <" << (double)col.getRed()/255.0 << "," << (double)col.getGreen()/255.0 << ","<<
			(double)col.getBlue()/255.0 << "," << (double)(255.0-col.getA())/255.0 << "> ";
	}
	return ss.str();
}

} // namespace Painter3D
