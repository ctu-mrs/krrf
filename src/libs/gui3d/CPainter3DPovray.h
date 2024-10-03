#ifndef CPAINTER3D_POVRAY_H
#define CPAINTER3D_POVRAY_H

#include <vector>
#include "../gui/CPainterBase.h"
#include "CPainter3D.h"
#include <string>
#include <fstream>

/* base class for all 3D painters: gui and povray */

namespace Painter3D {

class CPainter3DPovray : public CPainter3D {

	public:
	CPainter3DPovray(const char *filemask);
	virtual ~CPainter3DPovray();

	virtual void begin();
	virtual void close();

	virtual void drawPoint(const gPoint3 &position, const double size, const CPainters::CColor &color);

	virtual void drawLine(const gPoint3 &p1, const gPoint3 &p2, const double width, const CPainters::CColor &color);

	virtual void drawLine(const std::vector<gPoint3> &pts, const double width, const CPainters::CColor &color);

	virtual void drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &color);		
	virtual void drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3);		

	virtual void drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &color);

	virtual void drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3);



	// return type of painter
	virtual int getType();

	std::ofstream ofs;


	private:
	char mask[500];
	char camName[500];
	int index;

	void include(const char *file);

};

} // namespace Painter3D


#endif

