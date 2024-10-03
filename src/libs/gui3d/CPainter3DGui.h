#ifndef CPAINTER3D_GUI_H
#define CPAINTER3D_GUI_H

#include <vector>
#include "CPainter3D.h"
#include <semaphore.h>
/* base class for all 3D painters: gui and povray */

namespace Painter3D {

class CPainter3DGui : public CPainter3D {

	public:
	CPainter3DGui();
	virtual ~CPainter3DGui();

	virtual void drawBox(const gPoint3 &position, const double rx, const double ry, const double rz,
		   const double size, const CPainters::CColor &color);

	virtual void drawPoint(const gPoint3 &position, const double size, const CPainters::CColor &color);

	virtual void drawLine(const gPoint3 &p1, const gPoint3 &p2, const double width, const CPainters::CColor &color);

	virtual void drawLine(const std::vector<gPoint3> &pts, const double width, const CPainters::CColor &color);

	virtual void drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &color);		

	virtual void drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &color);
	virtual void drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3);

	virtual void drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3);

	virtual int getType();

	virtual void begin();
	virtual void close();
	bool isNewData();
	
	class NTriangle {
		public:
			NTriangle(const gPoint3 &ip1, const gPoint3 &ip2, const gPoint3 &ip3, const CPainters::CColor &icolor): 
				p1(ip1),p2(ip2),p3(ip3),color(icolor) {}
			NTriangle(const NTriangle &rhs): 
				p1(rhs.p1),p2(rhs.p2),p3(rhs.p3),color(rhs.color) {}
			gPoint3 p1,p2,p3;
			CPainters::CColor color;
	};

	class NTriangleColored  {
		public:
			NTriangleColored(const gPoint3 &ip1, const gPoint3 &ip2, const gPoint3 &ip3, 
					const CPainters::CColor &ic1, const CPainters::CColor &ic2, const CPainters::CColor &ic3):
				p1(ip1),p2(ip2),p3(ip3),c1(ic1),c2(ic2),c3(ic3) {}
			NTriangleColored(const NTriangleColored &rhs):
				p1(rhs.p1), p2(rhs.p2), p3(rhs.p3), c1(rhs.c1),c2(rhs.c2),c3(rhs.c3) {}

			gPoint3 p1,p2,p3;
			CPainters::CColor c1,c2,c3;

	};

	class NPoint {
		public:
			NPoint(const gPoint3 &ip, const double isize, const CPainters::CColor &icolor):
				p(ip),size(isize),color(icolor) {}
			NPoint(const NPoint &rhs):
				p(rhs.p),size(rhs.size),color(rhs.color) {}
			gPoint3 p;
			double size;
			CPainters::CColor color;
	};

	class NLine {
		public:
			NLine(const gPoint3 &ip1, const gPoint3 &ip2, const double iwidth, const CPainters::CColor &icolor):
				p1(ip1),p2(ip2),width(iwidth),color(icolor) {}
			NLine(const NLine &rhs):
				p1(rhs.p1),p2(rhs.p2),width(rhs.width),color(rhs.color) {}
			gPoint3 p1,p2;
			double width;
			CPainters::CColor color;
	};

	void getNewTriangles(std::vector<NTriangle> &triangles);
	void getNewTrianglesColored(std::vector<NTriangleColored> &triangles);
	void getNewPoints(std::vector<NPoint> &points);
	void getNewLines(std::vector<NLine> &lines);

	private:
	sem_t semNewData, semNewPoints, semNewLines, semNewTriangles, semIsNewData, semNewTrianglesColored;
	bool isNewDataF;

	std::vector<NTriangle> newTriangles;
	std::vector<NTriangleColored> newTrianglesColored;
	std::vector<NPoint> newPoints;
	std::vector<NLine> newLines;

	pthread_t thread;

};


} // namespace Painter3D


#endif

