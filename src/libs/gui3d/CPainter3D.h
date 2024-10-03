#ifndef CPAINTER3D_BASE_H
#define CPAINTER3D_BASE_H

#include <vector>
#include "../gui/CPainterBase.h"

/* base class for all 3D painters: gui and povray */

namespace Painter3D {

class gPoint3 {
	public:
	gPoint3(const double xx = 0, const double yy=0, const double zz=0) :
		x(xx),y(yy),z(zz) {}
	gPoint3(const gPoint3 &rhs): x(rhs.x), y(rhs.y), z(rhs.z) {}

	double x,y,z;
};

class CPainter3D {

	public:
	CPainter3D();
	virtual ~CPainter3D();

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
};

// rotate point according to rotation described by quaternion 	
void rotateQuaternion(gPoint3 &p, const double w, const double x, const double y, const double z);

// make triangles from given box. Box is centered in poiint center. Rotation is given
// as quaternion (w x y z)	
std::vector< std::vector<gPoint3> > makeTriangleFromBox(const gPoint3 &center, 
		const double lx, const double ly, const double lz, const double qw,
		const double qx, const double qy, const double qz);


} // namespace Painter3D


#endif

