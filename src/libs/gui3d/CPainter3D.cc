#include "CPainter3D.h"

namespace Painter3D {

typedef CPainters::CColor CColor;

CPainter3D::CPainter3D() {
}

CPainter3D::~CPainter3D() {
}

void CPainter3D::begin() {

}

void CPainter3D::close() {

}

//void CPainter3D::drawBox(const gPoint3 &position, const double rx, const double ry, const double rz,
//		const double size, const CColor &color) {
//
//}

void CPainter3D::drawPoint(const gPoint3 &position, const double size, const CColor &color){

}

void CPainter3D::drawLine(const gPoint3 &p1, const gPoint3 &p2, const double width, const CColor &color) {

}

void CPainter3D::drawLine(const std::vector<gPoint3> &pts, const double width, const CColor &color) {

}

void CPainter3D::drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CColor &color) {

}

void CPainter3D::drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &color) {

}

	
void CPainter3D::drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3) {

}

void CPainter3D::drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3) {

}

void rotateQuaternion(gPoint3 &p, const double w, const double x, const double y, const double z) {
	double t1,t2,t3,t4,t5,t6,t7,t8,t9,t10;
	const double a = w;
	const double b = x;
	const double c = y;
	const double d = z;
	t2 =   a*b;
	t3 =   a*c;
	t4 =   a*d;
	t5 =  -b*b;
	t6 =   b*c;
	t7 =   b*d;
	t8 =  -c*c;
	t9 =   c*d;
	t10 = -d*d;

	const double v1 = p.x;
	const double v2 = p.y;
	const double v3 = p.z;

	const double xx = 2*( (t8 + t10)*v1 + (t6 -  t4)*v2 + (t3 + t7)*v3 ) + v1;
	const double yy = 2*( (t4 +  t6)*v1 + (t5 + t10)*v2 + (t9 - t2)*v3 ) + v2;
	const double zz = 2*( (t7 -  t3)*v1 + (t2 +  t9)*v2 + (t5 + t8)*v3 ) + v3;
	p.x = xx;
	p.y = yy;
	p.z = zz;

}

std::vector< std::vector<gPoint3> > makeTriangleFromBox(
		    const gPoint3 &center, 
			const double lx, const double ly, const double lz, const double qw,
			const double qx, const double qy, const double qz) {

//	dVector3 res;
//	dGeomBoxGetLengths(obj, res);
//	const dReal *rp = dGeomGetPosition(obj);
//	dQuaternion quat;
//	dGeomGetQuaternion(obj,quat);

	gPoint3 a(-lx/2.0,-ly/2.0,-lz/2.0);
	gPoint3 b(lx/2.0,-ly/2.0,-lz/2.0);
	gPoint3 c(lx/2.0,ly/2.0,-lz/2.0);
	gPoint3 d(-lx/2.0,ly/2.0,-lz/2.0);

	gPoint3 e(-lx/2.0,-ly/2.0,lz/2.0);
	gPoint3 f(lx/2.0,-ly/2.0,lz/2.0);
	gPoint3 g(lx/2.0,ly/2.0,lz/2.0);
	gPoint3 h(-lx/2.0,ly/2.0,lz/2.0);

	gPoint3 p1,p2,p3;
	std::vector< std::vector<gPoint3> > triangles;
	std::vector< gPoint3 > triangle;

	triangle.push_back(a); triangle.push_back(b); triangle.push_back(c);
	triangles.push_back(triangle); triangle.clear();
	triangle.push_back(a); triangle.push_back(c); triangle.push_back(d);
	triangles.push_back(triangle); triangle.clear();

	triangle.push_back(a); triangle.push_back(b); triangle.push_back(f);
	triangles.push_back(triangle); triangle.clear();
	triangle.push_back(a); triangle.push_back(f); triangle.push_back(e);
	triangles.push_back(triangle); triangle.clear();

	triangle.push_back(b); triangle.push_back(c); triangle.push_back(g);
	triangles.push_back(triangle); triangle.clear();
	triangle.push_back(b); triangle.push_back(g); triangle.push_back(f);
	triangles.push_back(triangle); triangle.clear();
	
	triangle.push_back(d); triangle.push_back(c); triangle.push_back(g);
	triangles.push_back(triangle); triangle.clear();
	triangle.push_back(d); triangle.push_back(g); triangle.push_back(h);
	triangles.push_back(triangle); triangle.clear();
	
	triangle.push_back(a); triangle.push_back(d); triangle.push_back(h);
	triangles.push_back(triangle); triangle.clear();
	triangle.push_back(a); triangle.push_back(h); triangle.push_back(e);
	triangles.push_back(triangle); triangle.clear();
	
	triangle.push_back(e); triangle.push_back(f); triangle.push_back(g);
	triangles.push_back(triangle); triangle.clear();
	triangle.push_back(e); triangle.push_back(g); triangle.push_back(h);
	triangles.push_back(triangle); triangle.clear();

	for(int i=0;i<(int)triangles.size();i++) {
		for(int j=0;j<(int)triangles[i].size();j++) {
			rotateQuaternion(triangles[i][j],qw,qx,qy,qz);
			triangles[i][j].x += center.x;
			triangles[i][j].y += center.y;
			triangles[i][j].z += center.z;
		}
	}
	return triangles;

}

int CPainter3D::getType() {
	return 0;
}


} // namespace Painter3D
