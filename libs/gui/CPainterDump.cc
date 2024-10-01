
#include "CPainterDump.h"
#include <cstring>
#include <list>

namespace CPainters {

using std::list;
using std::ofstream;
using std::vector;

typedef enum _TObjectType {
	point=0,
	points,
	polyline,
	polygon,
	_text,
	_begin,
	_close,
	param
} gObjectType;

const char *gObjectTypeStr[] = {
	"p",
	"s",
	"l",
	"g",
	"t",
	"b",
	"c",
	"q" // param
};

void CPainterDump::ppoint(ofstream &ofs, const gPoint &p) const {
	ofs << p.x() << ' ' << p.y();
	if (p.r) {
		ofs << '1';
	}
}


void CPainterDump::pcolor(ofstream &ofs, const CColor &c) const {
	char str[20];
	sprintf(str,"%d",(int)c.getRed());
	ofs << str << ' ';
	sprintf(str,"%d",(int)c.getGreen());
	ofs << str << ' ';
	sprintf(str,"%d",(int)c.getBlue());
	ofs << str << ' ';
	sprintf(str,"%d",(int)c.getA());
	ofs << str;
}

void CPainterDump::ppoints(ofstream &ofs, const vector<gPoint> &pts) const {
    for(int i=0;i<(int)pts.size();i++) {
		ppoint(ofs,pts[i]);
		ofs << '\n';
	}
}


CPainterDump::CPainterDump(const char *_mask, CPainterBase *_dump): CPainterBase(_mask,_dump) {

	of.clear();
	of.open(_mask);
	figOpen = false;
}

 CPainterDump::~CPainterDump() {
	if (figOpen) {
		close();
	}
	if (of.is_open()) {
		of << "#end of dump\n";
	}
	of.close();
	dump = NULL;
}

void CPainterDump::begin() {

	if (doDump) {
		dump->begin();
	}

	if (figOpen) {
		close();
	}
	
	gObjectType o = _begin;
	if (of.is_open()) {
		of << gObjectTypeStr[o] << '\n';
	}
	figOpen = true;
}

void CPainterDump::close() {

	if (doDump) {
		dump->close();
	}

	gObjectType o = _close;
	if (figOpen) {
		if (of.is_open()) {
			of << gObjectTypeStr[o] << '\n';
		}
		figOpen = false;
	}
	of << std::flush;
}

void CPainterDump::draw(const gPoint &p, const int size, const int lsize, const CColor &lColor,
		const CColor &fColor)
{
	if (doDump) {
		dump->draw(p,size,lsize,lColor,fColor);
	}

	gObjectType o = point;
	of << gObjectTypeStr[o] << ' ' << size << ' ' << lsize << ' ';
	pcolor(of,lColor);
	of << ' ';
	pcolor(of,fColor);
	of << ' ';
	ppoint(of,p);
	of << '\n';

}


/**
  * draw curve
  */
void CPainterDump::draw(const vector<gPoint> &pointList, const int lsize,const CColor &lColor) {	
	if (doDump) {
		dump->draw(pointList,lsize,lColor);
	}

	gObjectType o = polyline;
	of << gObjectTypeStr[o] << ' ' << lsize << ' ' << pointList.size() << ' ';
	pcolor(of,lColor);
	of << '\n';
	ppoints(of,pointList);
}


// filled curve
void CPainterDump::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor, const CColor &fColor) 
{
	if (doDump) {
		dump->draw(pointList,lsize,lColor,fColor);
	}


	gObjectType o = polygon;
	of << gObjectTypeStr[o] << ' ' << lsize << ' ' << pointList.size() << ' ';
	pcolor(of,lColor);
	of << ' ';
	pcolor(of,fColor);
	of << '\n';
	ppoints(of,pointList);
}

void CPainterDump::draw(const char *text, const gPoint &p, const int size){

	if (doDump) {
		dump->draw(text,p,size);
	}

	gObjectType o = _text;
	of << gObjectTypeStr[o] << ' ' << size << ' ';
	ppoint(of,p);
	of << '\n' << text << '\n';
}

		
int CPainterDump::getWidth() {
	return 0;
}

int CPainterDump::getHeight() {
	return 0;
}

void CPainterDump::setParam(const std::string &paramName, const std::string &paramValue) {
		
	gObjectType o = param;
	of << gObjectTypeStr[o] << ' ' << paramName << ' ' << paramValue << '\n';
}



} // namespace



