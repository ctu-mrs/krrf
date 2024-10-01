
#include "CPainterGnuplot.h"

#include <iostream>
#include <cairo.h>
#include <cairo-pdf.h>
#include <math.h>
#include <list>
#include <sstream>
#include <string>

namespace CPainters {

using namespace std;

CPainterGnuplot::CPainterGnuplot(CPainterBase *_dump)
    :CPainterBase("",_dump)
{
    _g = new Gnuplot();
    _g->showonscreen();
    _first = true;
    _pts.clear(); 
}

CPainterGnuplot::~CPainterGnuplot() {
    delete _g;
    _pts.clear();
}

void CPainterGnuplot::begin() {
	if (doDump) {
		dump->begin();
	}
    _pts.clear();
}

void CPainterGnuplot::close() {


	if (doDump) {
		dump->close();
	}

    _g->cmd(string("\n"));
//    for(list< list<gPoint> >::const_iterator i = _pts.begin(); i != _pts.end(); i++) {
//        for(list<gPoint>::const_iterator j = i->begin(); j != i->end(); j++) {
    for(int i=0; i < (int)_pts.size(); i++) {
        for(int j=0;j<(int)_pts[i].size();j++) {
            stringstream ss;
            ss << " " << _pts[i][j].x() << " " << _pts[i][j].y() << "\n";
            _g->cmd(ss.str());
            ss.clear();
        }
        _g->cmd(string("e\n"));
    }
    _g->cmd(string("\n"));
//    *_g << Gnuplot::endl;
    _first = true;
}

/**
  * draw single point
  */
void CPainterGnuplot::draw(const gPoint &p, const int size, const int lsize, const CColor &lColor, const CColor &fColor) {

	if (doDump) {
		dump->draw(p,size,lsize, lColor, fColor);
	}
    if (_first) {
        _g->cmd(string("plot "));
        _first = false;
    } else {
        _g->cmd(string(", "));
    }

    stringstream ss;
    ss << "'-' w p lc rgbcolor \"" << lColor.getRGBColor() << "\" notitle ";
    _g->cmd(ss.str());
    ss.clear();


    vector<gPoint> l;
    l.reserve(1);
    l.push_back(p);
    _pts.push_back(l);

}

/**
  * return integer representation of a given color
  * see XFig format
  */
void CPainterGnuplot::setColor(const CColor & c) {
	int r,g,b,a;
	r = c.getRed();
	g = c.getGreen();
	b = c.getBlue();
	a = c.getA();
}


/**
  * draw curve from given points. does not write point itself
  */
void CPainterGnuplot::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor) {	

	if (doDump) {
		dump->draw(pointList,lsize,lColor);
	}

    if (_first) {
        _g->cmd(string("plot "));
        _first = false;
    } else {
        _g->cmd(string(", "));
    }

    stringstream ss;
    ss << " '-' w l lw " << lsize << " lc rgbcolor \"" << lColor.getRGBColor() << "\" notitle ";
    _g->cmd(ss.str());

    ss.clear();

    _pts.push_back(pointList);

}

/**
  * draw filled curve.
  */
void CPainterGnuplot::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor, const CColor &fColor) {

	if (doDump) {
		dump->draw(pointList,lsize,lColor,fColor);
	}

    if (_first) {
        _g->cmd(string("plot "));
        _first = false;
    } else {
        _g->cmd(string(", "));
    }

    stringstream ss;
    ss << " '-' w filledcurve lw " << lsize << " lc rgbcolor \"" << fColor.getRGBColor() << "\" notitle ";
    _g->cmd(ss.str());
    ss.clear();

    _pts.push_back(pointList);

    draw(pointList,lsize,lColor);


}



void CPainterGnuplot::draw(const char *text, const gPoint &p, const int size){

	if (doDump) {
		dump->draw(text,p,size);
	}	
	
}

int CPainterGnuplot::getWidth() {
	return 0;
}

int CPainterGnuplot::getHeight() {
	return 0;
}


}


