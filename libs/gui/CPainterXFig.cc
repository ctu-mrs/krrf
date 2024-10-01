
#include "CPainterXFig.h"

#include <stdio.h>
#include <math.h>
#include <vector>
#include <list>
#include <string>
#include <fstream>
#include <sstream>

namespace CPainters {

using std::vector;
using std::list;

const int CPainterXFig::fontSize = 15;

/**
  * dimension is array of: {minx, maxx, miny, maxy}
  * so width of a image is (maxx-minx) and it's height
  * is (maxy - miny)
  */
CPainterXFig::CPainterXFig(	const char *mask, const vector<double> &dimension, CPainterBase *_dump):CPainterBase(mask,_dump)
{
	width = (int)lround(dimension[1]-dimension[0]);
	height = (int)lround(dimension[3]-dimension[2]);
	sx = 1;
	sy = 1;
	depth = 50;
	/*
	if (width > 14031) { // bylo 500
		sx = 14031.0 / (double)width;
		sy = sx;
		width = 14031;
		height = lroundl(sy*height);
	}
	if (height > 9920.0) {
		sy = 9920.0/(double)height;
		sx = sy;
		height = 9920.0;
		width = lroundl(sx*width);
	}
	*/
//	std::cerr << "w,h = " << width << " " << height << "\n";	
   	int maxW, maxH;	
	if (width > height) {
		sx = 14031.0 / (double)width;
		sy = sx;
		width = 14031;
		height = lroundl(sy*height);
		paperType = 0;
		maxW = 14031;
		maxH = 9920;
//		std::cerr << "set w,h = " << width << " " << height << "\n";
//		std::cerr << "sx,sy = " << sx << " " << sy << "\n";
		/*
		if (height > maxH) {
			height = (int)lround(dimension[3]-dimension[2]);
			sy = 1.0*maxH/(double)height;
			sx = sy;
			height = maxH;
			width = lroundl(sx*width);
			std::cerr << "Correction to " << width << " " << height << "\n";
			std::cerr << "sx,sy = " << sx << " " << sy << "\n";
		}
		*/
	} else {
		paperType = 1;
		sy = 14031.0/(double)height;
		sx = sy;
		height = 14031;
		width = lroundl(sx*width);
		maxW = 9920;
		maxH = 14031;
/*
		sy = 9920.0/(double)height;
		sx = sy;
		height = 9920.0;
		width = lroundl(sx*width);
*/
		/*
		std::cerr << "*set w,h = " << width << " " << height << "\n";
		std::cerr << "sx,sy = " << sx << " " << sy << "\n";
		if (width > maxW) {
			width = (int)lround(dimension[1]-dimension[0]);
			sx = 1.0*maxW/(double)width;
			sy = sx;
			width = maxW;
			height = lroundl(sy*height);
			std::cerr << "Correction to " << width << " " << height << "\n";
			std::cerr << "sx,sy = " << sx << " " << sy << "\n";
		}
		*/
	}
	dx = dimension[0];
	dy = dimension[2];
	figOpen = false;
	imNumber = 0;
}

CPainterXFig::~CPainterXFig() {
	if (figOpen) {
		close();
	}
	dump = NULL;
}

void CPainterXFig::begin() {
	if (doDump) {
		dump->begin();
	}

	if (figOpen) {
		close();
	}
	char name[2000];
	snprintf(name,sizeof(name),"%s_%06d.fig",mask,-1);
	oft.clear();
	oft.open(name);

	figOpen = true;
}

void CPainterXFig::close() {
	if (doDump) {
		dump->close();
	}

	if (!figOpen) {
		return;
	}

	oft.close();
	
	char name[2000];
	snprintf(name,sizeof(name),"%s_%06d.fig",mask,imNumber);
	
	of.clear();
	of.open(name);
	of << "#FIG 3.2 Produced by CPAinterXFig\n";
	if (paperType == 0) {
	  	of << "Landscape\n";
	} else {
		of << "Portrait\n";
	}	
	of << "Center\n";
	of << "Inches\n";
//	of << "Letter\n";
	of << "A4\n";
	of << "100.0\n";
	of << "Single\n";
	of << "-2\n";
	of << "1200 2\n";	

	for(int i=0;i<(int)definedColors.size();i++) {
		of << "0 " << (i+33) << "#";
		sprintf(name,"%02X%02X%02X",definedColors[i].getRed() & 0xFF, definedColors[i].getGreen() & 0xFF, definedColors[i].getBlue() & 0xFF);
		of << name << "\n";
	}

	snprintf(name,sizeof(name),"%s_%06d.fig",mask,-1);
	std::ifstream ifs(name);
	std::string line;
	while(ifs) {
		std::getline(ifs,line);
		of << line << "\n";
	}
	ifs.close();
	of.close();

	remove(name);

	figOpen = false;
	imNumber++;
	definedColors.clear();

}

/**
  * draw single point
  */
void CPainterXFig::draw(const gPoint &p,
		const int size, const int lsize, const CColor &lColor, const CColor &fColor) {
	
	if (doDump) {
		dump->draw(p,size,lsize,lColor,fColor);
	}

	if (!figOpen || !enabled || size==0) {
		return;
	}

	const int scale = 1;
	int x,y;
	togPoint(p,x,y);
	int pc = setColor(lColor);
	int fc = setColor(fColor);
	int a = size*scale;
	int b = size*scale;
	int c = x;
	int d = y;
	oft << "1 3 0 " << lsize << " " << pc << " " << fc << " ";
	oft << " " << depth << " -1 20 0.000 1 0.000 " << x << " " << y << " " <<
		a << " " << b << " " << x << " " << y << " " << c << " " << d << "\n";
}

inline void CPainterXFig::togPoint(const gPoint &p, int &x, int &y) const 
{
	x = (int)lround((p.x()-dx)  * sx);
	y = (int)lround((p.y()-dy)  * sy);
	y = (int)lround(height-y);
	x*=1;
	y*=1;
}

inline int CPainterXFig::setColor(const CColor & c) {
	int r,g,b;
	r = c.getRed();
	g = c.getGreen();
	b = c.getBlue();
	int cn = -1;
	for(int i=0;i<(int)definedColors.size() && cn == -1;i++) {
		if (definedColors[i].getRed() == r &&
			definedColors[i].getGreen() == g &&
			definedColors[i].getBlue() == b)  {
			cn = i;
		}		
	}
	if (cn == -1) {
		definedColors.push_back(c);
		cn = definedColors.size()-1;
	}

	cn += 33;
	return cn;
}


/**
  * draw curve from given points. does not write point itself
  */
void CPainterXFig::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor) {	

	if (doDump) {
		dump->draw(pointList,lsize,lColor);
	}

	if (!figOpen || !enabled || pointList.size() == 0) {
		return;
	}
	int pc = setColor(lColor);
	int fc = 0;
	oft << "2 1 0 " << lsize << " " << pc << " " << fc << " " << depth << " -1 -1 0.000 0 0 -1 0 0 "
	   << (pointList.size()) << "\n";

	int x,y;	
    for(int i=0;i<(int)pointList.size();i++) {
		togPoint(pointList[i],x,y);
		oft << x << ' ' << y <<  ' ';
	}
	oft << "\n";

}

/**
  * draw filled curve.
  */
void CPainterXFig::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor, const CColor &fColor) {

	if (doDump) {
		dump->draw(pointList,lsize,lColor,fColor);
	}

	if (!figOpen || !enabled || pointList.size() == 0) {
		return;
	}

	int pc = setColor(lColor);
	int fc = setColor(fColor);

	oft << "2 3 0 " << lsize << " " << pc << " " << fc << " " << depth << " -1 20 0.000 0 0 7 0 0 " <<
		(pointList.size()+1) << "\n";

	int x,y;
    for(int i=0;i<(int)pointList.size();i++) {
		togPoint(pointList[i],x,y);
		oft << x << ' ' << y << ' ';
	}
	togPoint(pointList.front(),x,y);
	oft << x << ' ' << y << '\n';
}



void CPainterXFig::draw(const char *text, const gPoint &p, const int size){

	if (doDump) {
		dump->draw(text,p,size);
	}	
	
	if (!figOpen || !enabled) {
		return;
	}
	int x,y;
	togPoint(p,x,y);
	int pc= setColor(CColor(0,0,0));
	int font = 0;
	oft << "4 1 " << pc << " " << depth << " -1 " << font << " " << size << " 0.00 0 4 120 " <<
		x << " " << y << " " << text << "\\001" << "\n";

}

int CPainterXFig::getWidth() {
	return width;
}

int CPainterXFig::getHeight() {
	return height;
}

void CPainterXFig::setParam(const std::string &paramName, const std::string &paramValue) {


	if (doDump) {
		dump->setParam(paramName, paramValue);
	}

	std::istringstream iss(paramValue);
	int ival;
	if (paramName.compare("depth") == 0) {
		if (iss >> ival) {
			if (ival >= 1 && ival <=999) {
				depth = ival;
			}
		}
	} else if (paramName.compare("groupb")==0) {
		double minx,maxx,miny,maxy;
		if (iss >> minx >> maxx >> miny >> maxy) {
			oft << "6 " << minx << " " << maxy << " " << maxx << " " << miny << "\n";			
		}
	} else if (paramName.compare("groupe") == 0) {
			oft << "-6\n";	
	}

}

} 


