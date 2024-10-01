
#include "CPainterBase.h"
#include <cstring>
#include <iostream>
#include <math.h>


namespace CPainters {

using namespace std;

CPainterBase::CPainterBase(const char *_mask, CPainterBase *_dump) {
    if (_mask) {
    	strncpy(mask,_mask,sizeof(mask));
    }
	figOpen = false;
	imNumber = 0;
	dump = _dump;
	doDump = dump==NULL?false:true;	
	enabled = true;
}

CPainterBase::~CPainterBase() {
	if (figOpen) {
		of.close();
	}
}

void CPainterBase::setMask(const char *_mask) {
    strncpy(mask,_mask,sizeof(mask));
}

void CPainterBase::begin() {

}

void CPainterBase::close() {

}

void CPainterBase::draw(const gPoint &p,
		const int size, const int lsize,
		const CColor &lColor,
		const CColor &fColor)
{
}

int CPainterBase::getActualImage() const {
	return imNumber;
}	

void CPainterBase::setActualImage(const int i) {
	imNumber = i;
}

/**
  * draw curve
  */
void CPainterBase::draw(const vector<gPoint> &pointList,
		const int lsize,
		const CColor &lColor) 
{	
}

/**
  * draw given points
  */
void CPainterBase::draw(const vector<gPoint> &pointList,
		const int pSize,
		const int lSize,
		const CColor &lColor,
		const CColor &fColor) {

	if (enabled) {
        for(int i=0;i<(int)pointList.size();i++) {
			draw(pointList[i],pSize,lSize,lColor,fColor);
		}
	}
}

// filled curve
void CPainterBase::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor, const CColor &fColor) 
{
}

void CPainterBase::draw(const char *text, const gPoint &p, const int size){
}

		
int CPainterBase::getWidth() {
	return 0;
}

int CPainterBase::getHeight() {
	return 0;
}
		

void CPainterBase::setDump(const bool dump) {
		doDump = true;
}

bool CPainterBase::isDumpOn() const {
	return doDump;
}

CPainterBase *CPainterBase::getDumpPainter() const {
	return dump;
}

void CPainterBase::enable() {
	enabled = true;
}

void CPainterBase::disable() {
	enabled = false;
}

bool CPainterBase::isEnabled() const {
	return enabled;
}

void CPainterBase::setParam(const std::string &paramName, const std::string &paramValue) {
	
}



} // namespace

