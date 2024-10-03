#ifndef _CPAINTERBASE_H__
#define _CPAINTERBASE_H__

#include <fstream>
#include <vector>
#include <string>
#include "CPainterColor.h"

namespace CPainters {

void saveColorsForTex(const char *filename);

class gPoint {
	public:
		gPoint(const double x=0, const double y=0, const double resizeable = false): xx(x),yy(y),r(resizeable){}
		gPoint(const gPoint &gp):xx(gp.xx),yy(gp.yy),r(gp.r){}
		
		double x() const { return xx; }
		double y() const { return yy; }

		
		double xx,yy;
		bool r;
};



class CPainterBase {

	public:
		CPainterBase(const char *_mask, CPainterBase *_dump = NULL);

		virtual ~CPainterBase();

        void setMask(const char *_mask);

        std::string getLastFilename() const {
            return _lastFileName;
        }


		virtual void begin();
		virtual void close();

		virtual bool isGui() const { return false; }

		virtual void draw(const gPoint &p, 
				const int size,
				const int lsize,
				const CColor &lColor,
				const CColor &fColor);


		virtual void draw(const std::vector<gPoint> &pointList,
			const int lsize,
			const CColor &lColor);

		virtual void draw(const std::vector<gPoint> &pointList,
					const int lsize,
					const CColor &lColor,
					const CColor &fColor);

		// if size == -1 .. then default size for each derived class is used
		virtual void draw(const char *text, const gPoint &p,
				const int size = -1);

		void draw(const std::vector<gPoint> &pointList,
				const int pSize,
				const int lSize,
				const CColor &lColor,
				const CColor &fColor);

		int getActualImage() const;
		void setActualImage(const int i);

		virtual int getWidth();
		virtual int getHeight();

		virtual void setParam(const std::string &paramName, const std::string &paramValue);

		void setDump(const bool dump);
		bool isDumpOn() const;
		CPainterBase *getDumpPainter() const;
	

		void disable();
		void enable();

		bool isEnabled() const;
	protected:
		char mask[200];
		std::ofstream of;
		bool figOpen;
		int imNumber;
        std::string _lastFileName;

		// if true, all command are stored with CPainterDump painter
		bool doDump;

		CPainterBase *dump;

		// if true, painting is done, if false, all painter command are ignored
		bool enabled;	
};


} // namespace


#endif

