
#ifndef _CPAINTERXFIG_H_
#define _CPAINTERXFIG_H_


#include "CPainterBase.h"
#include <cairo.h>
#include <vector>
#include <fstream>

namespace CPainters {

class CPainterXFig: public CPainterBase {

	

	public:
			CPainterXFig(const char *mask,const std::vector<double> &dimension,CPainterBase *_dump = NULL);

			virtual ~CPainterXFig();
			
			virtual void begin();
			virtual void close();

			virtual int getWidth();
			virtual int getHeight();

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

			virtual void draw(const char *text, const gPoint &p,
					const int size = -1);
			virtual void setParam(const std::string &paramName, const std::string &paramValue);

		private:
			int setColor(const CColor & c);
			void togPoint(const gPoint &p,int &x, int &y) const ;

			int width;
			int height;
			double dx,dy;
			const static int fontSize;
			double sx;
			double sy;
			int paperType;
			int depth;
			std::vector<CColor> definedColors;
			std::ofstream oft;
};



} // namespace
#endif



