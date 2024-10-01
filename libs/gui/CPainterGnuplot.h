
#ifndef _CPAINTERGNUPLOT_H_
#define _CPAINTERGNUPLOT_H_


#include "CPainterBase.h"
#include <cairo.h>
#include "gnuplot.h"

namespace CPainters {



class CPainterGnuplot: public CPainterBase {

	public:
			CPainterGnuplot(CPainterBase *_dump = NULL);

			virtual ~CPainterGnuplot();
			
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
		private:
			void setColor(const CColor & c);

            Gnuplot * _g;
            std::vector< std::vector<gPoint> > _pts;
            bool _first;
};



} // namespace
#endif



