#ifndef _CPAINTER_DUMP_H__
#define _CPAINTER_DUMP_H__

#include <fstream>
#include "CPainterBase.h"

/**
  * this painter just store all incoming paint requests to a txt
  * file that can be used lately to make painting to any other painter
  */

namespace CPainters {

class CPainterDump: public CPainterBase {

	public:
		CPainterDump(const char *_mask, CPainterBase *_dump = NULL);

		virtual ~CPainterDump();

		virtual void begin();
		
		virtual void close();

		virtual void draw(const gPoint &p, const int size, const int lsize, const CColor &lColor, 
				const CColor &fColor);

		virtual void draw(const std::vector<gPoint> &pointList, const int lsize, const CColor &lColor);

		virtual void draw(const std::vector<gPoint> &pointList, const int lsize, const CColor &lColor,
					const CColor &fColor);

		virtual void draw(const char *text, const gPoint &p, const int size = -1);
		
		virtual void setParam(const std::string &paramName, const std::string &paramValue);

		virtual int getWidth();
		virtual int getHeight();
	
	private:
		void ppoint(std::ofstream &ofs, const gPoint &p) const ;
		void pcolor(std::ofstream &ofs, const CColor &c) const;
		void ppoints(std::ofstream &ofs, const std::vector<gPoint> &pts) const;
};


} // namespace


#endif

