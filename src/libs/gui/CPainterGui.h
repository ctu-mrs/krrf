
#ifndef _CPAINTERGUI_H_
#define _CPAINTERGUI_H_


#include "CPainterBase.h"
#include <fstream>
#include <pthread.h>
#include <semaphore.h>
#include "SDL.h"
#include <list>

namespace CPainters {

class CPainterGui: public CPainterBase {

	public:

			class Circle {
				public:
					gPoint p;
					Uint32 color, fillColor;
					int	size,lsize;
					Circle(const gPoint &ip, const Uint32 &ic, const Uint32 &ifc,
							const int isize, const int ilsize) :
						p(ip),color(ic),fillColor(ifc),size(isize),lsize(ilsize) 
						{ }
			};

			class PolyLine {
				public:
					int width;
					int type;
					Uint32 color, fillColor;
					std::vector<gPoint> pts;

					~PolyLine() {
						pts.clear();
					}	
			};

			class Text {
				public:
					Text(const Text &it):t(it.t),p(it.p),color(it.color) {}
					Text(const std::string &it, const gPoint &ip, const Uint32 c):
							t(it),p(ip),color(c) {}
					std::string t;
					gPoint p;
					Uint32 color;
			};

			

			CPainterGui(const int w, const int h,const std::vector<double> &dimension,
					const int behav, CPainterBase *_dump = NULL);

			virtual ~CPainterGui();
			
			virtual void begin();
			virtual void close();
			virtual int getWidth();
			virtual int getHeight();

            virtual bool isGui() const { return true; }

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

			std::list<PolyLine> getPolylines() const;
			std::list<Circle> getCircles() const;
			std::list<Text> getTexts() const;

			void transformPrevious();
			void setNewWinSize(const int w, const int h);

			void toggleGrid();
			void toggleGridtext();
			void selectBestGrid();
			int getImNumber();

			void changeRotateType();
			int getRotateType();

			std::list<CPainterGui::PolyLine> getGrid();
			std::list<CPainterGui::Text> getGridText();

			sem_t doneSem, dataSem, newDataSem,sizeSem, semGridStep;
			sem_t rotateSem;


			bool isDone;
			bool autoscaleframe;
			bool newData;
			bool sameXYScale;
			int width;
			int height;
			int showGrid;
			int showGridText;
			int rotateType;  // 0 .. no, 1 .. 90 clockwise etc.
		private:

			void init();
			void tGP(gPoint &p) const;
			void rotatePoint(gPoint &p, const int rotation) const;
			void transform();
			void determineScale();
			bool getBounds(double &minx, double &maxx, double &miny, double &maxy, const int type);
			void rotateNewData();

			pthread_t thread;
			double dx,dy;
			const static int fontSize;
			double sx;
			double sy;
			double gridXStep;
			double gridYStep;
			std::vector<CColor> showGridColor, showGridTextColor;


			// objects for painting
			std::list<Circle> newgPoints, points, oldgPoints;
			std::list<PolyLine> newPolys, polys, oldPolys;
			std::list<Text> texts, newTexts, oldTexts;

};



} // namespace imr


#endif



