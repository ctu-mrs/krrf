
#include "CPainterGui.h"

#include <iostream>
#include <math.h>
#include <SDL.h>
#include "SDL_gfxPrimitives.h"
#include "SDL_thread.h"
#include "SDL_image.h"
#include "lama.h"
#include <unistd.h>

#define PL_LINE 1
#define PL_GRID 2

namespace CPainters {

using namespace std;
const int CPainterGui::fontSize = 15;



Uint32 toColor(const CColor &c, const int alpha = 255) {
	Uint32 cc;

	cc = ((c.getRed() <<   24) & 0xFF000000) | 
		 ((c.getGreen() << 16) & 0x00FF0000) |
		 ((c.getBlue()  << 8 ) & 0x0000FF00) | 
		 (c.getA() & 0x000000FF);
//	cerr << "Color alpha is " << (int)c.getA() << "\n";
//	cerr << "cc je " << (int)(cc & 0xFF) << "\n";
	return cc;
}

// draw circles
template<typename T>
inline void drawgPoints(const T &circles, SDL_Surface *scr) {

	int x,y;
	for(typename T::const_iterator i = circles.begin(); i != circles.end(); i++) 
	{
		if (i->size > 0) {
			x = (int)round(i->p.x());
			y = (int)round(i->p.y());
			filledCircleColor(scr,x,y,i->size,i->color);
			if (i->fillColor != 0 && (i->size - i->lsize) > 0) {
				filledCircleColor(scr,x,y,i->size-i->lsize,i->fillColor);
			}
		}
	}

}

//draw line with given width
inline void drawLine(SDL_Surface *scr, const gPoint &p1, const gPoint &p2, 
		const int width, const Uint32 color) {

	double x1 = p1.x();
	double y1 = p1.y();
	double x2 = p2.x();
	double y2 = p2.y();
	double angle,s,c;
	const double w = width/2.0;

	if (width > 2) {
		Sint16 vvx[4];
		Sint16 vvy[4];
		angle = atan2(y2-y1,x2-x1);
		s = sin(angle);
		c = cos(angle);
		vvx[0] = (int)round(x1 - w*s);
		vvy[0] = (int)round(y1 + w*c);
				
		vvx[1] = (int)round(x1 + w*s);
		vvy[1] = (int)round(y1 - w*c);

		vvx[2] = (int)round(x2 + w*s);
		vvy[2] = (int)round(y2 - w*c);
				
		vvx[3] = (int)round(x2 - w*s);
		vvy[3] = (int)round(y2 + w*c);
	
		filledPolygonColor(scr,vvx,vvy,4,color);
	} 
	else if (width > 0) {
		lineColor(scr,(int)x1,(int)y1,(int)x2,(int)y2,color);	
	}

}

template<typename T>
void drawPolyLine(const T &t, SDL_Surface *scr,
	   	const int width, const Uint32 color, const bool closeLoop) {

//	Sint16 vvx[4];
// 	Sint16 vvy[4];
//	double angle,s,c;
//	double x1,y1,x2,y2;
//	const int w = width/2;

	typename T::const_iterator i,j;

	for(i = t.begin(); i != t.end();++i) {
		j = i;
		j++;
		if (j != t.end()) {
			drawLine(scr,*i,*j,width,color);
		}
	}

	if (closeLoop && t.size() > 2) {
		i = t.begin();
		j = t.end();
		--j;
		drawLine(scr,*i,*j,width,color);
	}
}

template<typename T>
void drawPolyLine(const T &t, SDL_Surface *scr,
	   	const int width, const int r,
		const int g, const int b, const bool closeLoop) 
{
	drawPolyLine<T>(t,scr,width,toColor(CColor(r,g,b)),closeLoop);
}



template<typename T>
void drawPolys(const T &p, SDL_Surface *scr) {

//	Sint16 vvx[4];
//	Sint16 vvy[4];

	for(typename T::const_iterator i = p.begin(); i != p.end(); i++) {

		if (i->fillColor != 0) {		
			// filled polygon
			Sint16 *vx = new Sint16[i->pts.size()];
			Sint16 *vy = new Sint16[i->pts.size()];
			for(int j=0;j<(int)i->pts.size();j++) {
				vx[j] = (int)round(i->pts[j].x());
				vy[j] = (int)round(i->pts[j].y());
			}
			filledPolygonColor(scr,vx,vy,i->pts.size(),i->fillColor);
			drawPolyLine(i->pts,scr,i->width,i->color,true);

		} else {
			drawPolyLine(i->pts,scr,i->width,i->color,false);
		}

	}
	
}

template<typename T>
inline void drawTexts(const T &t, SDL_Surface *scr) {
	int x,y;
	for(typename T::const_iterator i = t.begin(); i != t.end(); i++) {
		x = (int)round(i->p.x());
		y = (int)round(i->p.y());
		stringColor(scr,x,y,i->t.c_str(),i->color);
	}
}

void clearScreen(SDL_Surface *scr) {
	Uint32 color;
	color = SDL_MapRGB(scr->format,20,20,20);
	SDL_FillRect(scr,NULL,color);
}

void showBitmap(SDL_Surface	*scr, const char *filename) {

	SDL_Surface *tmp = IMG_ReadXPMFromArray((char **)lamaXPM);

	SDL_Surface *ret = NULL;

	if (tmp != NULL) {
		ret = SDL_DisplayFormat(tmp);
		if (ret != NULL) {
			SDL_FreeSurface(tmp);
		} else {
			cerr << "ret== NULL!\n";
		}
	} else {
		cerr << "unable to load " << filename << "!\n";
	}

	// copy ret to a scr
	SDL_Rect r;
	r.x = (scr->w >> 1) >> - (ret->w >> 1);
	r.y = (scr->h >> 1) >> - (ret->h >> 1);
	SDL_BlitSurface(ret, NULL, scr, &r);
	SDL_UpdateRect(scr,r.x,r.y,r.w,r.h);
}

// in this function all painting to sdl window is held
void *paintLoop(void *d) {

	CPainterGui *p = (CPainterGui *)d;

	// inis SDL window
	if (SDL_Init(SDL_INIT_VIDEO) == -1) {
		Uint32 flags = SDL_WasInit(SDL_INIT_VIDEO);

		if (!(flags & SDL_INIT_VIDEO)) {
			cerr << "unable to init video mode!\n";
			SDL_Quit();
			exit(0);
		}
	}

	Uint32 Rmask = 0xFF000000;
	Uint32 Gmask = 0x00FF0000;
	Uint32 Bmask = 0x0000FF00;
	Uint32 Amask = 0x000000FF;

	SDL_Surface *scrS = SDL_SetVideoMode(p->width,p->height,32,SDL_RESIZABLE | SDL_SWSURFACE);
	SDL_Surface *scr = SDL_CreateRGBSurface(SDL_SRCALPHA,p->width,p->height,32,Rmask,Gmask,Bmask,Amask);
//	SDL_Surface *scr = SDL_SetVideoMode(p->width,p->height,0,SDL_RESIZABLE | SDL_SWSURFACE);
	SDL_WM_SetCaption("CPainterGui",NULL);
//	Uint32 color;
	bool done;

	sem_wait(&p->doneSem);
	done = p->isDone;
	sem_post(&p->doneSem);

//	clearScreen(scr);
	p->setNewWinSize(scr->w,scr->h);
	p->transformPrevious();
	SDL_SetVideoMode(p->width,p->height,0,SDL_RESIZABLE | SDL_SWSURFACE);
	clearScreen(scr);
	

	bool newData;
	bool painted = true;
	bool firstRun = true;
	bool wasStopped = false;
	SDL_Event event;


	list<CPainterGui::Circle> c;
	list<CPainterGui::PolyLine> pl,grid;
	list<CPainterGui::Text> texts,gridText;
	int imNum = 0;
	int rotateType = p->getRotateType();

	unsigned long paintIter = 0;
	char caption[100];
	char info[200];

	while(!done) {

		sprintf(info,"r=%d",rotateType);

		sem_wait(&p->newDataSem);
		newData = p->newData;
		
		if (newData) {
			c.clear();
			pl.clear();
			texts.clear();
			grid.clear();
			gridText.clear();

			c = p->getCircles();
			pl = p->getPolylines();
			texts = p->getTexts();
			grid = p->getGrid();
			gridText = p->getGridText();

			p->newData = false;
			painted = false;
		}
		sem_post(&p->newDataSem);

		if (!painted) {
			imNum = p->getImNumber();
			sprintf(caption,"[%s] CPainterGui(%d)",info,imNum);
			SDL_WM_SetCaption(caption,NULL);
			clearScreen(scr);
			drawPolys(pl,scr);
			drawgPoints(c,scr);
			drawTexts(texts,scr);
			drawPolys(grid,scr);
			drawTexts(gridText,scr);
			painted = true;
			firstRun = false;
		}

		if (!p->isEnabled()) {
			sprintf(caption,"[%s] CPainterGui(%d): STOPPED ",info,imNum);
			SDL_WM_SetCaption(caption,NULL);
			wasStopped = true;
		} else if (wasStopped) {
			sprintf(caption,"[%s] CPainterGui(%d)",info,imNum);
			SDL_WM_SetCaption(caption,NULL);
			wasStopped = false;
		}
		
		if (paintIter < 2 || firstRun == true) {
			//showBitmap(scr, "lama.png");
		}

		SDL_UpdateRect(scr,0,0,0,0);
		SDL_BlitSurface(scr,0,scrS,0);
		SDL_Flip(scrS);

		if (wasStopped) {
			SDL_Delay(100);
		} else {
			SDL_Delay(60);
		}

		while(SDL_PollEvent(&event)) {
			switch(event.type) {
				case SDL_VIDEORESIZE:
					{
						p->setNewWinSize(event.resize.w,event.resize.h);
						p->transformPrevious();
//						SDL_SetVideoMode(p->width,p->height,0,SDL_RESIZABLE | SDL_SWSURFACE);
						scrS=SDL_SetVideoMode(p->width,p->height,0,SDL_RESIZABLE | SDL_SWSURFACE);
						SDL_FreeSurface(scr);
						scr = SDL_CreateRGBSurface(SDL_SRCALPHA,p->width,p->height,32,Rmask,Gmask,Bmask,Amask);

						break;
					}
				case SDL_QUIT: 
					{
						sem_wait(&p->doneSem);
						p->isDone = true;
						sem_post(&p->doneSem);
						break;
					}
				case SDL_KEYDOWN:
					{
						switch(event.key.keysym.sym) {
							case SDLK_g:
								{
									p->toggleGrid();
									if (p->showGrid) {
										grid = p->getGrid();
									} else {
										grid.clear();
									}
									painted = false;
									break;
								}
							case SDLK_h:
								{
									p->toggleGridtext();
									if (p->showGridText) {
										gridText = p->getGridText();
									} else {
										gridText.clear();
									}
									painted = false;
									break;
								}
							case SDLK_a:
								{
									sem_wait(&p->semGridStep);
									p->selectBestGrid();
									sem_post(&p->semGridStep);
									painted = false;
									break;
								}
							case SDLK_s:
								{
									if (p->isEnabled()) {
										p->disable();
									} else {
										p->enable();
									}
								}
							case SDLK_r:
								{
									p->changeRotateType();
									p->transformPrevious();
									painted = false;
									rotateType = p->getRotateType();				
								}
						}
						break;
					}
			} // switch even type
		} // while poll event
		sem_wait(&p->doneSem);
		done = p->isDone;
		sem_post(&p->doneSem);

		paintIter++;

	} // while !done

	SDL_Quit();

	c.clear();
	pl.clear();
	texts.clear();
	grid.clear();
	gridText.clear();

	p->disable();
	cerr << "quit loop";
	pthread_exit(NULL);
}


// must be called after initialization with and height
void CPainterGui::init() {

	sem_init(&doneSem,0,1);
	sem_init(&dataSem,0,1);
	sem_init(&newDataSem,0,1);
	sem_init(&sizeSem,0,1);
	sem_init(&semGridStep,0,1);
	sem_init(&rotateSem,0,1);

	pthread_create(&thread,NULL,&paintLoop,(void *)this);

}

void CPainterGui::setNewWinSize(const int w, const int h) {
	sem_wait(&sizeSem);
	width = w;
	height = h;
	sem_post(&sizeSem);
}


CPainterGui::CPainterGui(const int w, const int h,
		const vector<double> &dimension, const int behav = 0,
		CPainterBase *_dump):CPainterBase("mask", _dump)
{

	isDone = false;
	newData = false;
	showGrid = 0;
	showGridText = 0;

	showGridColor.push_back(CColor("purple"));
	showGridColor.push_back(CColor("purple"));
	showGridColor.push_back(CColor("yellow1"));
	showGridColor.push_back(CColor("green"));

	showGridTextColor.push_back(CColor("purple"));
	showGridTextColor.push_back(CColor("purple"));
	showGridTextColor.push_back(CColor("yellow1"));
	showGridTextColor.push_back(CColor("green"));



	width = w;
	height = h;

	sameXYScale = true;


	if (dimension[1] - dimension[0] == 0)
		sx = 1;
	else
		sx = width/(dimension[1] - dimension[0]);

	if (dimension[3] - dimension[2] == 0)
		sy = 1;
	else
		sy = height/(dimension[3] - dimension[2]);

	dx = dimension[0];
	dy = dimension[2];

	if (sameXYScale) {
		sx = std::min(sx,sy);
		sy = sx;
	}



	autoscaleframe = false;
	if (behav == 1)
		autoscaleframe = true;

	gridXStep = 50;
	gridYStep = 50;

	rotateType = 0;

	init();
}

CPainterGui::~CPainterGui() {
	sem_wait(&doneSem);
	isDone = true;
	sem_post(&doneSem);

	pthread_join(thread,NULL);
	usleep(20000);

	points.clear();
	newgPoints.clear();
	oldPolys.clear();

	polys.clear();
	oldPolys.clear();
	newPolys.clear();

	texts.clear();
	newTexts.clear();
	oldTexts.clear();
//	cerr << "quit cpaintergui";

	dump = NULL;
}

void CPainterGui::begin() {

	if (doDump) {
		dump->begin();
	}	
}

void CPainterGui::close() 
{
	if (doDump) {
		dump->close();
	}

	if (enabled) {

		sem_wait(&newDataSem);
/*
		points = newgPoints;
		polys = newPolys;
		texts = newTexts;

		newPolys.clear();
		newgPoints.clear();
		newTexts.clear();
*/
		newData = true;
		rotateNewData();
//		oldgPoints = points;
//		oldPolys = polys;
//		oldTexts = texts;

		if (autoscaleframe) {
			determineScale();
		}

		transform();
		imNumber++;
		sem_post(&newDataSem);
	}
}

inline void CPainterGui::rotatePoint(gPoint &p, const int rotation) const {
	double t;
	switch (rotation) {
		case 1: { t = p.xx; p.xx = -p.yy; p.yy = t; break;}
		case 2: { p.xx = -p.xx; p.yy = -p.yy; break;}
		case 3: { t = p.xx; p.xx = p.yy; p.yy = -t; break;}
	}
}

void CPainterGui::rotateNewData() {
	const int rotation = getRotateType();

	points = newgPoints;
	polys = newPolys;
	texts = newTexts;
	newPolys.clear();
	newgPoints.clear();
	newTexts.clear();
	oldgPoints = points;
	oldPolys = polys;
	oldTexts = texts;

	for(list<Circle>::iterator i = points.begin(); i != points.end(); i++) {
		rotatePoint(i->p,rotation);
	}
	for(list<PolyLine>::iterator i = polys.begin(); i != polys.end(); i++) {
		for(int j=0;j<(int)i->pts.size();j++) {
			rotatePoint(i->pts[j],rotation);
		}
	}
	for(list<Text>::iterator i = texts.begin(); i != texts.end(); i++) {
		rotatePoint(i->p,rotation);
	}
}

int CPainterGui::getImNumber() {
	int r;
	sem_wait(&newDataSem);
	r = imNumber;
	sem_post(&newDataSem);
	return r;
}

void CPainterGui::transformPrevious() {
	sem_wait(&newDataSem);

	if (autoscaleframe) {
		points.clear();
		points = oldgPoints;
		polys.clear();
		polys = oldPolys;
		texts.clear();
		texts = oldTexts;
		const int rotation = getRotateType();

		for(list<Circle>::iterator i = points.begin(); i != points.end(); i++) {
			rotatePoint(i->p,rotation);
		}
		for(list<PolyLine>::iterator i = polys.begin(); i != polys.end(); i++) {
			for(int j=0;j<(int)i->pts.size();j++) {
				rotatePoint(i->pts[j],rotation);
			}
		}
		for(list<Text>::iterator i = texts.begin(); i != texts.end(); i++) {
			rotatePoint(i->p,rotation);
		}	
		
		determineScale();
		transform();
		newData = true;
	}
//	cerr << "close: sx,sy="<<sx<<","<<sy<<",dxd,y="<<dx<<","<<dy<<"\n";

	sem_post(&newDataSem);
}

inline void CPainterGui::tGP(gPoint &p) const {
	p = gPoint((p.x()-dx) * sx, height - (p.y()-dy) * sy, p.r);
}

template<typename T>
inline void myswap(T &t1, T &t2) {
	T t = t1;
	t1 = t2;
	t2 = t;
}

// return true if bounds are determined from data
bool CPainterGui::getBounds(double &minx, double &maxx, double &miny, double &maxy, const int type) {

	const list<PolyLine> *ps = NULL;
	const list<Circle> *cir = NULL;

	if (type == 0) {
		ps = &polys;
		cir = &points;
	} else if (type == 1) {
		ps = &newPolys;
		cir = &newgPoints;
	} else {
		ps = &oldPolys;
		cir = &oldgPoints;
	}

	bool newscale = false;

//	if (cir->size() > 0) {
	if (!cir->empty()) {
		minx = cir->begin()->p.x();
		miny = cir->begin()->p.y();
		maxx = minx;
		maxy = miny;
		for(list<Circle>::const_iterator i = cir->begin(); i != cir->end();i++) {
			minx = std::min(minx,i->p.x());
			miny = std::min(miny,i->p.y());
			maxx = std::max(maxx,i->p.x());
			maxy = std::max(maxy,i->p.y());
		}
		newscale = true;
	}

	if (!ps->empty()) {
		if (newscale == false) {
			if (ps->begin()->pts.size() > 0) {
				minx = ps->begin()->pts[0].x();
				miny = ps->begin()->pts[0].y();
				maxx = minx;
				maxy = miny;
			}
		}
		for(list<PolyLine>::const_iterator i = ps->begin(); i != ps->end();i++) {
			for(int j = 0;j<(int)i->pts.size();j++) {
				minx = std::min(minx,i->pts[j].x());
				miny = std::min(miny,i->pts[j].y());
				maxx = std::max(maxx,i->pts[j].x());
				maxy = std::max(maxy,i->pts[j].y());
			}
		}
		newscale = true;
	}
//	int t = getRotateType();
//	double tmp;

//	if (t == 1 || t == 3) {
//		myswap(minx,miny);
//		myswap(maxx,maxy);
//	}

	return newscale;
}



void CPainterGui::determineScale() {

	double minx,maxx,miny,maxy;
	bool newscale = getBounds(minx,maxx,miny,maxy,0);

	if (newscale) {
		sem_wait(&sizeSem);
		sx = (width-2)/(maxx - minx);
		sy = (height-2)/(maxy - miny);
		if (sameXYScale == true) {
			sx = std::min(sx,sy);
			sy = sx;
		}
		dx = minx;
		dy = miny;
		sem_post(&sizeSem);
	}
}

void CPainterGui::draw(const gPoint &p,
		const int size, const int lsize, const CColor &lColor, const CColor &fColor) 
{
	if (doDump) {
		dump->draw(p,size,lsize,lColor,fColor);
	}

	if (enabled) {
		newgPoints.push_back(Circle(p,toColor(lColor),toColor(fColor),size,lsize));
	}
}

void CPainterGui::draw(const vector<gPoint> &pointList, const int lsize, const CColor &lColor) 
{	

	if (doDump) {
		dump->draw(pointList,lsize,lColor);
	}

	if (enabled) {
		PolyLine pl;
		pl.pts.reserve(pointList.size());
		
        //for(list<gPoint>::const_iterator i = pointList.begin(); i != pointList.end(); i++) {

        for(int i=0;i<(int)pointList.size();i++) {
            pl.pts.push_back(pointList[i]);
		}

		pl.color = toColor(lColor);
		pl.fillColor = 0;
		pl.width = lsize;
		pl.type = PL_LINE;
		newPolys.push_back(pl);
	}
}

/**
  * draw filled curve.
  */
void CPainterGui::draw(const std::vector<gPoint> &pointList,
		const int lsize, const CColor &lColor, const CColor &fColor) 

{
	if (doDump) {
		dump->draw(pointList,lsize,lColor,fColor);
	}

	if (enabled) {
		PolyLine pl;
		pl.pts.reserve(pointList.size());

        for(int i=0;i<(int)pointList.size();i++) {
            pl.pts.push_back(pointList[i]);
		}

		pl.color = toColor(lColor);
		pl.fillColor = toColor(fColor);
		pl.width = lsize;
		pl.type = PL_LINE;
		newPolys.push_back(pl);
	}
}


void CPainterGui::draw(const char *text, const gPoint &p, const int size){
	if (doDump) {
		dump->draw(text,p,size);
	}
	if (enabled) {
		newTexts.push_back(Text(string(text),p,toColor(CColor("purple1"))));
	}
}

list<CPainterGui::PolyLine> CPainterGui::getPolylines() const {
	return polys;
}

list<CPainterGui::Circle> CPainterGui::getCircles() const {
	return points;
}

list<CPainterGui::Text> CPainterGui::getTexts() const {
	return texts;
}


void CPainterGui::transform() {

//	const int rotation = getRotateType();

	for(list<Circle>::iterator i = points.begin(); i != points.end(); i++) {
			tGP(i->p);
			if (i->p.r) {
				i->size=(int)round(i->size*sx);
			}
	}

	for(list<PolyLine>::iterator i = polys.begin(); i != polys.end(); i++) {
		for(int j=0;j<(int)i->pts.size();j++) {
			tGP(i->pts[j]);
			//cerr << "PLPoint(" << i->pts[j].x() << "," << i->pts[j].y() << "\n";
		}
	}

	for(list<Text>::iterator i = texts.begin(); i != texts.end(); i++) {
		tGP(i->p);
	}

}


void CPainterGui::toggleGrid() {

	if (++showGrid > (int)showGridColor.size()-1) 	
		showGrid = 0;

}

void CPainterGui::toggleGridtext() {
	if (++showGridText > (int)showGridTextColor.size()-1) {
	   	showGridText = 0;
	}
}	




list<CPainterGui::PolyLine> CPainterGui::getGrid() {
	list<PolyLine> grid;

	if (!showGrid) return grid;

	sem_wait(&semGridStep);
	const double gridX = gridXStep;
	const double gridY = gridYStep;
	sem_post(&semGridStep);
	
	double minx,maxx,miny,maxy;
	double b = getBounds(minx,maxx,miny,maxy,2);
	double x,y;
	const int alpha = 250;

	if (!b) {
		return grid;
	}

//	int rotation = getRotateType();

	for(x = 0;x < maxx; x+=gridX) {
		PolyLine p;
		p.pts.push_back(gPoint(x,miny));
		p.pts.push_back(gPoint(x,maxy));
		grid.push_back(p);
		p.pts.clear();
	}

	for(x = 0; x > minx; x-=gridX) {
		PolyLine p;
		p.pts.push_back(gPoint(x,miny));
		p.pts.push_back(gPoint(x,maxy));
		grid.push_back(p);
		p.pts.clear();
	}

	for(y = 0; y < maxy; y+=gridY) {
		PolyLine p;
		p.pts.push_back(gPoint(minx,y));
		p.pts.push_back(gPoint(maxx,y));
		grid.push_back(p);
		p.pts.clear();
	}

	for(y = 0; y > miny; y-=gridY) {
		PolyLine p;
		p.pts.push_back(gPoint(minx,y));
		p.pts.push_back(gPoint(maxx,y));
		grid.push_back(p);
		p.pts.clear();
	}

	for(list<PolyLine>::iterator i = grid.begin(); i != grid.end(); i++) {
		i->type = PL_GRID;
		i->width = 1;
		i->color = toColor(showGridColor[showGrid],alpha); 
		if (i->pts[0].x() == 0 || i->pts[0].y() == 0) {
			i->color = toColor(showGridColor[(showGrid+1)%showGridColor.size()],alpha);
			i->width=2;
		}
		i->fillColor = 0;
	}

	for(list<PolyLine>::iterator i = grid.begin(); i != grid.end(); i++) {
		for(int j=0;j<(int)i->pts.size();j++) {
			tGP(i->pts[j]);
		}
	}

	return grid;
}


list<CPainterGui::Text> CPainterGui::getGridText() {

	sem_wait(&semGridStep);
	const double gridX = gridXStep;
	const double gridY = gridYStep;
	sem_post(&semGridStep);


	list<Text> res;
	if (!showGridText) 
		return res;

	double minx,maxx,miny,maxy;
	double b = getBounds(minx,maxx,miny,maxy,0);
	double x,y;
	char tt[20];
	const int alpha = 250;

	if (!b)
		return res;


	if (0 >= miny && 0 <= maxy) {
		for(x = 0;x < maxx; x+=gridX) {
			sprintf(tt,"%.1lf",x);
			res.push_back(Text(tt,gPoint(x,0),
						toColor(showGridTextColor[showGridText],alpha)));
		}

		for(x = 0; x > minx; x-=gridX) {
			sprintf(tt,"%.1lf",x);
			res.push_back(Text(tt,gPoint(x,0),
						toColor(showGridTextColor[showGridText],alpha)));
		}
	}
	if (0 >= minx && 0 <= maxx) {
		for(y = 0; y < maxy; y+=gridY) {
			sprintf(tt,"%.1lf",y);
			res.push_back(Text(tt,gPoint(0,y),
						toColor(showGridTextColor[showGridText],alpha)));
		}

		for(y = 0; y > miny; y-=gridY) {
			sprintf(tt,"%.1lf",y);
			res.push_back(Text(tt,gPoint(0,y),
						toColor(showGridTextColor[showGridText],alpha)));
		}
	}

	return res;


}

void CPainterGui::selectBestGrid() {

	double minx,maxx,miny,maxy;
	double b = getBounds(minx,maxx,miny,maxy,0);

	if (b) {
		gridXStep = (maxx - minx) / 10.0;
		gridYStep = (maxy - miny) / 10.0;
	}

}

int CPainterGui::getWidth() {
	return width;
}

int CPainterGui::getHeight() {
	return height;
}

void CPainterGui::changeRotateType() {
	sem_wait(&rotateSem);
	if (++rotateType > 3) { 
		rotateType = 0;
	}
	sem_post(&rotateSem);
}

int CPainterGui::getRotateType()  {
	sem_wait(&rotateSem);
	int r = rotateType;
	sem_post(&rotateSem);
	return r;
}


} // namespace rrtplanning


