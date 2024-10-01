
#include "CPainterCairo.h"

#include <iostream>
#include <cairo.h>
#include <cairo-pdf.h>
#include <math.h>

namespace CPainters {

using namespace std;

const int CPainterCairo::fontSize = 15;

/**
  * dimension is array of: {minx, maxx, miny, maxy}
  * so width of a image is (maxx-minx) and it's height
  * is (maxy - miny)
  */
CPainterCairo::CPainterCairo(	const char *mask, const SurfaceType type,
		const vector<double> &dimension, CPainterBase *_dump, const double scale):CPainterBase(mask,_dump)
{
		_scale = scale;
		cr = NULL;
		surface = NULL;
		width = (int)lround(scale*dimension[1]-scale*dimension[0]);
		height = (int)lround(scale*dimension[3]-scale*dimension[2]);
		sx = 1;
		sy = 1;
		if (width > 5000) {
			sx = 5000.0 / width;
			sy = sx;
			width = 5000;
			height = lroundl(sy*height);
		}
		if (height > 5000) {
			sy = 5000.0/height;
			sx = sy;
			height = 5000;
			width = lroundl(sx*width);
		}
		surfacet = type;
		dx = scale*dimension[0];
		dy = scale*dimension[2];

}

CPainterCairo::~CPainterCairo() {
	if (figOpen)
		close();
	surface = NULL;
	cr = NULL;

	dump = NULL;
}


// see getCanvasFormat for details how to access the data */
unsigned char *CPainterCairo::getCanvasData() {

    if (surface) {
       return cairo_image_surface_get_data(surface);
    }

    return NULL;

}

/** access is:
    int w,h,stride;
    ((CPainterCairo *)pa)->getCanvasFormat(w,h,stride);

    for(int i = 0; i < h; i++) {
        unsigned char *row = data + i*stride;
        for(int j=0;j<w;j++) {

            int idx = i*stride + j*4; // pixel with = 4
            //cerr << "idx=" << idx << "\n";
            data[idx+0] = 0xFF; // red
            data[idx+1] = 0x00; // green
            data[idx+2] = 0xFF; // blue
            data[idx+3] = 0xFF; // alpha, FF=full color, 0=transparent 
        }
    }
*/
void CPainterCairo::getCanvasFormat(int &width, int &height, int &stride) {
    width = -1;
    height = -1;
    stride = -1;
    if (surface) {
        width = cairo_image_surface_get_width(surface);
        height = cairo_image_surface_get_height(surface);
        stride = cairo_image_surface_get_stride(surface);
    }
}


void CPainterCairo::begin() {
	if (doDump) {
		dump->begin();
	}

	if (figOpen)
		close();

	if (surfacet == 0) 
		surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
				width,height);
	else if (surfacet == 1) {
		char name[2000];
		snprintf(name,sizeof(name),"%s.%06d.pdf",mask,imNumber);
        _lastFileName = name;
		surface = cairo_pdf_surface_create(name,width,height);
	}
	
	cr = cairo_create(surface);
	figOpen = true;
}

void CPainterCairo::close() {
	char fname[2000];
	snprintf(fname,sizeof(fname),"%s.%06d.png",mask,imNumber);
    _lastFileName = string(fname);
	imNumber++;
    close(fname);

}

void CPainterCairo::close(const char *outfile) {
	if (doDump) {
		dump->close();
	}

	if (!figOpen)
		return;

	figOpen = false;

	cairo_destroy(cr);

	if (surfacet == 0) {
		cairo_surface_write_to_png(surface,outfile);
	} else if (surfacet == 1) {
		
	}

	
	cairo_surface_destroy(surface);

	surface = NULL;
	cr = NULL;
	
}

/**
  * draw single point
  */
void CPainterCairo::draw(const gPoint &p, const int size, const int lsize, const CColor &lColor, const CColor &fColor) {
	
	if (doDump) {
		dump->draw(p,size,lsize,lColor,fColor);
	}

	if (!figOpen || !enabled || size==0) {
		return;
	}

	double x,y;
	togPoint(p,x,y);

	cairo_stroke(cr);
	cairo_arc(cr,x,y,size,0,2*M_PI);

	setColor(fColor);
	cairo_fill_preserve(cr);
	
	setColor(lColor);
	cairo_set_line_width(cr,lsize);
	cairo_stroke(cr);
}

/**
  * print coordinates of a point
  *
  */
void CPainterCairo::togPoint(const gPoint &p, double &x, double &y) const 
{
	x = (_scale*p.x()-dx)  * sx;
	y = (_scale*p.y()-dy)  * sy;
	y = height-y;
}

/**
  * return integer representation of a given color
  * see XFig format
  */
void CPainterCairo::setColor(const CColor & c) {
	int r,g,b,a;
	r = c.getRed();
	g = c.getGreen();
	b = c.getBlue();
	a = c.getA();

//	cairo_set_source_rgb(cr,(double)r/255.0,(double)g/255.0,(double)b/255.0);
	cairo_set_source_rgba(cr,(double)r/255.0,(double)g/255.0,(double)b/255.0, (double)a/255.0);

}


/**
  * draw curve from given points. does not write point itself
  */
void CPainterCairo::draw(const vector<gPoint> &pointList,
        const int lsize, const CColor &lColor) {	

    if (doDump) {
        dump->draw(pointList,lsize,lColor);
    }

    if (!figOpen || !enabled || pointList.size() == 0) {
        return;
    }


    double x,y;
    togPoint(pointList[0],x,y);
    cairo_move_to(cr,x,y);
    for(int i=0; i < (int)pointList.size(); i++) {
        togPoint(pointList[i],x,y);
        cairo_line_to(cr,x,y);
    }
    cairo_set_line_width(cr,lsize);
    setColor(lColor);
    cairo_stroke(cr);

}

/**
  * draw filled curve.
  */
void CPainterCairo::draw(const vector<gPoint> &pointList,
		const int lsize, const CColor &lColor, const CColor &fColor) {

	if (doDump) {
		dump->draw(pointList,lsize,lColor,fColor);
	}

	if (!figOpen || !enabled || pointList.size() == 0) {
		return;
	}

	double x,y;
	togPoint(pointList[0],x,y);
	cairo_move_to(cr,x,y);

    for(int i=0;i<(int)pointList.size();i++) {
		togPoint(pointList[i],x,y);
		cairo_line_to(cr,x,y);
	}
	cairo_close_path(cr);
	setColor(fColor);
	cairo_fill_preserve(cr);
	cairo_set_line_width(cr,lsize);
	setColor(lColor);
	cairo_stroke(cr);

}



void CPainterCairo::draw(const char *text, const gPoint &p, const int size){

	if (doDump) {
		dump->draw(text,p,size);
	}	
	
	if (!figOpen || !enabled) {
		return;
	}

	double xx,yy;
	togPoint(p,xx,yy);

	const int fsize = size < 0 ? fontSize : size;

	cairo_select_font_face(cr,"sans",
			CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_BOLD);
	cairo_set_font_size(cr,fsize);
	cairo_move_to(cr,(int)round(xx),(int)round(yy));
	//cairo_show_text(cr,text);
	cairo_text_path(cr,text);
	setColor(CColor("GreenYellow"));
	cairo_fill_preserve(cr);
	setColor(CColor("black"));
	cairo_set_line_width(cr,0.8);
	cairo_stroke(cr);
	
}

int CPainterCairo::getWidth() {
	return width;
}

int CPainterCairo::getHeight() {
	return height;
}


} 


