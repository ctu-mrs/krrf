
#ifndef _CPAINTERCAIRO2_H_
#define _CPAINTERCAIRO2_H_


#include "CPainterBase.h"
#include <cairo.h>

namespace CPainters {


class CPainterCairo: public CPainterBase {

    public:
        typedef enum _type {
            PNG = 0,
            PDF        
        } SurfaceType;

        CPainterCairo(
                const char *mask,
                const SurfaceType type,
                const std::vector<double> &dimension,
                CPainterBase *_dump = NULL,
                const double scale=1.0);

        unsigned char *getCanvasData();
        void getCanvasFormat(int &width, int &height, int &stride);

        virtual ~CPainterCairo();

        virtual void begin();
		virtual void close();
        void close(const char *outfile);

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
        void togPoint(const gPoint &p,
                double &x, double &y) const ;

        cairo_t *cr;
        cairo_surface_t *surface;	

        SurfaceType surfacet;
        int width;
        int height;
        double dx,dy;
        const static int fontSize;
        double sx;
        double sy;
        double _scale;
};



} // namespace
#endif



