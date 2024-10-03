#ifndef _CPAINTERS_CPAINTER_COLOR_H_
#define _CPAINTERS_CPAINTER_COLOR_H_

#include <string>

namespace CPainters {

class CColor {

	public:
		CColor(const unsigned char _r=0, const unsigned char _g=0, 
				const unsigned char _b=0, const unsigned char _a=255);
		
        //CColor(const double _r, const double _g, const double _b);

		CColor(const char *colorName, const unsigned char _a = 255);

        CColor &operator=(const CColor &rhs) {
            if (this != &rhs) {
                r = rhs.r; g = rhs.g; b = rhs.b; a = rhs.a;
            }
            return *this;
        }
        
        void setColor(const char *colorName);
    
        
        // set color in range 0-1
        void setFloatRGB(const double red, const double green, const double blue, const double _a=255);


		unsigned char getRed() const;
		unsigned char getGreen() const;
		unsigned char getBlue() const;
		unsigned char getA() const;

        // return color in range 0-1
        double getRedf() const;
        double getGreenf() const;
        double getBluef() const;
        double getAf() const;

		void setA(const unsigned char aa) {
			a = aa;
		}

        /* return int in form: 0, RED, GREEN, BLUE -> RED is on the most left side */
        unsigned int getHash() const {
            unsigned int o = (((unsigned int)a << 24) & (0xFF000000)) | 
                             (((unsigned int)r << 16) & (0x00FF0000)) | 
                             (((unsigned int)g << 8) &  (0x0000FF00)) | 
                             ((unsigned int)b & 0x000000FF);
            return o;
        }

        /* return rgbcolor in format "#RRGGBB" */
        std::string getRGBColor() const;

        /* return <r,g,b> */
        std::string getPovColorRGB() const; 
        
        /* return color with alpha channel in format <r,g,b,f>, where f = 255-alpha */
        std::string getPovColorRGBF() const; 

		friend std::ostream &operator<<(std::ostream &os, const CColor &c);

	private:
		unsigned char r,g,b,a;
		
};
		

} // namespace



#endif


