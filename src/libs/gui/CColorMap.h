#ifndef _CCOLORMAP_H_
#define _CCOLORMAP_H_


#include "CPainterBase.h"
#include "colormaps/autumn.h"
#include "colormaps/bone.h"
#include "colormaps/cool.h"
#include "colormaps/copper.h"
#include "colormaps/gray.h"
#include "colormaps/hot.h"
#include "colormaps/hsv.h"
#include "colormaps/jet.h"
#include "colormaps/pink.h"
#include "colormaps/spring.h"
#include "colormaps/summer.h"
#include "colormaps/autumn.h"
#include "colormaps/winter.h"
#include "colormaps/turbo.h"
#include <string>
#include <math.h>
#include <vector>

namespace CPainters {

class CColorMap {

	public:

	CColorMap(const std::string name = "jet", const bool inverse=false):_inverse(inverse) {

        colormap = NULL;
		
        // if new map is added, it should be also in getColormapNames 

		if (name.compare("autumn")==0) {
			size = 64;
			loadCM(autumnCM,size);	
		} else if (name.compare("bone")==0) {
			size = 64;
			loadCM(boneCM,size);
		} else if (name.compare("cool")==0) {
			size = 64;
			loadCM(coolCM,size);
		} else if (name.compare("copper")==0) {
			size = 64;
			loadCM(copperCM,size);
		} else if (name.compare("gray")==0) {
			size = 64;
			loadCM(grayCM,size);
		} else if (name.compare("hot")==0) {
			size = 64;
			loadCM(hotCM,size);
		} else if (name.compare("hsv")==0) {
			size = 64;
			loadCM(hsvCM,size);
		} else if (name.compare("pink")==0) {
			size = 64;
			loadCM(pinkCM,size);
		} else if (name.compare("spring")==0) {
			size = 64;
			loadCM(springCM,size);
		} else if (name.compare("summer")==0) {
			size = 64;
			loadCM(summerCM,size);
		} else if (name.compare("autumn")==0) {
			size = 64;
			loadCM(autumnCM,size);
		} else if (name.compare("winter")==0) {
			size = 64;
			loadCM(winterCM,size);
		} else if (name.compare("turbo")==0) {
			size = 256;
			loadCM(turboCM,size);
		} else {
			size = 64;
			loadCM(jetCM,size);
		}
		cmname = name;
	}

	std::string getName() const {
		return cmname;
	}

    static void getColormapNames(std::vector< std::string> &results) {
        results.clear();
        results.push_back("jet");
        results.push_back("autumn");
        results.push_back("bone");
        results.push_back("cool");
        results.push_back("copper");
        results.push_back("grey");
        results.push_back("hot");
        results.push_back("hsv");
        results.push_back("pink");
        results.push_back("spring");
        results.push_back("summer");
        results.push_back("autumn");
        results.push_back("winter");
    }

    void refineColorMap(const int newsize) {
        // interpolate colors
        double **cm2 = new double*[newsize];
        for(int i=0;i<newsize;i++) {
            cm2[i] = new double[3];

            const double fsize = size*1.0*i/newsize;
            const int low = (int)floor(fsize);
            int high = (int)ceil(fsize);
            if (high >= size) {
                high = size - 1;
            }
            //std::cerr << "Ref: " << i << "/" << newsize << ", low/high=" << low << "," << high << ", fsize="<< fsize << ", size=" << size << "\n";


            if (low == high) {
                cm2[i][0] = colormap[low][0];
                cm2[i][1] = colormap[low][1];
                cm2[i][2] = colormap[low][2];
            } else {
                const double t = fsize - floor(fsize);
                for(int j=0;j<3;j++) {
                    cm2[i][j] = (1-t)*colormap[low][j] + t*colormap[high][j];
                }
            }
        }
        clearColormap();
        size = newsize;
        colormap = cm2;

    }

    void createColorMap(const std::vector<CColor> &colors) {
        clearColormap();

        size = colors.size();

		colormap = new double*[size];
		for(int i=0;i<size;i++) {
			colormap[i] = new double[3];
            colormap[i][0] = colors[i].getRedf();
            colormap[i][1] = colors[i].getGreenf();
            colormap[i][2] = colors[i].getBluef();
		}
    }


	void loadCM(const double m[64][3], const int size) {

		colormap = new double*[size];
		for(int i=0;i<size;i++) {
			colormap[i] = new double[3];
		}

		for(int i=0;i<size;i++) {
			for(int j=0;j<3;j++) {
				colormap[i][j] = m[i][j];
			}
		}

	}

    void clearColormap() {
		for(int i=0;i<size;i++) {
			delete [] colormap[i];
		}
		delete [] colormap;

		colormap = NULL;

    }


	~CColorMap() {
        clearColormap();
	}

	void setRange(const double _min, const double _max) {
		min = _min;
		max = _max;
	}

    void setRange(const std::vector<double> &data) {
        if (data.size() > 0) {
            min = data[0];
            max = data[0];
            for(int i=0;i<(int)data.size();i++) {
                if (data[i] < min) {
                    min = data[i];
                }
                if (data[i] > max) {
                    max = data[i];
                }
            }
        }
    }

    void setRange(const std::vector<int> &data) {
        if (data.size() > 0) {
            min = data[0];
            max = data[0];
            for(int i=0;i<(int)data.size();i++) {
                if (data[i] < min) {
                    min = data[i];
                }
                if (data[i] > max) {
                    max = data[i];
                }
            }
        }
    }




    double getMin() const { return min; }
    double getMax() const { return max; }

    int getColorIndex(const double val) const {
		const int s = getNumColors();

		int i = (int)lround((s-1)*(val-min)/(max-min));
		if (i < 0) {
			i = 0;
		}
		if (i >= s) {
			i = s-1;
		}
        return i;
    }


	CColor getColor(const double val) const{
        const int i = getColorIndex(val);

        if (_inverse) {
		    const int s = getNumColors();
    		return getColorIdx(s-1-i);
        }
    	return getColorIdx(i);
	}

    


	CColor getColorIdx(const int idx) const{
		if (idx >=0 && idx < getNumColors()) {
			return CColor((unsigned char)lround(255*colormap[idx][0]),
					(unsigned char)lround(255*colormap[idx][1]),(unsigned char)lround(255*colormap[idx][2]));
		}
		else {
			return CColor((unsigned char)lround(255*colormap[0][0]),
					(unsigned char)lround(255*colormap[0][1]),(unsigned char)lround(255*colormap[0][2]));
		}
	}

	int getNumColors() const {
		return size;
	}

    void saveToPovray(std::ofstream &ofs) const {
        ofs << "#macro cmColor" << cmname << "(integervalue)\n";
        ofs << "  #switch(integervalue)\n";
        for(int i=0;i<size;i++) {
            ofs << "  #case(" << i << ")\n";
            ofs << "   rgb <" << colormap[i][0] << "," << colormap[i][1]<<"," << colormap[i][2] << ">\n";
            ofs << "   #break\n";
        }
        ofs << "  #end\n";
        ofs << "#end\n\n";
    }


    static void saveCMStoPovray(std::ofstream &ofs) {

        std::vector< std::string > names;
        getColormapNames(names);

        for(int i=0;i<(int)names.size();i++) {
            CColorMap cm(names[i]);
            cm.saveToPovray(ofs);
        }

        ofs << "#macro cmColor(intvalue)\n";
        ofs << " cmColorjet(intvalue)\n";
        for(int i=0;i<(int)names.size();i++) {
            ofs << "// cmColor"<< names[i] << "(intvalue)\n";
        }
        ofs << "#end\n";

   }

    void saveCMToPovrayColorMap(std::ofstream &ofs) {
        
        ofs << "#declare myColorMap=color_map {\n";
        for(int i=0;i<(int)size;i++) {
            ofs << "[ " << ((double)1.0*i/size) << " rgb < "<<colormap[i][0] << "," << colormap[i][1] << "," << colormap[i][2] << "> ]\n";
        }
        ofs << "}\n";
    }

	private:
	double min, max;
	int size;
	double **colormap;
    const bool _inverse;
	std::string cmname;

};

} // namespace CPainters


#endif


