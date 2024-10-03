
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>

#include "CPainters.h"
#include "CColorMap.h"
#include <math.h>

using namespace std;
using namespace CPainters;

int counter = 0;
#define intro counter++; if (id == -1) std::cerr << counter << " :"<< __FUNCTION__ << "\n"; if (id != counter) return;

static void printPercentStatus(const double actual, const double max, double &old, std::ostream &os) {

	char tmp[20];
	double n = 100*actual/max;
	if (fabs(n-old) > 0.1) {
		sprintf(tmp,"%.2lf%%   \r",n);
		old = n;
		os << tmp; 
	}
}


static void setRandom(){
	time_t t;
	time(&t);
	srand(t);
}

static void getTime(struct rusage *t){
	getrusage(RUSAGE_SELF,t);
}

double getTime(struct rusage one, struct rusage two) {

	const unsigned long as = one.ru_utime.tv_sec;
	const unsigned long bs = two.ru_utime.tv_sec;
	const unsigned long aus = one.ru_utime.tv_usec;
	const unsigned long bus = two.ru_utime.tv_usec;

	return (double)((double)bs-(double)as) + 
		(double)((double)bus-(double)aus)/1000000.0;

}


vector<double> toDoubles(const string &s) {

	vector<double> tmp;

	const int n = s.size();
	char tmps[200];
	int i,j;
	i = 0;
	double d;

	while(i<n) {
		while(i<n && s[i] == ' ') i++;
		if (i>=n) return tmp;
		j = i;
		while(j<n && s[j] != ' ') j++;
		
		for(int k=i;k<j;k++)
			tmps[k-i]=s[k];
		tmps[j-i] = '\0';
		sscanf(tmps,"%lf",&d);
		tmp.push_back(d);
		i = j;
	}
	return tmp;
}


/**
  * test dumping painter
  */
static void test2(int argc, char **argv, const int id) {
	intro;

	vector<double> d(4,0);
	d[0] = -5; d[1] = 5; d[2] = -5; d[3] = 5;

	CPainterBase *pdump = new CPainterDump("dump");
	CPainterBase *pa = new CPainterGui(500,500,d,1,pdump);

	pa->begin();
	vector<gPoint> lp;
	lp.push_back(gPoint(-5,-5));
	lp.push_back(gPoint(5,-5));
	lp.push_back(gPoint(5,5));
	lp.push_back(gPoint(-5,5));
	pa->draw(lp,3,CColor("green"),CColor("red"));
	pa->draw(gPoint(0,0),15,3,CColor("gold"),CColor("blue1"));
	pa->draw(gPoint(1,-2),5,3,CColor("red"),CColor("blue1"));
	pa->draw("ahoj",gPoint(0,2),12);
	pa->close();

	sleep(1);

}

CColor getColor(const vector<double> &data, const int from) {
	return CColor((char)data[from],(char)data[from+1],
			(char)data[from+2],(char)data[from+3]);
}


static void updateMinMaxValues(const gPoint &g, double &minx, double &miny,
		double &maxx, double &maxy, bool &first) {
	if (first == true) {
		minx = g.x();
		maxx = g.x();
		miny = g.y();
		maxy = g.y();
		first = false;
	} else {
		minx = std::min(minx,g.x());
		maxx = std::max(maxx,g.x());
		miny = std::min(miny,g.y());
		maxy = std::max(maxy,g.y());
	}
}

// load dump file, send their contents to a painter, or, if 
// dimp == NULL, just determine min and max values of points
static vector<double> paintDump(const char *filename, CPainterBase *dump, int &numFrames, const int skipFrames) {
	ifstream ifs(filename);

	int state = 0;
	string line;
	size_t pos;
	gPoint point;
	int oldSize;
	int oldNumOfPoint;
	CColor oldlColor;
	CColor oldfColor;
	double x,y,r;
	bool first = true;
	vector<gPoint> points;

    // if you want to enlarge save lines & points
    const int pointSizeAdd = 4;
    const int radiusAdd = 4;

	cerr << filename << "\n";

	double minx, miny, maxx, maxy;
	int actFrames = 0;
	double oldVal = -1;
	int lastDrawedFrame = 0;
	while(ifs) {
		switch (state) {
			case 0: {
						getline(ifs,line);
						pos = line.find('#');
						if (pos != string::npos) {
							line.erase(pos);
						}
						if (line.size() > 0) {
							if (pos = line.find('b') == 0) {
								state = 1;
							} else if ((pos = line.find('c')) == 0) {
								state = 2;
							} else if ((pos = line.find('s')) == 0) {
								state = 3;
							} else if ((pos = line.find('p')) == 0) {
								state = 4;
								line[pos] = ' ';
							} else if ((pos = line.find('l')) == 0) {
								state = 5;
								line[pos] = ' ';
							} else if ((pos = line.find('t')) == 0) {
								state = 6;
								line[pos] = ' ';
							} else if ((pos = line.find("g")) == 0) {
								state = 7;
								line[pos] = ' ';
							} else if ((pos = line.find('q')) == 0) {
								state = 10;
								line[pos] = ' ';
							}
						//	cerr << "new state = " << state << "\n";
						}
						break;
					}
			case 1: { // begin of a picture
						if (dump) {
							if (actFrames == 0 || (actFrames-lastDrawedFrame) >= skipFrames) {
								dump->begin();
							}
							actFrames++;
							printPercentStatus(actFrames,numFrames,oldVal,std::cerr);
						} else {
							numFrames++;
						}
						state = 0;
						break;
					}
			case 2: { // close picture
						if (dump) {
							dump->close();
						}
						state = 0;
						break;
					}
			case 3: {// points: line contain num of points an their colors
						state = 0;
						break;
					}
			case 4: { // point .. line contains point parametres
						vector<double> vd(toDoubles(line));
						if (dump) {
							if (vd.size() == 12) {
								x = vd[10];
								y = vd[11];
								r = false;
							} else {
								x = vd[10];
								y = vd[11];
								r = vd[12];
							}
							point = gPoint(x,y,r);
							updateMinMaxValues(point,minx,miny,maxx,maxy,first);
							dump->draw(point,(int)round(vd[0])+pointSizeAdd,(int)round(vd[1])+pointSizeAdd,
									getColor(vd,2),getColor(vd,6));
						}
						state = 0;
						vd.clear();
						break;
					}
			case 5: { // polyline 
						vector<double> vd(toDoubles(line));
						oldSize = (int)round(vd[0]);
						oldNumOfPoint = (int)round(vd[1]);
						oldlColor = getColor(vd,2);
						state = 8;
						points.clear();
						vd.clear();
						break;
					}
			case 6: {// text
					vector<double> vd(toDoubles(line));
					oldSize = (int)round(vd[0]);
					x = vd[1];
					y = vd[2];
					if (vd.size() == 4) {
						r = (int)round(vd[3]);
					} else {
						r = false;
					}
					point = gPoint(x,y,r);
					updateMinMaxValues(point,minx,miny,maxx,maxy,first);
					getline(ifs,line);					
					if (dump) {
						dump->draw(line.c_str(),point,oldSize);
					}
					state = 0;
					vd.clear();
					break;
					}
			case 7: { // polygon line
//						cerr << "Line is: " << line << "|\n";
						vector<double> vd(toDoubles(line));
						oldSize = (int)round(vd[0]);
						oldNumOfPoint = (int)round(vd[1]);
						oldlColor = getColor(vd,2);
						oldfColor = getColor(vd,6);
//						cerr <<  "COlor is: l=" << oldlColor << ", f=" << oldfColor << "\n";
						state = 9;
						points.clear();
						vd.clear();
						break;
					}
			case 8: {
					 // reading points, one point per line
					 while(ifs) {
						if (ifs >> x >> y) {
							points.push_back(gPoint(x,y));
							updateMinMaxValues(points.back(),minx,miny,maxx,maxy,first);
						} else {
							ifs.clear();
							if (ifs >> x >> y >> r) {
								points.push_back(gPoint(x,y,r+radiusAdd));
								updateMinMaxValues(points.back(),minx,miny,maxx,maxy,first);
							} else {
								ifs.clear();
								state = 0;
								if (dump) {
									dump->draw(points,oldSize+pointSizeAdd,oldlColor);
								}
								break;
							}
							
						}
					 }
					 break;
					}
			case 9: { // reading points for polygon
					while(ifs) {
						if (ifs >> x >> y) {
							points.push_back(gPoint(x,y));
							updateMinMaxValues(points.back(),minx,miny,maxx,maxy,first);
//							cerr << "poiny" << points.back().x() << "," << points.back().y() << "\n";
						} else {
							ifs.clear();
							if (ifs >> x >> y >> r) {
								points.push_back(gPoint(x,y,r+radiusAdd));
								updateMinMaxValues(points.back(),minx,miny,maxx,maxy,first);
							} else {
								ifs.clear();
								state = 0;
								if (dump) {
									dump->draw(points,oldSize+pointSizeAdd,oldlColor,oldfColor);
								}
								break;
							}		
						}
					}
					break;
					}
			case 10: {
					pos = line.find(' ',2);
					string s1(line.substr(2,pos-2));
					string s2(line.substr(pos+1,line.size()-pos-1));
//				   	cerr << "s1 = " << s1 << "\ns2 = " << s2 << "\n";
					state = 0;
					if (dump) {
						dump->setParam(s1,s2);
					}
					break;
					}

		}// switch
	}
	points.clear();
	ifs.close();
	vector<double> dimension;
	dimension.push_back(minx);
	dimension.push_back(maxx);
	dimension.push_back(miny);
	dimension.push_back(maxy);
	return dimension;
}

static void loadToPainter(int argc, char **argv, const int id) {
	intro;
	if (argc < 5) {
		cerr << "Convert painter dump file to another painter.\n";
		cerr << "usage: " << argv[0] << " <dumpFile> <painterType> <painterMask> <skipFrames>\n";
		cerr << "where painterType is:\n";
		cerr << "                      0 .. cairo2 pdf\n";
		cerr << "                      1 .. cairo2 png\n";
		cerr << "                      2 .. gui\n";
		cerr << "                      3 .. xfig\n\n";
		cerr << "<skipFrames>          skip each n-th frame\n";
		exit(0);
	}
	const char *dumpFile = argv[1];
	const int painterType = atoi(argv[2]);
	const char *maskName = argv[3];
	const int skipFrames = atoi(argv[4]);

	int numFrames = 0;
	CPainterBase *pa = NULL;
	vector<double> dimension(paintDump(dumpFile,NULL,numFrames,skipFrames));
	cerr << "Dimension is " << dimension[0] << " " << 
		dimension[1] << " " << dimension[2] << " " << dimension[3] << "\n";
	cerr << "Number of frames: " << numFrames << "\n";

	switch(painterType) {
        case 0: pa = new CPainterCairo(maskName,CPainterCairo::PDF,dimension);
				break;
        case 1: pa = new CPainterCairo(maskName,CPainterCairo::PNG,dimension);
				break;
		case 2: pa = new CPainterGui(400,400,dimension,1);
				break;
		case 3: pa = new CPainterXFig(maskName,dimension);
				break;
	}
	paintDump(dumpFile,pa,numFrames,skipFrames);
	
}

/**
  * test gui painter
  */
static void test1(int argc, char **argv, const int id) {
	intro;

	const int width = 500;
	const int height = 500;

	vector<double> d(4,0);
	d[0] = -5;
	d[1] = 5;
	d[2] = -5;
	d[3] = 5;

	CPainterBase *pa = new CPainterGui(width,height,d,1);

	pa->begin();
	vector<gPoint> lp;
	lp.push_back(gPoint(-5,-5));
	lp.push_back(gPoint(5,-5));
	lp.push_back(gPoint(5,5));
	lp.push_back(gPoint(-5,5));


	pa->draw(lp,3,CColor("blue"),CColor("yellow"));
	lp.clear();

	lp.push_back(gPoint(-5,-5));
	lp.push_back(gPoint(5,5));
	pa->draw(lp,5,CColor("olivedrab2"));

	pa->draw(gPoint(0,0),15,3,CColor("gold"),CColor("blue1"));
	pa->draw(gPoint(1,-2),5,3,CColor("red"),CColor("blue1"));
	pa->draw(gPoint(0,0),50,3,CColor("red",100),CColor("blue1",100));
	pa->draw("ahoj",gPoint(0,2),12);
	pa->draw("todle je painter typu GUI, okno se ukonci po 60 sec",gPoint(0,1),14);
	pa->close();

	sleep(5);

	struct rusage t1,t2;
	while(1) {
		for(int i=0;i<width;i+=50) {
			getTime(&t1);
			pa->begin();
			for(int j=0;j<height;j+=10) {
				lp.push_back(gPoint(0,j));
				lp.push_back(gPoint(width,j));
				pa->draw(lp,3,CColor("royalblue1"));
				lp.clear();			
			}
			pa->draw(gPoint(i,rand() % height),20,2,CColor("gold",100),CColor("red",200));
			pa->close();
			getTime(&t2);
			cerr << "Draw time = "  << getTime(t1,t2) << "s, ~ " << (1.0/getTime(t1,t2)) << "FPS\n";
			usleep(200000);
		}

	}

}


	
static void test3(int argc, char **argv, const int id) {
	intro;

	const int width = 500;
	const int height = 200;

	const int minx = -5; 
	const int maxx = 5;
	const int miny = 0;
	const int maxy = 1;
	vector<double> d(4,0);
	d[0] = minx; d[1] = maxx; d[2] = miny; d[3] = maxy;

	CPainterBase *pa = new CPainterGui(width,height,d,1);

	pa->begin();
	vector<gPoint> lp;
	lp.push_back(gPoint(minx,miny));
	lp.push_back(gPoint(maxx,miny));
	lp.push_back(gPoint(maxx,maxy));
	lp.push_back(gPoint(minx,maxy));

	pa->draw(lp,3,CColor("blue"),CColor("yellow"));

	pa->draw(gPoint(0,0),15,3,CColor("gold"),CColor("blue1"));
	pa->draw(gPoint(1,-2),5,3,CColor("red"),CColor("blue1"));
	pa->draw(gPoint(0,0),50,3,CColor("red",100),CColor("blue1",100));
	lp.clear();
	pa->close();
	sleep(20);
}
	

static void testXfig(int argc, char **argv, const int id) {
	intro;

/*
	const int minx = -5; 
	const int maxx = 5;
	const int miny = 0;
	const int maxy = 1;
*/

	const int minx = 0; 
	const int maxx = 297;
	const int miny = 0;
	const int maxy = 210;

	vector<double> d(4,0);
	d[0] = minx; d[1] = maxx; d[2] = miny; d[3] = maxy;

	CPainterBase *qq = new CPainterDump("dd");
	CPainterBase *pa = new CPainterXFig("pokus",d,qq);

	pa->begin();
	vector<gPoint> lp;

	lp.push_back(gPoint(0,0));
	lp.push_back(gPoint(maxx,0));
	lp.push_back(gPoint(maxx/2,maxy));

//	pa->draw("pokus",gPoint(3,4),35);
	pa->setParam("depth","49");
	pa->draw(lp,3,CColor("blue"),CColor("yellow"));
	pa->setParam("depth","50");

	lp.clear();
	lp.push_back(gPoint(0,30));
	lp.push_back(gPoint(10,15));
	lp.push_back(gPoint(21,17));
	lp.push_back(gPoint(20,16));
	pa->draw(lp,3,CColor("orangered"));

	pa->draw(gPoint(2,2),15,3,CColor("gold"),CColor("blue1"));
//	pa->draw(gPoint(1,-2),5,3,CColor("red"),CColor("blue1"));
//	pa->draw(gPoint(0,0),50,3,CColor("red",100),CColor("blue1",100));
//	lp.clear();
	pa->close();
	delete qq;
//	sleep(20);
}



static void testColorMap(int argc, char **argv, const int id) {
	intro;

	

	const int width = 1024;
	const int height = 768;

	const int minx = 0; 
	const int maxx = width;
	const int miny = 0;
	const int maxy = height;
	vector<double> d(4,0);
	d[0] = minx; d[1] = maxx; d[2] = miny; d[3] = maxy;

//	CPainterBase *pa = new CPainterGui(width,height,d,1);
	CPainterCairo *pa = new CPainterCairo("cmaps",CPainterCairo::PNG,d);

	pa->begin();
	vector<gPoint> lp;

	vector<CColorMap *> cmp;
	cmp.push_back(new CColorMap("autumn"));
	cmp.push_back(new CColorMap("bone"));
	cmp.push_back(new CColorMap("cool"));
	cmp.push_back(new CColorMap("copper"));
	cmp.push_back(new CColorMap("gray"));
	cmp.push_back(new CColorMap("hot"));
	cmp.push_back(new CColorMap("hsv"));
	cmp.push_back(new CColorMap("jet"));
	cmp.push_back(new CColorMap("pink"));
	cmp.push_back(new CColorMap("spring"));
	cmp.push_back(new CColorMap("summer"));
	cmp.push_back(new CColorMap("winter"));
	cmp.push_back(new CColorMap("turbo"));

	const int nn = 64;
	for(int i=0;i<(int)cmp.size();i++) {
		cmp[i]->setRange(0,nn);
	}

	double w = (double)width /nn;
	double h = (double)height / cmp.size();
	pa->begin();
	for(int j=0;j<(int)cmp.size();j++) {
		for(int i=0;i<nn;i++) {

			lp.push_back(gPoint(w*i,miny+j*h));
			lp.push_back(gPoint(w*(i+1),miny+j*h));
			lp.push_back(gPoint(w*(i+1),miny+(j+1)*h));
			lp.push_back(gPoint(w*i,miny+(j+1)*h));

			CColor c(cmp[j]->getColor(i));
			pa->draw(lp,3,c,c);
			char aa[100];
			sprintf(aa,"%s",cmp[j]->getName().c_str());
			pa->draw(aa,gPoint(10,miny+j*h+h/2.0),10);
			lp.clear();
		}
	}
	pa->close();

    {
	    pa->begin();
        CColorMap aa("jet");
        int nn = 200;
        aa.refineColorMap(nn);
        aa.setRange(0,nn);
    	double w = (double)width /nn;

        for(int j=0;j<(int)cmp.size();j++) {
            for(int i=0;i<nn;i++) {

                lp.push_back(gPoint(w*i,miny));
                lp.push_back(gPoint(w*(i+1),miny));
                lp.push_back(gPoint(w*(i+1),maxy));
                lp.push_back(gPoint(w*i,maxy));

                CColor c(aa.getColor(i));
                pa->draw(lp,3,c,c);
                char aa[100];
                sprintf(aa,"%s",cmp[j]->getName().c_str());
                pa->draw(aa,gPoint(10,miny+j*h+h/2.0),10);
                lp.clear();
            }
        }
        pa->close();
    }

    if (pa->isGui()){
    	sleep(20);
    }
}
	
static void testGnuplot(int argc, char **argv, const int id) {
	intro;

    Gnuplot g;

    g.showonscreen();

    
    stringstream ss;
    ss << "plot sin(x) w filledcurve fs solid 1.0 lc rgb \"royalblue1\"\n";
    //g.cmd(ss.str());
    ss.clear();

    sleep(2);

    vector<gPoint> aa;
    for(int i=0;i<10;i++) {
        const double t = i*3*M_PI/1000;
        aa.push_back(gPoint(t,sin(t)+0.2*1.0*rand()/RAND_MAX));
    }

    ss.clear();
    ss << "plot '-' w lp lc rgb 'purple1'\n";
    g.cmd(ss.str());
    for(int i=0;i<(int)aa.size();i++) {
        ss << aa[i].x() << " " << aa[i].y() << "\n";
    }
    ss << "e\n";
    g.cmd(ss.str());
    cerr << "cmd is " << ss.str() << "\n\n";

    sleep(10);

}


static void testGnuplotPainter(int argc, char **argv, const int id) {
	intro;

    CPainterBase *pa = new CPainterGnuplot();

    vector<gPoint> pts;

    pa->begin();
    pa->close();

    const int ss = 10;
    for(int i=0;i<ss;i++) {
        const double a = i*2*M_PI/ss;
        pts.push_back(gPoint(1*cos(a),1*sin(a)));
    }

    pa->begin();
    pa->draw(pts.front(),3,1,CColor("purple"),CColor("purple"));
    pa->close();

    sleep(2);

    pa->begin();
    pa->draw(pts,1,CColor("purple"));
    pa->draw(pts.front(),6,6,CColor("red"),CColor("blue"));
    pa->draw(gPoint(0,0),6,6,CColor("red"),CColor("blue"));
    pa->close();


    sleep(2);

 
    pa->begin();
    pa->draw(pts,1,CColor("green"));
    pa->draw(pts,1,CColor("red"),CColor("yellow"));
    pa->draw(pts.front(),6,6,CColor("black"),CColor("blue"));
    pa->close();


    sleep(4);

       

} 



static void demo_for_michal(int argc, char **argv, const int id) {
	intro;

	const int width = 500;
	const int height = 500;

	vector<double> d(4,0);
	d[0] = -5;
	d[1] = 5;
	d[2] = -5;
	d[3] = 5;

	//CPainterBase *pa = new CPainterGui(width,height,d,1);
	CPainterBase *pa = new CPainterCairo("prefix",CPainterCairo::PNG,d,NULL,100);
	//CPainterBase *pa = new CPainterXFig("xfigprefix",d);

	pa->begin();
	vector<gPoint> lp;
	lp.push_back(gPoint(-5,-5));
	lp.push_back(gPoint(5,-5));
	lp.push_back(gPoint(5,5));
	lp.push_back(gPoint(-5,5));
	pa->draw(lp,3,CColor("blue"),CColor(100,100,100));

    lp.clear();

	lp.push_back(gPoint(-1.5,-1.5));
	lp.push_back(gPoint(1.5,-1.5));
	lp.push_back(gPoint(1.5,1.5));
	lp.push_back(gPoint(-1.5,1.5));

    pa->draw(lp,3,CColor("green"));
	pa->close();

    for(int i=0;i<1000;i++) {
        pa->begin();

        lp.push_back(gPoint(-5,-5));
        lp.push_back(gPoint(5,-5));
        lp.push_back(gPoint(5,5));
        lp.push_back(gPoint(-5,5));
        pa->draw(lp,3,CColor("blue"),CColor(100,100,100));

        lp.clear();


        lp.push_back(gPoint(-1.5+i,-1.5));
        lp.push_back(gPoint(1.5+i,-1.5));
        lp.push_back(gPoint(1.5+i,1.5));
        lp.push_back(gPoint(-1.5+i,1.5));
        pa->draw(lp,3,CColor("green"));
        pa->close();
        usleep(50000);
        lp.clear();
    }
    
    
    sleep(10);
    
}



/** writing directly to cairo's surface */
static void test_direct_bitmap(int argc, char **argv, const int id) {
	intro;

	const int width = 500;
	const int height = 500;

	vector<double> d(4,0);
	d[0] = 0;
	d[1] = width;
	d[2] = 0;
	d[3] = height;

	CPainterBase *pa = new CPainterCairo("prefix",CPainterCairo::PNG,d,NULL,1);

	pa->begin();
    unsigned char *data = ((CPainterCairo *)pa)->getCanvasData();
    int w,h,stride;
    ((CPainterCairo *)pa)->getCanvasFormat(w,h,stride);
    cerr << "Cairo format: " << w << " x " << w << ", stride=" << stride << "\n";

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


    pa->close();
}





int main(int argc, char **argv) {

	const int k = argc < 2?-1:atoi(argv[1]);
	if (k > 0) {
		argc--;
		argv++;
	}
//	saveColorsForTex("ahoj.tex");

	// call various routines here
	test1(argc,argv,k);
	test2(argc,argv,k);
	test3(argc,argv,k);
	loadToPainter(argc,argv,k);
	testXfig(argc,argv,k);
	testColorMap(argc,argv,k);
    testGnuplot(argc,argv,k);
    testGnuplotPainter(argc,argv,k);
    demo_for_michal(argc,argv,k);
    test_direct_bitmap(argc,argv,k);
	return 0;
}

#undef intro
