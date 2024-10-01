
#ifndef __painting_h__
#define __painting_h__

#include <list>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

//#include <ode/ode.h>
#include "gui/CPainters.h"
#include "types.h"
#include "map.h"
#include "mapr.h"
#include "mesh.h"

#include "rrt2Bidirect.h"

#include "CRobot3D.h"
#include "rrtstate.h"

#include "gui3d/CPainters3D.h"

namespace rrtPlanning {

using std::list;
using namespace CPainters;
using namespace Painter3D;

typedef enum _TPainter {
	GUI = 0,
	PNG,
	PDF,
	XFIG,
    GNUPLOT
} TPainter;



gPoint tgp(const TPoint &p, const bool resizeable=false);
gPoint3 tgp3(const TPoint3 &p);
std::vector<gPoint3> tgp3(const std::vector<TPoint3> &pts);


void writeLights(const vector<TLight> &lights, std::ofstream &ofs);
void writeMacros(std::ofstream &ofs);

void povDrawPlTrajSegment(const std::vector<TPoint3> &pts, std::ofstream &ofs, const int index);

CPainterBase *initPainter(enum _TPainter type, const std::vector<double> &dimension, const char *fileMask, 
		CPainterBase *dump=NULL,const double scale=1.0);
		
std::string printPainterHelp();
std::string povColor(const CColor &c);
std::string povColorRaw(const CColor &color);
void povDeclareCoord(std::ofstream &ofs);

void createCylinder(std::vector< std::vector< TPoint3 > > &cyl, const double length, const double radius, const int n);

void povDeclareRobotColorMap(const std::vector<std::vector< TPoint3> > &triangles, std::ofstream &ofs);
void povDeclareRobot(const std::vector< Triangle > &triangles, std::ofstream &ofs, const CColor &color=CColor(250,255,0));


std::string printPOVtriangleDeformable(const std::vector< std::vector<TPoint3> > &triangles, const double scaleX, const double scaleY, const double scaleZ);
std::string printPOVtriangle(const Triangle &triangle);
std::string printPOVtriangles(const std::vector< Triangle > &triangles);

void povDrawPl(const std::vector<TPoint3> &pts, const double width, const CColor &color, std::ofstream &ofs, const bool noshadow=false);

void povDrawPl(const std::vector<TPoint3> &pts, const std::string &width, const CColor &color, std::ofstream &ofs, const bool noshadow=false);
void povDrawPl2(const std::vector<TPoint3> &pts, const std::string &width, const std::string &color, std::ofstream &ofs, const bool noshadow=false);


std::string dbl2str(const double d);






void povDrawTreeSegments(const std::vector<TPoint3> &pts, const std::string &color, std::ofstream &ofs, const bool noshadow=false);
void povDrawLineMacro(const std::vector<TPoint3> &pts, const std::string &macroname, std::ofstream &ofs);
void povDrawLineMacro2(const std::vector<TPoint3> &pts, const int idx, const std::string &macroname, std::ofstream &ofs);

void povIfObject(std::ofstream &ofs, const std::string &name);
void povInclude(std::ofstream &ofs, const std::string &name);

void povDeclareMap(const std::vector< Triangle > &triangles, std::ofstream &ofs);
void povDrawRobot(const std::vector<TPoint3> &triangles, std::ofstream &ofs);
void povDrawPoint(const TPoint3 &point, const double radius, const CColor &color, std::ofstream &ofs);
void povDrawPoint(const TPoint3 &point, const string &radius, const CColor &color, std::ofstream &ofs);


template<typename ITER>
void povDrawPts(ITER begin, ITER end, const double radius, const CColor  &color, std::ofstream &ofs) {
	for(ITER i = begin; i != end; i++) {
		povDrawPoint(*i,radius,color,ofs);
	}
}

template<typename ITER>
void povDrawPts(ITER begin, ITER end, const string &radius, const CColor  &color, std::ofstream &ofs) {
	for(ITER i = begin; i != end; i++) {
		povDrawPoint(*i,radius,color,ofs);
	}
}




template<typename T>
void povDrawPts(const T &container, const double radius, const CColor &color, std::ofstream &ofs) {
    povDrawPts(container.begin(), container.end(), radius, color, ofs);
}

template<typename T>
void povDrawPts(const T &container, const string &radius, const CColor &color, std::ofstream &ofs) {
    povDrawPts(container.begin(), container.end(), radius, color, ofs);
}




std::string povPoint(const TPoint3 &point);
std::string povPoint(const double &x, const double &y, const double &z);

	
	

void drawLine(std::ofstream &ofs, const TPoint3 &p1, const TPoint3 &p2, const double radius, const CColor &color);
void drawLine(std::ofstream &ofs, const TPoint3 &p1, const TPoint3 &p2, const char *radius, const CColor &color);
void drawTriangle(std::ofstream &ofs,const TPoint3 &p1, const TPoint3 &p2, const TPoint3 &p3, const CColor &color);
void drawTriangle(std::ofstream &ofs,const TPoint3 &p1, const TPoint3 &p2, const TPoint3 &p3);
void drawPoint(std::ofstream &ofs, const TPoint3 &pts, const double size, const CColor &color);


void drawMesh(ofstream &ofs, const vector<TPoint3> &pts, const int indicesSize, int *indices, const bool drawEdges, const bool drawColorHeightField, const char *colorMapName = "copper");



void blenderIntro(std::ostream &os);
void saveTreeBlender(std::ofstream &ofsr, const std::vector<TPoint3> &pts, const int offset = 0);
void saveSegmentBlender(std::ofstream &ofs, const TPoint3 &a, const TPoint3 &b, const char *cname, const char *matname, const char *radname);


void draw(const TPoint &p, const int ps, const int lw, const CColor &c, const CColor &cf, CPainterBase *pa, const bool resizeable=false);

void drawMap(const Map &m, CPainterBase *pa);
void drawMap(const MapR &m, CPainterBase *pa);
void drawMesh(const Mesh &mesh, CPainterBase *pa, const double drawExpensiveEdges=-1);

bool equalPoints(const std::vector<TPoint> &pts);


template<typename Iter>
vector<gPoint> tgPoint(Iter begin, Iter end) {
	vector<gPoint> res;
	for(Iter i = begin; i != end; ++i) {
		res.push_back(tgp(*i));
	}
	return res;
}

template<typename PT>
vector<gPoint> tgPoint(const vector<PT> &pts) {
	vector<gPoint> res;
    res.reserve(pts.size());
	for(int i=0;i<(int)pts.size();i++) {
		res.push_back(tgp(pts[i]));
	}
	return res;
}

// draw poly line
template<typename PT>
void drawPl(const std::vector<PT> &pts, const int lw, const CColor &cc, CPainterBase *pa)  {
	if (pa) {
		pa->draw(tgPoint(pts),lw,cc);
	}
}


void drawPl_2(const TPoint &p1, const TPoint &p2, const int lw, const CColor &cc, CPainterBase *pa);

// draw single points
template<typename PT>
void drawPts(const vector<PT> &pts, const int ps, const int lw, const CColor &c1, const CColor &c2, CPainterBase *pa) {
	if (pa) {
		pa->draw(tgPoint(pts),ps,lw,c1,c2);
	}
}

template<typename PT>
void drawPlg(const vector<PT> &pts, const int ps, const int lw, const CColor &cline, const CColor &cfill, CPainterBase *pa) {
	if (pa) {
		pa->draw(tgPoint(pts),lw,cline,cfill);
	}
}

void drawDim(const std::vector<double> &dim, const int ps, const int lw, const CColor &cLine, const CColor &cFill, CPainterBase *pa);




void drawColorMap(CColorMap &c, CPainterBase *pa);




template<typename PT>
void drawRobot(const vector<PT> &pts, CPainterBase *pa, const int type = 0) {
	if (type == 0) {
		//drawPlg(pts,0,0,CColor("blue",100),CColor("blue",100),pa);
		drawPlg(pts,0,0,CColor("wheat4",180),CColor("wheat4",200),pa);
	} else {
		drawPlg(pts,0,2,CColor("red"),CColor("coral",100),pa);
	}
}

void drawTriangle(const Triangle &t, const CColor &lineColor, const CColor &fillColor, CPainterBase *pa);


template<typename PT>
void drawRobot(const vector< vector<PT> > &pts, CPainterBase *pa, const int type = 0) {
    for(int i=0;i<(int)pts.size();i++) {
        drawRobot(pts[i], pa, type);
    }
}

void drawRobot(const std::vector< Triangle > &pts, CPainterBase *pa, const int type = 0);



void drawRobot(const std::vector<TShape> &s, CPainterBase *pa);


/** draw dd radius */
template<typename RRTT, typename RT>
void drawRRT_DDradius(const  RRTT &rrt, RT &robot, const bool drawRadius, CPainterBase *pa) {

    const std::vector<TNode *> &tree(rrt.getTree());

    std::vector< TPoint > pts;

	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
			pts.push_back(robot.getRefPoint(node->pstate));
			pts.push_back(robot.getRefPoint(node->state));
			drawPl(pts,2,CColor("green4"),pa);
			pts.clear();

            if (drawRadius) {
                pts.push_back(robot.getRefPoint(node->state));
                drawPts(pts, node->RRTDD_radius(),1,CColor("orange"),CColor("orange",80),pa);
            }
            pts.clear();

		}
	}
}


template<typename RT>
void drawNodeTree2(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const int numPointOnEdge, const CColor &color) {
    std::vector< TPoint > pts;
	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
			pts.push_back(robot.getRefPoint(node->pstate));
			pts.push_back(robot.getRefPoint(node->state));
			drawPl(pts,2,color,pa);
			pts.clear();
		}
	}
}




/* assuming CRobot2D */
template<typename RT>
void drawNodeTreeControl(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const CColor &color, const int numPointOnEdge) {

    typedef typename RT::State State;

	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
            std::vector< TPoint > pts;
            pts.push_back(robot.getRefPoint(node->pstate));

			robot.setState(node->pstate);
			const double t = node->RRT2DControl_timeGet() / (double) numPointOnEdge;
			pts.push_back(robot.getRefPoint(node->pstate));
            std::vector<State> rs(robot.control(node->input,t,numPointOnEdge));
            
            std::vector<TPoint> pts2(states2pts2(rs, robot) );
            std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));

            pts.push_back(robot.getRefPoint(node->state));

			drawPl(pts,2,color,pa);
			pts.clear();
		}
	}
}

/* assuming CRobot2D */
template<typename RT>
void drawNodeTreeKRRFControl(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const CColor &color) {

    typedef typename RT::State State;

	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
			const int numPointOnEdge = node->KRRF_edgeSamplesGet();
            std::vector< TPoint > pts;
            pts.push_back(robot.getRefPoint(node->pstate));

			robot.setState(node->pstate);
			const double t = node->KRRF_timeGet() / (double) numPointOnEdge;
			pts.push_back(robot.getRefPoint(node->pstate));
            std::vector<State> rs(robot.control(node->input,t,numPointOnEdge));
            
            std::vector<TPoint> pts2(states2pts2(rs, robot) );
            std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));

            pts.push_back(robot.getRefPoint(node->state));

			drawPl(pts,2,color,pa);
			pts.clear();
		}
	}
}

/* assuming CRobot2D */
template<typename RT>
void drawNodeTreeLazyTSPControl(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const CColor &color) {

    typedef typename RT::State State;

	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
			const int numPointOnEdge = node->LazyTSP_edgeSamplesGet();
            std::vector< TPoint > pts;
            pts.push_back(robot.getRefPoint(node->pstate));

			robot.setState(node->pstate);
			const double t = node->LazyTSP_timeGet() / (double) numPointOnEdge;
			pts.push_back(robot.getRefPoint(node->pstate));
            std::vector<State> rs(robot.control(node->input,t,numPointOnEdge));
            
            std::vector<TPoint> pts2(states2pts2(rs, robot) );
            std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));

            pts.push_back(robot.getRefPoint(node->state));

			drawPl(pts,2,color,pa);
			pts.clear();
		}
	}
}












/* assuming CRobot2D */
template<typename RT>
void drawNodeTree(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const CColor &color) {
    std::vector< TPoint > pts;
	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
            pts.push_back(robot.getRefPoint(node->pstate));
            pts.push_back(robot.getRefPoint(node->state));
			drawPl(pts,2,color,pa);
			pts.clear();
		}
	}
}



template<typename RT>
void drawNodeTreeARTLengths(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const int numPointOnEdge) {
    std::vector< TPoint > pts;
	
    double min = -1;
    double max = -1;
    
    for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
        const double val = node->ART_lengthGet();
        if (val < min || min == -1) {
            min = val;
            max = val;
        } 
        if (val > max ) {
            max = val;
        }
    }

    CColorMap cm("jet");
    cm.setRange(min,max);


	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
			pts.push_back(robot.getRefPoint(node->pstate));
			pts.push_back(robot.getRefPoint(node->state));
			drawPl(pts,2,cm.getColor(node->ART_idxGet()), pa); // ART_idx is index of tree, used to determine the color
			pts.clear();

            for(int j=0;j<(int)node->next.size();j++) {
                const int idx2 = node->next[j];
			    pts.push_back(robot.getRefPoint(node->state));
			    pts.push_back(robot.getRefPoint(tree[idx2]->state));
			    //drawPl(pts,2,CColor("black"), pa); // ART_idx is index of tree, used to determine the color
			    drawPl(pts,2,CColor("black"), pa); // ART_idx is index of tree, used to determine the color
                pts.clear();
            }
                
            const double val = node->ART_lengthGet();
            draw(robot.getRefPoint(node->state), 5,1, cm.getColor(val), cm.getColor(val), pa);
		}
	}
}







template<typename RT>
void drawNodeTreeART(const vector< TNode *> &tree, RT &robot,  CPainterBase *pa, const int numPointOnEdge, CColorMap &cm) {
    std::vector< TPoint > pts;
            
    const int lineWidth = 3; // 2 for non-scaled painter
	for(int i=0;i<(int)tree.size();i++) {
        TNode *node = tree[i];
		if (node->pstate.size() != 0) {
//			robot.setState(node->pstate);
//			const double t = node->RRT2D_time() / (double) numPointOnEdge;
			pts.push_back(robot.getRefPoint(node->pstate));
			pts.push_back(robot.getRefPoint(node->state));
  //          std::vector<typename RT::State> rs(robot.control(node->input,t,numPointOnEdge));
	//		for(int j=0;j<(int)rs.size();j++) {
	//			pts.push_back(robot.getRefPoint(rs[j]));
	//		}
			drawPl(pts,lineWidth,cm.getColor(node->ART_idxGet()), pa); // ART_idx is index of tree, used to determine the color
			pts.clear();

            for(int j=0;j<(int)node->next.size();j++) {
                const int idx2 = node->next[j];
			    pts.push_back(robot.getRefPoint(node->state));
			    pts.push_back(robot.getRefPoint(tree[idx2]->state));
			    drawPl(pts,lineWidth,CColor("black"), pa); // ART_idx is index of tree, used to determine the color
                pts.clear();
            }
		}
	}
}



/** draw classic RRT for new trees */
template<typename RRTT, typename RT>
void drawRRT_2D(const  RRTT &rrt, RT &robot,  CPainterBase *pa) {
    const std::vector<TNode *> &tree(rrt.getTree());
    drawNodeTree(tree, robot, pa,  CColor("green4"));

}

/** draw classic RRT for new trees */
template<typename RRTT, typename RT>
void drawRRT_2DControl(const  RRTT &rrt, RT &robot,  CPainterBase *pa, const int numPointOnEdge) {
    const std::vector<TNode *> &tree(rrt.getTree());
    drawNodeTreeControl(tree, robot, pa,  CColor("green4"), numPointOnEdge);

}

/** draw classic RRT for new trees */
template<typename RT>
void drawRRT_2DControlTree(const vector< TNode *> &tree,  RT &robot,  CPainterBase *pa, const int numPointOnEdge) {
    drawNodeTreeControl(tree, robot, pa,  CColor("green4"), numPointOnEdge);

}

/** draw RRT-Bidirrect trees */
template<typename RRTT, typename RT>
void drawKRRF_2D(const RRTT &rrt, RT &robot,  CPainterBase *pa) {


    WDEBUG("drawing KRRF tree!!!! ");
    const std::vector<TNode *> &tree1(rrt.getTreeFwd());
    drawNodeTree(tree1, robot, pa, CColor("green4"));
    const std::vector<TNode *> &tree2(rrt.getTreeBack());
    drawNodeTree(tree2, robot, pa, CColor("red"));
}

/** draw RRT-Bidirrect trees */
template<typename RRTT, typename RT>
void drawKRRF_2DControl(const RRTT &rrt, RT &robot,  CPainterBase *pa) {


    WDEBUG("drawing KRRF control trees!!!! ");
    const std::vector<TNode *> &tree1(rrt.getTreeFwd());
    drawNodeTreeKRRFControl(tree1, robot, pa, CColor("green4"));
    const std::vector<TNode *> &tree2(rrt.getTreeBack());
    drawNodeTreeKRRFControl(tree2, robot, pa, CColor("red"));
}

/** draw RRT-Bidirrect trees */
template<typename RRTT, typename RT>
void drawKRRF_2DControl(const RRTT &rrt, RT &robot,  CPainterBase *pa, bool one_color) {
	std::vector<const char *> colors{"green3", "DimGrey", "DodgerBlue3", "orange", "salmon", "peru", "khaki3", "brown", "black", "DarkOliveGreen", "sienna", "tomato2", "IndianRed", "SkyBlue", "cyan3"};

    WDEBUG("drawing KRRF control trees!!!! ");
    const std::vector<std::vector<TNode *> > &tree(rrt.getTrees());
	for (int t=0; t<tree.size(); t++){
		if (one_color){
			drawNodeTreeKRRFControl(tree[t], robot, pa, CColor("DarkOliveGreen"));
		} else {
			drawNodeTreeKRRFControl(tree[t], robot, pa, CColor(colors[t%colors.size()]));
		}
	}
	//const std::vector<TNode *> &tree2(rrt.getTreeBack());
    //drawNodeTreeKRRFControl(tree2, robot, pa, CColor("red"));
}

/** draw RRT-Bidirrect trees */
template<typename RRTT, typename RT>
void drawLazyTSP_2DControl(const RRTT &rrt, RT &robot,  CPainterBase *pa) {
	std::vector<const char *> colors{"green3", "DimGrey", "DodgerBlue3", "orange", "salmon", "peru", "khaki3", "brown", "black", "DarkOliveGreen", "sienna", "tomato2", "IndianRed", "SkyBlue", "cyan3"};

    WDEBUG("drawing lazy TSP control trees!!!! ");
    const std::vector<std::vector<TNode *> > &tree(rrt.getTrees());
	for (int t=0; t<tree.size(); t++){
		drawNodeTreeLazyTSPControl(tree[t], robot, pa, CColor(colors[t%colors.size()]));
	}
	//const std::vector<TNode *> &tree2(rrt.getTreeBack());
    //drawNodeTreeKRRFControl(tree2, robot, pa, CColor("red"));
}

/** draw RRT-Bidirrect trees */
template<typename RRTT, typename RT>
void drawRRTBidirect_2D(const RRTT &rrt, RT &robot,  CPainterBase *pa) {


    WDEBUG("drawing bidir tree!!!! ");
    const std::vector<TNode *> &tree1(rrt.getTreeFwd());
    drawNodeTree(tree1, robot, pa, CColor("green4"));
    const std::vector<TNode *> &tree2(rrt.getTreeBack());
    drawNodeTree(tree2, robot, pa, CColor("red"));
}


#if 0

template<typename RT>
void drawRRT(const  RRTBidirect<RT> &rrt, RT &robot, CPainterBase *pa) {


	const int numPointOnEdge = rrt.getEdgeSamples();
	typedef typename RRTBidirect<RT>::TreeNode TreeNode;

	double t;
	
	list<TreeNode> states(rrt.getTreeInList(0));

    std::vector< TPoint > pts;
	for(typename list<TreeNode>::iterator i = states.begin(); i != states.end(); i++) {
		if (i->prevState.size() != 0) {
			robot.setState(i->prevState);
			t = i->time / (double) numPointOnEdge;
			pts.push_back(robot.getRefPoint(i->prevState));

            std::vector<typename RT::State> rs(robot.control(i->input,t,numPointOnEdge));
            std::vector< TPoint> pts2(states2pts2(rs, robot) );
            std::back_inserter(pts2.begin(), pts2.end(), std::copy(pts));
			drawPl(pts,2,CColor("red"),pa);
			pts.clear();
		}
	}
	states = rrt.getTreeInList(1);
	for(typename list<TreeNode>::iterator i = states.begin(); i != states.end(); i++) {
		if (i->prevState.size() != 0) {
			robot.setState(i->prevState);
			t = i->time / (double) numPointOnEdge;
			pts.push_back(robot.getRefPoint(i->prevState));
            std::vector<typename RT::State> rs(robot.control(i->input,t,numPointOnEdge));
            pts.reserve(rs.size());
			for(int j=0;j<(int)rs.size();j++) {
				pts.push_back(robot.getRefPoint(rs[j]));
			}
			drawPl(pts,2,CColor("tan4"),pa);
			pts.clear();
		}
	}
	states.clear();
}

#endif



template<typename RT>
void drawTrajectory2D(const std::vector< TNode > &nodes, RT &robot, CPainterBase *pa) {

    std::vector< TPoint > pts;
	for(int i=0;i<(int)nodes.size();i++) {
        pts.push_back(robot.getRefPoint(nodes[i].state));
	}

	pa->setParam("depth","49");

	drawPl(pts,6,CColor("red"),pa); // bylo 2
	pts.clear();
	
	pa->setParam("depth","50");
}


template<typename RT>
void drawTrajectory2DControl(const std::vector< TNode > &nodes, RT &robot, CPainterBase *pa, const int numPointOnEdge) {

    std::vector< TPoint > pts;
    typedef typename RT::State State;

	for(int i=0;i<(int)nodes.size();i++) {
		if (nodes[i].pstate.size() != 0) {
			robot.setState(nodes[i].pstate);
			const double t = nodes[i].RRT2DControl_timeGet() / (double) numPointOnEdge;
            std::vector< State > rs(robot.control(nodes[i].input,t, numPointOnEdge));
            std::vector< TPoint > pts2( states2pts2(rs, robot) );
            std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));
		}
	}

	pa->setParam("depth","49");

	drawPl(pts,6,CColor("red"),pa); // bylo 2
	pts.clear();
	
	pa->setParam("depth","50");
}

template<typename RT>
void drawTrajectoryKRRF2DControl(const std::vector<::vector< TNode > > &nodes, RT &robot, CPainterBase *pa, bool final_path) {

    std::vector< TPoint > pts;
    typedef typename RT::State State;
	//WDEBUG("drawing KRRF control trajectory!!!! ");
	for(int i=0;i<(int)nodes.size();i++) {
		//WDEBUG("Drrawing another traj");
		for(int j=0;j<(int)nodes[i].size();j++){
			if (nodes[i][j].pstate.size() != 0) {
				const int numPointOnEdge = nodes[i][j].KRRF_edgeSamplesGet();
				robot.setState(nodes[i][j].pstate);
				const double t = nodes[i][j].KRRF_timeGet() / (double) numPointOnEdge;
				std::vector< State > rs(robot.control(nodes[i][j].input,t, numPointOnEdge));
				std::vector< State > ext{};
				ext.push_back(nodes[i][j].pstate);
				ext.insert(ext.end(), rs.begin(), rs.end());
				std::vector< TPoint > pts2( states2pts2(ext, robot) );
				std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));
				//WDEBUG("State FROM" << nodes[i][j].pstate[0] << " "  << nodes[i][j].pstate[1] << " TO " << nodes[i][j].state[0] << " "  << nodes[i][j].state[1] << " " << numPointOnEdge);
			}
		}
		pa->setParam("depth","49");
		//WDEBUG("State " << pts[0].x << " " << pts[0].y << " " << pts[1].x << " " << pts[1].y  << " " << pts[pts.size()-1].x << " " << pts[pts.size()-1].y);
		if (final_path){
			drawPl(pts,5,CColor("magenta"),pa); // bylo 2
		} else {
			drawPl(pts,2,CColor("red"),pa); // bylo 2
		}
		pts.clear();
		
		pa->setParam("depth","50");
	}
}
template<typename RT>
void drawTrajectoryKRRF2DControl(const std::vector< TNode > &nodes, RT &robot, CPainterBase *pa) {

    std::vector< TPoint > pts;
    typedef typename RT::State State;
	WDEBUG("drawing KRRF control trajectory!!!! ");
	for(int i=0;i<(int)nodes.size();i++) {
		if (nodes[i].pstate.size() != 0) {
			const int numPointOnEdge = nodes[i].KRRF_edgeSamplesGet();
			robot.setState(nodes[i].pstate);
			const double t = nodes[i].KRRF_timeGet() / (double) numPointOnEdge;
			std::vector< State > rs(robot.control(nodes[i].input,t, numPointOnEdge));
			std::vector< TPoint > pts2( states2pts2(rs, robot) );
			std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));
		}
	}

	pa->setParam("depth","49");

	drawPl(pts,6,CColor("red"),pa); // bylo 2
	pts.clear();
	
	pa->setParam("depth","50");
}

template<typename RT>
void drawTrajectoryLazyTSP2DControl(const std::vector<::vector< TNode > > &nodes, RT &robot, CPainterBase *pa) {

    std::vector< TPoint > pts;
    typedef typename RT::State State;
	WDEBUG("drawing lazy TSP control trajectory!!!! ");
	for(int i=0;i<(int)nodes.size();i++) {
		WDEBUG("Drrawing another traj");
		for(int j=0;j<(int)nodes[i].size();j++){
			if (nodes[i][j].pstate.size() != 0) {
				const int numPointOnEdge = nodes[i][j].LazyTSP_edgeSamplesGet();
				robot.setState(nodes[i][j].pstate);
				const double t = nodes[i][j].LazyTSP_timeGet() / (double) numPointOnEdge;
				std::vector< State > rs(robot.control(nodes[i][j].input,t, numPointOnEdge));
				std::vector< TPoint > pts2( states2pts2(rs, robot) );
				std::copy(pts2.begin(), pts2.end(), std::back_inserter(pts));
			}
		}
		pa->setParam("depth","49");
		drawPl(pts,6,CColor("red"),pa); // bylo 2
		pts.clear();
		
		pa->setParam("depth","50");
	}
}




template<typename ST, typename RT>
void drawTrajectory2Dtn(const std::vector< ST > &trajNodes, RT &robot, CPainterBase *pa, const int numPointOnEdge = 10) {

    std::vector< TPoint > pts( states2pts2( trajNodes, robot) );
	drawPl(pts,6,CColor("red"),pa); // bylo 2
}









template<typename RT>
void drawTrajectoryRRTDD(const std::vector< TNode  > &nodes, 
		RT &robot, CPainterBase *pa) {

    std::vector< TPoint > pts;

	for(int i=0;i<(int)nodes.size();i++) {
        pts.push_back(robot.getRefPoint(nodes[i].state));
	}

	pa->setParam("depth","49");

	drawPl(pts,6,CColor("red"),pa); // bylo 2
	pts.clear();
	
	pa->setParam("depth","50");
}

void drawPointGraph(double **mm, const std::vector<TPoint> &pts, const int drawEdges, CPainterBase *pa);


/** draw unit sphere according to given metric */
template<typename RT>
void povDrawUnitSphere(RT *robot, const int num, const typename RT::State &initState, std::ofstream &ofs) {

    typedef typename RT::State State;

    ofs << "#declare unitSphereRR=0.05;\n";

    State st;
    int trial = 0;
    const double sphereRadius = 10;
    for(int i=0;i<2*num;i++) {
        st = getRandomVectorOnBall(initState,1);
        trial = 0;
        double d = robot->distance(initState,st);
        while (fabs(d-sphereRadius) > 0.1*sphereRadius && trial < 1000) {
            const double alp = d>sphereRadius ? 0.95 : 1.05;
            for(int j=0;j<(int)st.size();j++) {
                 st[j] = alp*(st[j] - initState[j]) + initState[j];
            }    
            trial++;
            d = robot->distance(initState,st);
            WDEBUG("dist is " << d);
        }
        if (fabs(d-sphereRadius) < 0.13*sphereRadius) {
            ofs << "sphere { <" << st[0] << "," << st[1] << "," << st[2] << ">,unitSphereRR pigment { color rgb <1,0,0> }}\n";
        }
    }

}

#if 0
/** also print rotation and translation of the robot
  * draw whole robot, not using #decalration of a robot;
  */
template<typename ST>
void povDrawRobotRaw(const std::vector<TPoint3> &triangles, std::ofstream &ofs, const ST &state) {
	const double k = 180.0/M_PI;
	ofs << "mesh{ // robot\n";
	ofs << printPOVtriangle(triangles) << "\n";
	ofs << "interior {\ncaustics 1.0\nior 1.5}\n";
    ofs << "texture {\nT_Glass1\npigment { color blue 1.0 filter 0.85 }\n";
	ofs << "finish { phong 1\nphong_size 300\nreflection 0.15 }\n}\n";
	ofs << " rotate <" << state[3]*k << "," << state[4]*k << "," << state[5]*k <<">\n";
	ofs << " translate <" << state[0] <<","<<state[1] <<","<<state[2]<<">\n";
    ofs << "\n} // robot\n";	
}


/** paiting of deformable robot */
template<typename ST>
void povDrawRobotRawDeformable(const std::vector<std::vector<TPoint3> > &triangles, std::ofstream &ofs, const ST &state) {
	const double k = 180.0/M_PI;

	ofs << "mesh{ // robot\n";
	ofs << printPOVtriangleDeformable(triangles,state[6],state[7],state[8]) << "\n";
	ofs << " texture { pigment { color rgb " << povColorRaw("yellow") << "}\n";
	ofs << "           finish { diffuse 0.9 phong 0.5 } }\n";

//	ofs << "interior {\ncaustics 1.0\nior 1.5}\n";
//    ofs << "texture {\nT_Glass1\npigment { color blue 1.0 filter 0.85 }\n";
//	ofs << "finish { phong 1\nphong_size 300\nreflection 0.15 }\n}\n";

	ofs << " rotate <" << state[3]*k << "," << state[4]*k << "," << state[5]*k <<">\n";
	ofs << " translate <" << state[0] <<","<<state[1] <<","<<state[2]<<">\n";
    ofs << "\n} // robot\n";	
}

/** triangles must be deformed before calling this function */
template<typename ST>
void povDrawRobotRawDeformableBones(const std::vector< Triangle > &triangles, std::ofstream &ofs, const ST &state) {
	const double k = 180.0/M_PI;

	ofs << "mesh{ // robot\n";
	ofs << printPOVtriangles(triangles) << "\n";
/** paiting of deformable robot */
	ofs << " texture { pigment { color rgb " << povColorRaw("yellow") << "}\n";
	ofs << "           finish { diffuse 0.9 phong 0.5 } }\n";

//	ofs << "interior {\ncaustics 1.0\nior 1.5}\n";
//    ofs << "texture {\nT_Glass1\npigment { color blue 1.0 filter 0.85 }\n";
//	ofs << "finish { phong 1\nphong_size 300\nreflection 0.15 }\n}\n";

	ofs << " rotate <" << state[3]*k << "," << state[4]*k << "," << state[5]*k <<">\n";
	ofs << " translate <" << state[0] <<","<<state[1] <<","<<state[2]<<">\n";
    ofs << "\n} // robot\n";	
}
#endif

/** draw robot using #declare robot */
template<typename ST>
void povDrawNamedRobot(std::ofstream &ofs, const ST &state, const std::string &name) {
	const double k = 180.0/M_PI;
	ofs << "object {\n";
	ofs << " " << name << "\n";
	ofs << " rotate <" << state[3]*k << "," << state[4]*k << "," << state[5]*k <<">\n";
	ofs << " translate <" << state[0] <<","<<state[1] <<","<<state[2]<<">\n";
    ofs << "\n} // " << name << "\n";	
}

/** draw robot using #declare robot */
template<typename ST>
void povDrawRobot(std::ofstream &ofs, const ST &state) {
	const double k = 180.0/M_PI;
	ofs << "object {\n";
	ofs << " robot\n";
//	ofs << " pigment {checker rgb " << povColor(CColor("blue1")) << ", rgb " << povColor(CColor("green2")) << " scale 0.08}\n";
//	ofs << "interior {\ncaustics 1.0\nior 1.5}\n";
//  ofs << "texture {\nT_Glass1\npigment { color blue 1.0 filter 0.85 }\n";
//	ofs << "finish { phong 1\nphong_size 300\nreflection 0.15 }\n}\n";

	ofs << " rotate <" << state[3]*k << "," << state[4]*k << "," << state[5]*k <<">\n";
	ofs << " translate <" << state[0] <<","<<state[1] <<","<<state[2]<<">\n";
    ofs << "\n} // robot\n";	
}

/** draw robot using #declare robo
  *
  * also draw 'feature points' - sse CRobot3DPointMetric for details
  * these points are draw explicitly for verifiy, that the computation of the global coordinates of feature points is ok
  *
  * fpts are given in GLOBAL coorinates
  */
template<typename ST>
void povDrawRobotWithFeaturePoint(std::ofstream &ofs, const ST &state, const std::vector<TPoint3> &fpts) {
	const double k = 180.0/M_PI;
	ofs << "object {\n";
	ofs << " robot\n";
	ofs << " rotate <" << state[3]*k << "," << state[4]*k << "," << state[5]*k <<">\n";
	ofs << " translate <" << state[0] <<","<<state[1] <<","<<state[2]<<">\n";
    ofs << "\n} // robot\n";

    if (fpts.size() > 0) {
        for(int i=0;i<(int)fpts.size();i++) {
            povDrawPoint(fpts[i],0.1,CColor("DarkOrchid1"),ofs);
        }
    }
        
}



/** directry print states on trajectory without approxamitng them using robot's controller.
   * Appropriated for smoothed trajectory */
template<typename RT, typename RRTSTATE>
void povDeclareTrajectoryRaw(const std::vector< RRTSTATE > &nodes, RT &robot, std::ofstream &ofs, const bool noshadow=false) {

    std::vector< TPoint3 > pts;
	ofs << "// smoothed trajectory\n";
	ofs << "#declare trajectory=\n";
	ofs << "union {\n";
	for(int i=0;i<(int)nodes.size();i++) {
		pts.push_back(robot.getRefPoint(nodes[i].state));
	}
	povDrawPl(pts,"trajectoryWidth",CColor("red2"),ofs,noshadow);
	ofs << "\n}// end of trajectory\n";
}


/** directry print states on trajectory without approxamitng them using robot's controller.
   * Appropriated for smoothed trajectory */
template<typename RT, typename RRTSTATE>
void povDeclareTrajectoryRawCDR(const std::vector< RRTSTATE > &nodes, RT &robot, std::ofstream &ofs, const bool noshadow=false) {

	ofs << "// smoothed trajectory\n";
	ofs << "#declare trajectory=\n";
	ofs << "union {\n";
    for(int j=0;j<j<robot.getNumRobots();j++) {
        std::vector< TPoint3 > pts;
    	for(int i=0;i<(int)nodes.size();i++) {
	    	pts.push_back(robot.getRefPoint(nodes[i].state,j));
    	}
	    povDrawPl(pts,"trajectoryWidth",CColor("red2"),ofs,noshadow);
    }
	ofs << "\n}// end of trajectory\n";
}




/** draw robot trajectory */
template<typename RT>
void povDeclareTrajectoryNodes(const std::vector< TNode > &nodes, RT &robot, std::ofstream &ofs) {

    std::vector< TPoint3 > pts;

//    CColor color("red2");
	ofs << "#declare trajectory=\n";
	ofs << "union {\n";
	for(int i=0;i<(int)nodes.size();i++) {
        if (nodes[i].state.size() != 0) {
            pts.push_back(robot.getRefPoint(nodes[i].state));
        }
	}
    povDrawLineMacro(pts,"traj",ofs);
	ofs << "\n}// end of trajectory\n";
}


/** draw robot trajectory */
template<typename State>
void povDeclareTrajectoryStates(const std::vector< State > &traj, CRobot3D &robot, std::ofstream &ofs) {

    std::vector< TPoint3 > pts;

 //   CColor color("red2");
	ofs << "#declare trajectory=\n";
	ofs << "union {\n";
	for(int i=0;i<(int)traj.size();i++) {
        pts.push_back(robot.getRefPoint(traj[i]));
	}
    povDrawLineMacro(pts,"traj",ofs);
	ofs << "\n}// end of trajectory\n";
}








void povDeclareConvexHull(const std::vector< std::vector<TPoint3> > &triangles, std::ofstream &ofs, const std::string &name);





int xyz2idx(const TPoint3 &p, const std::vector<double> &mapDimension, const double resolution,  const int nx, const int ny, const int nz );




/* print rrt tree to povray file, assuming that RRT3DStar_getCost is working for the nodes */
template<typename RRTT, typename RT>
void povDeclareRRTNodesWithCost(const  RRTT &rrt, RT &robot, std::ofstream &ofs,const bool noshadow=false, const bool intermediatePoints=true) {


	ofs << "#declare rrt=\n";
	ofs << "union {\n";

    double minCost = -1;
    double maxCost = -1;

    const std::vector< TNode *> &tree(rrt.getTree());
    
	for(int i=0;i<(int)tree.size();i++) {
        const double cost = tree[i]->RRT3DStar_getCost();
        if ((cost >= 0 && cost < minCost ) || minCost == -1) {
            minCost = cost;
        }
        if (cost > maxCost) {
            maxCost = cost;
        }
    }

    char name[50];

    CColorMap cm;
    cm.setRange(minCost, maxCost);

    std::vector< TPoint3 > pts;

	for(int i=0;i<(int)tree.size();i++) {
		if (tree[i]->pstate.size() != 0) {
            pts.push_back(robot.getRefPoint(tree[i]->pstate));
            pts.push_back(robot.getRefPoint(tree[i]->state));

            CColor col(cm.getColor(tree[i]->RRT3DStar_getCost()) );
            snprintf(name,sizeof(name), "rgb %s",col.getPovColorRGB().c_str());
            
            povDrawTreeSegments(pts,name,ofs,noshadow);
			pts.clear();
		}
	}
	ofs << "}// end of rrt\n";
	pts.clear();
}




void povDraw3DNodesColor( const std::vector< TNode *> &tree,  CRobot3D *robot, std::ofstream &ofs, const char *colorName);



template<typename RT>
void povDraw3DNodes(const std::vector< TNode *> &tree, RT &robot, 
        std::ofstream &ofs,const bool noshadow=false, const bool intermediatePoints=true) {
    std::vector< TPoint3 > pts;
    for(int i=0;i<(int)tree.size();i++) {
        if (tree[i]->pstate.size() != 0) {
            pts.push_back(robot.getRefPoint(tree[i]->pstate));
            pts.push_back(robot.getRefPoint(tree[i]->state));
            povDrawTreeSegments(pts,"treeColor",ofs,noshadow);
            pts.clear();
        }
    }
}


template<typename RT>
void povDeclareRRTNodes(const vector<TNode *> &tree, RT &robot, std::ofstream &ofs,const bool noshadow=false, const bool intermediatePoints=true) {
	ofs << "#declare rrt=\n";
	ofs << "union {\n";
    povDraw3DNodes(tree,robot, ofs, noshadow, intermediatePoints);    
    ofs << "}// end of rrt\n";
}



template<typename RRTT, typename RT>
void povDeclareRRT_BidirectNodes(const  RRTT &rrt, RT &robot, std::ofstream &ofs) {

    ofs << "#declare rrt=\n";
    ofs << "union {\n";

    povDraw3DNodes(rrt.getTreeFwd(), robot, ofs);    
    povDraw3DNodes(rrt.getTreeBack(),robot, ofs);    

    ofs << "}\n";
}








void draw3Map(CPainter3D *pa, const std::vector< std::vector<TPoint3 > > &tr);



template<typename RT, typename ST>
void draw3Robot(CPainter3D *pa, RT &robot, const ST &state, const CColor &color) {

	if (pa) {
		double rmatrix[3][3];
		double tvector[3];
		RAPID_model *mmm = robot.getRapidModel(rmatrix,tvector,state);
		std::vector< std::vector< TPoint3 > > tr(robot.getTriangles());
		for(int i=0;i<(int)tr.size();i++) {
			translate(tr[i][0],rmatrix,tvector);
			translate(tr[i][1],rmatrix,tvector);
			translate(tr[i][2],rmatrix,tvector);
			pa->drawTriangle(tgp3(tr[i][0]),tgp3(tr[i][1]),tgp3(tr[i][2]),color);
		}
		tr.clear();
        mmm = NULL;
	}
}


/* draws a trajectory which was previously smoothed - hence there is no valid input informations. The
   * trajectory is painter as a simple polyline .. */
template<typename RT>
void draw3SmoothTrajectory(CPainter3D *pa, const std::vector< RRTState<typename RT::State, typename RT::Input> > &nodes, 
		RT &robot, const int numPointOnEdge = 10) {

	if (pa) {
    	std::vector< TPoint3 > pts;
		for(int i=0;i<(int)nodes.size();i++) {
			pts.push_back(robot.getRefPoint(nodes[i].state));
		}
		pa->drawLine(tgp3(pts),0.02,CColor("red2"));
	}
}

void draw3Surface(CPainter3D *pa, const std::vector<TPoint3> &pts, const int indicesSize, int *indices);


/** draws PRM.
  * edgeType: 0 .. no edges
  *           1 .. straight lines (just connects states)
  *           2 .. trajectories according to inputs
  */
template<typename PRMT, typename RT>
void drawPrm(PRMT &prm, RT &robot, CPainterBase *pa, const int edgeType) {
	typedef typename PRMT::PState PState;

	if (pa) {
		std::vector<PState> pstates(prm.getPrmStates());

		std::vector<TPoint> pts;
        pts.reserve(pstates.size());

		for(int i=0;i<(int)pstates.size();i++) {
			pts.push_back(robot.getRefPoint(pstates[i].state));	
		}
		drawPts(pts,1,1,CColor("blue"),CColor("blue"),pa);

		pts.clear();
        pts.reserve(2);
		if (edgeType == 1) {
			for(int i=0;i<(int)pstates.size();i++) {
				for(int j=0;j<(int)pstates[i].edges.size();j++) {
					pts.push_back(robot.getRefPoint(pstates[i].state));
					pts.push_back(robot.getRefPoint(pstates[pstates[i].edges[j].end].state));
					drawPl(pts,1,CColor("green3"),pa);
					pts.clear();
				}
			}		

		} else if (edgeType == 2) {

		}
		pstates.clear();
	}
}


/** draws PRM.
  status = -1 -> all edges, otherwise only edges with given status
  */
template<typename PRMT, typename RT>
void povDrawPrm(PRMT &prm, RT &robot, std::ofstream &ofs, const int drawPts, const int drawEdges, const int whichStatus = -2) {
    typedef typename PRMT::PState PState;

    std::vector<PState> pstates(prm.getPrmStates());

    ofs << "#declare prm=union {\n";
    ofs << "// num nodes=" << pstates.size() << ", drawPts=" << drawPts << ", drawEdges=" << drawEdges << "\n";
    if (drawPts > 0) {
        std::vector<TPoint3> pts;
        for(int i=0;i<(int)pstates.size();i++) {
            pts.push_back(robot.getRefPoint(pstates[i].state));	
        }
        povDrawPts(pts.begin(), pts.end(),"prmPoint",CColor("yellow"),ofs);
        pts.clear();
    }
    ofs << "//prm edges\n";
                

    if (drawEdges > 0) {
        std::vector<TPoint3> pts;
        for(int i=0;i<(int)pstates.size();i++) {
            for(int j=0;j<(int)pstates[i].edges.size();j++) {
                pts.push_back(robot.getRefPoint(pstates[i].state));
                pts.push_back(robot.getRefPoint(pstates[pstates[i].edges[j].end].state));
                const int status = pstates[i].edges[j].status;
                if (1 || whichStatus == -2 || status == whichStatus) {
                    if (status == 0) {
                        povDrawPl2(pts,"prmEdgeWidth","prmEdgeColorNone",ofs);
                    } else if (status == 1) {
                        povDrawPl2(pts,"prmEdgeWidth","prmEdgeColor",ofs);
                    } else if (status == -1) {
                        povDrawPl2(pts,"prmEdgeWidth","prmEdgeColorFalse",ofs);
                    }
                }
                pts.clear();
            }
        }		
    }
    ofs << "} // end of prm\n";
    pstates.clear();
}





template<typename ST, typename RT>
void drawPRMTraj(const std::vector< ST > &states, RT &robot, CPainterBase *pa) {
	if (pa) {
        std::vector<TPoint> pts;
        pts.reserve(states.size());
		for(int i=0;i<(int)states.size();i++) {
			pts.push_back(robot.getRefPoint(states[i]));
		}	
		drawPl(pts,3,CColor("blue1"),pa);
		pts.clear();
	}
}



void raw_draw_cylinder(const TPoint3 &p1in, const TPoint3 &p2in, const double radius, const int resolution, ofstream &ofs);
void raw_drawPl(const vector<TPoint3> &pts, ofstream &ofs, const double radius);




} // namespace


#endif

