
#ifndef __painting_h__
#define __painting_h__

#include <list>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "gui/CPainters.h"
#include "types.h"
//#include "map.h"
#include "mapr.h"
#include "mesh.h"

#include "rrt2Bidirect.h"

#include "CRobot3D.h"
#include "rrtstate.h"


namespace rrtPlanning {

using std::list;
using namespace CPainters;

typedef enum _TPainter {
	GUI = 0,
	PNG,
	PDF,
	XFIG,
    GNUPLOT
} TPainter;



gPoint tgp(const TPoint &p, const bool resizeable=false);


void writeLights(const vector<TLight> &lights, std::ofstream &ofs);
void writeMacros(std::ofstream &ofs);

void povDrawPlTrajSegment(const std::vector<TPoint3> &pts, std::ofstream &ofs, const int index);

CPainterBase *initPainter(enum _TPainter type, const std::vector<double> &dimension, const char *fileMask, 
		CPainterBase *dump=NULL,const double scale=1.0);
		
std::string printPainterHelp();

void createCylinder(std::vector< std::vector< TPoint3 > > &cyl, const double length, const double radius, const int n);

std::string dbl2str(const double d);


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

//void drawMap(const Map &m, CPainterBase *pa);
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
			pts.push_back(robot.getRefPoint(node->pstate));
			pts.push_back(robot.getRefPoint(node->state));
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



} // namespace


#endif

