#include "types.h"
#include "mapr.h"
#include "mesh.h"
#include "krrf.h"
#include "CParseArgs.h"
#include "timerN.h"
#include "problems2D.h"
#include "options2D.h"



#include "libs/op/dataset_loader_obst.h"
extern "C" {
#include "libs/triangle/triangle.h"

}

#include <sys/types.h>
#include <unistd.h>

#include "CRobotDiff.h"
#include "CRobot2D.h"
#include "CRobotCarLike.h"
#include "CRobotBike.h"
#include "ut.h"
#include "polygonUtils.h"
#include "Voronoi.h"
#include "CStat.h"

#include "libs/gui/CPainterBase.h"
#include "libs/gui/CPainterGui.h"
#include "libs/gui/CPainterCairo.h"
#include "libs/gui/CPainterDump.h"
#include "libs/gui/CColorMap.h"
#include "painting.h"
#include "WLog.h"
#include "rapid/RAPID.H"


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
#include <utility>
#include <fstream>
#include <typeinfo>
#include <iterator>
#include <map>


using namespace rrtPlanning;
using namespace std;
using namespace CPainters;


typedef CRobotCarLike TGlobalRobot;
// typedef CRobotDiff TGlobalRobot;
// typedef CRobotBike TGlobalRobot;



#define intro counter++; if (id == -1) std::cerr << counter << " :"<< __FUNCTION__ << "\n"; if (id != counter) return; else std::cerr << __PRETTY_FUNCTION__ << "\n";
static int counter = 0;

static void initLogger(const char *filename, int argc, char **argv) {
    char logName[2000];
    if (filename) {
        snprintf(logName,sizeof(logName),"%s.pid-%05d.log",filename,getpid());
    } else {
        snprintf(logName,sizeof(logName),"_prot.pid-%05d.log",getpid());
    }
    __wlog = new _WLog(logName,cerr);

    for(int i=0;i<argc;i++) {
        WDEBUG("Argv[" << i << "]: " << argv[i]);
    }   
}





template<typename ST, typename RT, typename MP>
void printInitGoalStatus(const ST &initState, const ST &goalState,
		RT *robot, MP *map, std::ostream &os = std::cerr) {

	os << "Init state: ";
	print(initState," "," ",os);
	if (map->isFreePts(robot->getShapePoints(initState)))
		os << "pts:free ";
	else
		os << "pts:non-free! ";

	if (map->isFreePolygons(robot->getShape(initState))) {
		os << "polygons:free";
	} else {
		os << "polygons:non-free!";
	}

	os << ", validState: "; 
	if (robot->isValidState(initState)) {
		os << "true\n";
	} else {
		os << "false\n";
	}

	os << "GoalState: ";
	print(goalState," "," ",os);
	if (map->isFreePts(robot->getShapePoints(goalState))) {
		os << "pts:free";
	} else {
		os << "pts:non-free!";
	}

	if (map->isFreePolygons(robot->getShape(goalState))) {
		os << "polygons:free, ";
	} else {
		os << "polygons:non-free!, ";
	}

	os << ", validState: ";
	if (robot->isValidState(goalState)) {
		os << "true\n";
	} else {
		os << "false\n";
	}

}

/** return true if robot is in collision with some obstacle in a map
  * collision is detcted using RAPID library, so both map and
  * robot must provide getRapidModel method
  */
template<typename RT, typename ST>
inline bool isCollisionRapid(MapR *map, RT *robot, const ST &state) {

	double rmap[3][3], vmap[3], rrobot[3][3], vrobot[3];
	RAPID_model *mmap = map->getRapidModel(rmap,vmap);
	RAPID_model *mrobot = robot->getRapidModel(rrobot,vrobot,state);

	RAPID_Collide(rmap,vmap,mmap,rrobot,vrobot,mrobot);
	return RAPID_num_contacts != 0;
}


template<typename ST, typename RT>
void printInitGoalStatusRapid(const ST &initState, const ST &goalState,
		RT *robot, MapR *map, std::ostream &os = std::cerr) {

    WDEBUG("Init state " << printString(initState) << " isColision? " << isCollisionRapid(map, robot, initState));
    WDEBUG("Goal state " << printString(goalState) << " isColision? " << isCollisionRapid(map, robot, goalState));
	
}


template<typename ST, typename RT>
std::string printInitGoalStatusRapidString(const ST &initState, const ST &goalState, RT *robot, MapR *map) {

	stringstream os;
	os << "Init state: ";
	printString(initState," "," ",os);
	
	if (!isCollisionRapid(map,robot,initState)) {
		os << "shape:free ";
	} else {
		os << "shape:non-free! ";
	}

	os << ", validState: "; 
	if (robot->isValidState(initState)) {
		os << "true\n";
	} else {
		os << "false\n";
	}

	os << "GoalState: ";
	printString(goalState," "," ",os);
	
	if (!isCollisionRapid(map,robot,goalState)) {
		os << "shape:free";
	} else {
		os << "shape:non-free!";
	}

	os << ", validState: ";
	if (robot->isValidState(goalState)) {
		os << "true\n";
	} else {
		os << "false\n";
	}
	return os.str();
}



template<typename RobotT, typename RRT, typename ST>
void trajectoryInfoTNode(RobotT *robot, RRT &rrt,const ST  &initState, const ST &goalState,
	   double &distanceToGoal, double &pathTimeLength, double &pathMetricLength	) {

	typedef TPoint Point;
	vector<typename RRT::TreeNode> nodes(rrt.getTrajectory(initState,goalState));

	if (nodes.size() < 1) {
		distanceToGoal = robot->distance(initState,goalState);
		pathTimeLength = -1;
		pathMetricLength = -1;
		return;
	}

	distanceToGoal = robot->distance(goalState, nodes.back().state);
	pathTimeLength = nodes.size();

	const int numPoint = 4;
	double t;
	vector<Point> pts;
	pts.reserve((nodes.size()+1) * numPoint);

	for(int i=0;i<(int)nodes.size();i++) {
	
		if (nodes[i].pstate.size() != 0) {
			robot->setState(nodes[i].pstate);
			t = nodes[i].time / (double) numPoint;
			vector<typename RobotT::State> states(robot->control(nodes[i].input,t,numPoint));
			pts.push_back(robot->getRefPoint(nodes[i].prevState));
			for(int j=0;j<(int)states.size();j++) {
				pts.push_back(robot->getRefPoint(states[j]));
			}
			states.clear();
		}	
	}
	pathMetricLength = 0;
	for(int i=0;i<(int)pts.size()-1;i++) {
		pathMetricLength += pointDistanceEucleid(pts[i],pts[i+1]);
	}
	pts.clear();
	nodes.clear();
}


template<typename RobotT, typename RRT, typename ST>
void trajectoryInfo(RobotT *robot, RRT &rrt,const ST  &initState, const ST &goalState,
	   double &distanceToGoal, double &pathTimeLength, double &pathMetricLength	) {
}





template<typename State, typename TNode, typename TRobot>
void trajectoryInfo(TRobot *robot, vector<TNode> &traj, CStat &stat, State &initState, State &goalState) {
        

	typedef TPoint Point;

    stat["distanceToGoal"] = -1;
    stat["distanceToInit"] = -1;
    stat["pathMetricLength"] = -1;
    stat["trajSize"] = traj.size();

    if (traj.size() > 0) {
    	stat["distanceToGoal"] = robot->distance(goalState, traj.back().state);
        stat["distanceToInit"] = robot->distance(initState, traj.back().state);

	    double pathMetricLength = 0;
        double path3DLength = 0;
    	for(int i=0;i<(int)traj.size()-1;i++) {
    		path3DLength += pointDistanceEucleid(robot->getRefPoint(traj[i].state),robot->getRefPoint(traj[i+1].state));
            pathMetricLength += sqrt(robot->distance(traj[i].state, traj[i+1].state));
        }
        stat["pathMetricLength"] = pathMetricLength;
        stat["path3DLength"] = path3DLength;
        WDEBUG("INfo Length " << pathMetricLength);
	}
}

template<typename RT, typename ST>
void traj2nodes(const std::vector< ST > &traj, RT &robot, vector< TNode > &nodes) {

	typedef typename RT::State State;
    nodes.clear();
    
    nodes.reserve(traj.size());
    for(int i=0;i<(int)traj.size(); i++) {
        nodes.push_back(TNode());
        nodes.back().KRRF_setDefault();
        nodes.back().state = traj[i];
        if (i > 0) {
            nodes.back().pstate = traj[i-1];
        } else {
            nodes.back().pstate = nodes.back().state;
        }
    }
}



template<typename RT, typename ST>
void nodes2traj(const std::vector< TNode > &nodes, RT &robot, vector<ST> &trajectory) {

	typedef typename RT::State State;
	//WDEBUG("getTrajectorStates: point on edge = " << numPointOnEdge << ", nodes.size=" << nodes.size());
	//if (nodes.size() > 1) {
	//	WDEBUG("edge time " << nodes[0].RRT2DControl_timeGet());
	//	WDEBUG("states per second = " << (1.0 / nodes[0].RRT2DControl_timeGet())*numPointOnEdge);
	//}

	std::vector<State> rs;
    trajectory.clear();
    trajectory.reserve(nodes.size());

    const int ssize = robot.getStateSize();

	for(int i=0;i<(int)nodes.size();i++) {
        trajectory.push_back(nodes[i].state);
	}
}
template<typename RT, typename ST>
void nodes2trajs(const std::vector<std::vector< TNode > > &nodes, RT &robot, vector<vector<ST>> &trajectory) {

	typedef typename RT::State State;
	//WDEBUG("getTrajectorStates: point on edge = " << numPointOnEdge << ", nodes.size=" << nodes.size());
	//if (nodes.size() > 1) {
	//	WDEBUG("edge time " << nodes[0].RRT2DControl_timeGet());
	//	WDEBUG("states per second = " << (1.0 / nodes[0].RRT2DControl_timeGet())*numPointOnEdge);
	//}

	//std::vector<State> rs;
    trajectory.clear();
    //vector<vector<ST>> trajectory{nodes.size(), vector<ST>(nodes.size(), 0)};//.reserve(nodes.size());
    
    const int ssize = robot.getStateSize();
    vector<ST> subtraj;
    for(int j=0;j<(int)nodes.size();j++){
        for(int i=0;i<(int)nodes.size();i++) {
            subtraj.push_back(nodes[j][i].state);
        }
        WDEBUG("Next Subtraj" << j << "/" << nodes.size());
        trajectory.push_back(subtraj);
        subtraj.clear();
    }
}



template<typename RT, typename TTreeNode>
std::list<typename RT::State> getTrajectoryStates(const std::vector< TTreeNode > &nodes, RT &robot, const int pointOnEdge = 5) {

	typedef typename RT::State State;
	double  time;
	WDEBUG("getTrajectorStates: point on edge = " << pointOnEdge);
	if (nodes.size() > 1) {
		WDEBUG("edge time " << nodes[1].time);
		WDEBUG("states per second = " << (1.0 / nodes[1].time)*pointOnEdge);
	}

	std::list<State> trajectoryStates;
	std::vector<State> rs;

	for(int i=0;i<(int)nodes.size();i++) {

		if (nodes[i].prevState.size() != 0) {

			robot.setState(nodes[i].prevState);

			time = nodes[i].time / (double) pointOnEdge;

			rs = robot.control(nodes[i].input, time, pointOnEdge);

			std::copy(rs.begin(),rs.end(),std::back_inserter(trajectoryStates));
			rs.clear();
		}
	}

	return trajectoryStates;
}


template<typename RT, typename ST>
void getTrajectoryStatesTNode(const std::vector< TNode > &nodes, RT &robot, const int pointOnEdge, vector<ST> &result) {
	WDEBUG("getTrajectorStates: point on edge = " << pointOnEdge);
	if (nodes.size() > 1) {
		WDEBUG("edge time " << nodes[1].RRTDD_timeGet());
		WDEBUG("states per second = " << (1.0 / nodes[1].RRTDD_timeGet())*pointOnEdge);
	}

	std::vector<ST> rs;

    result.clear();
    result.reserve(nodes.size()*(pointOnEdge+1));

	for(int i=0;i<(int)nodes.size();i++) {
        result.push_back(nodes[i].state);
        /*
		if (nodes[i].pstate.size() != 0) {
			robot.setState(nodes[i].pstate);
			const double time = nodes[i].RRTDD_timeGet() / (double) pointOnEdge;
			rs = robot.control(nodes[i].input, time, pointOnEdge);
			std::copy(rs.begin(),rs.end(),std::back_inserter(result));
			rs.clear();
		}
        */
	}
}

template<typename ST>
void loadAllSGStates(const char *filename, const int stateSize, vector<ST> &initStates, vector<ST> &goalStates) {
    initStates.clear();
    goalStates.clear();

    ifstream ifs(filename);
    while(ifs) {
        string l1,l2;
        std::getline(ifs,l1);
        std::getline(ifs,l2);
        vector<double> a(lineToNum<double>(l1));
        vector<double> b(lineToNum<double>(l2));
        if ((int)a.size() == stateSize && (int)b.size() == stateSize) {
            initStates.push_back(a);
            goalStates.push_back(b);
        }
    }
    ifs.close();
}


/** computes info about coverage of the configuration space using the stat/goals and distanceToGoalTh = dtg.
  sg are loaded from sgFile, output is written to filename */
template<typename RT, typename MT>
void getSGInfo(const char *sgFile, RT &robot, MT &map, const double dtg, const char *filename) {
    
    typedef typename RT::State ST;

    vector<ST> initStates, goalStates;
    loadAllSGStates(sgFile, robot.getStateSize(), initStates,goalStates);

    const vector<double> dim(map.getDimension().getVector());

    double area = (dim[1] - dim[0])*(dim[3] - dim[2]);

    const int numSamples = 500000;

    double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
    RAPID_model *mmap = map.getRapidModel(rMap,vMap);
    RAPID_model *mrobot;
    int freeSamples = 0;
    int obstacleSamples = 0;
    int inInitSamples = 0;
    int inGoalSamples = 0;
    
    for(int i=0;i<numSamples;i++) {
        ST rs(robot.getRandomState(dim));
        mrobot = robot.getRapidModel(rRobot,vRobot,rs);
        RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
        bool f = (RAPID_num_contacts == 0);
        if (f) {
            freeSamples++;
        } else {
            obstacleSamples++;
        }

        // could be replced by kd-tree, this is only fortesting
        for(int j=0;j<(int)initStates.size();j++) {
            if (robot.distance(initStates[j],rs) < dtg) {
                inInitSamples++;
                break;
            }
        }
        for(int j=0;j<(int)goalStates.size();j++) {
            if (robot.distance(goalStates[j],rs) < dtg) {
                inGoalSamples++;
            }
        }
    }

    const double freeAreaRatio = 1.0*freeSamples/(double)numSamples;
    const double obstacleAreaRatio = 1.0*obstacleSamples/(double)numSamples;

    const double initAreaRatio = 1.0*inInitSamples/(double)numSamples;
    const double initFreeAreaRatio = 1.0*inInitSamples/(double)freeSamples;

    const double goalAreaRatio = 1.0*inGoalSamples/(double)numSamples;
    const double goalFreeAreaRatio = 1.0*inGoalSamples/(double)freeSamples;

    ofstream ofs(filename);
    ofs << "dtg: " << dtg << "\n";
    ofs << "num-samples: " << numSamples << "\n";
    ofs << "area: " << area << "\n";
    ofs << "free-area: " << freeAreaRatio*area << "\n";
    ofs << "free-area-ratio: " << freeAreaRatio << "\n";
    ofs << "nonfree-area: " << obstacleAreaRatio*area << "\n";
    ofs << "nonfree-area-ratio: " << obstacleAreaRatio << "\n";
    ofs << "init-area: " << initAreaRatio*area << "\n";
    ofs << "init-area-ratio: " << initAreaRatio << "\n";
    ofs << "goal-area: " << goalAreaRatio*area << "\n";
    ofs << "goal-area-ratio: " << goalAreaRatio << "\n";
    ofs << "init-free-area: " << initFreeAreaRatio*freeAreaRatio*area << "\n";
    ofs << "init-free-ratio: " << initFreeAreaRatio << "\n";
    ofs << "goal-free-area:"  << goalFreeAreaRatio*freeAreaRatio*area << "\n";
    ofs << "goal-free-ratio: " << goalFreeAreaRatio << "\n";
    ofs << "num-init-states: " << initStates.size() << "\n";
    ofs << "num-goal-states: " << goalStates.size() << "\n";
    ofs.close();
}


template<typename ST>
bool loadFromSGFile(const char *sgfile, const int sgindex, const int stateSize, ST &initState, ST &goalState) {
    ifstream ifs(sgfile);
    int i = 0;
    bool wasLoaded = false;
    while(ifs) {
        string l1,l2;
        std::getline(ifs,l1);
        std::getline(ifs,l2);
        if (i == sgindex) {
            vector<double> vd1(lineToNum<double>(l1));
            vector<double> vd2(lineToNum<double>(l2));
            if ((int)vd1.size() == stateSize && (int)vd2.size() == stateSize) {
                initState = vd1;
                goalState = vd2;
                wasLoaded = true;
            } else {
                WDEBUG("Cannot load sg-point from " << sgfile << " sgindex="<<sgindex << ", because vd1.size=" << vd1.size() <<
                        ", vd2.size=" << vd2.size() << ", but robot.statesize=" << stateSize);
                exit(0);
            }
            break;
        }
        i++;
    }
    return wasLoaded;
}


template<typename T>
struct myLessVD {
	bool operator()(const T &a, const T &b) {
		if (a.x < b.x)  {
			return true;
		}

		if (a.x > b.x) {
			return false;
		}

		if (a.y < b.y) return true;

		return false;
	}
};


/** speed  = size_of_new_oanted_tree / size_of_old_painted_tree, 
 e.g.
      speed=1.1 defines that each painted frame differs in 10% of tree size
  */
// assuming CRobot32D
template<typename ST>
static void drawTreeGrowingProcess( const vector< TNode *> &tree, CPainterBase *pa, MapR &map, CRobot2D &robot,
        const ST &initState, const ST &goalState, const double speed) {

    const int numPointOnEdge = 5;

    if (pa) {
        int lastPainted = -1;
        vector<TPoint> pts;
        for(int j=1; j < (int)tree.size();j++) {
            if ((( (double)(j - lastPainted) / (double)lastPainted) >= speed) || lastPainted == -1 || j == (int)tree.size()-1) {

                pa->begin();
                drawMap(map,pa);

                for(int i=0;i<j;i++) {
                    TNode *node = tree[i];
                    if (node->pstate.size() != 0) {
                        pts.push_back(robot.getRefPoint(node->pstate));
                        pts.push_back(robot.getRefPoint(node->state));

                        /*robot.setState(node->pstate);
                        const double t = node->RRT2D_time() / (double) numPointOnEdge;
                        pts.push_back(robot.getRefPoint(node->pstate));
                        std::vector<ST> rs(robot.control(node->input,t,numPointOnEdge));
                        for(int j=0;j<(int)rs.size();j++) {
                            pts.push_back(robot.getRefPoint(rs[j]));
                        }
                        */
                        drawPl(pts,2,CColor("green4"),pa);
                        pts.clear();

                    }
                }



                draw(robot.getRefPoint(initState),4,1,CColor("black"),CColor("blue"),pa);
                draw(robot.getRefPoint(goalState),4,1,CColor("black"),CColor("cyan"),pa);
                pa->close();
                lastPainted = j;
            }

        }
    }

}


/** animating robot moving along the trajectory */
template<typename ST, typename RRTT, typename RT>
static void animateTrajectory( RRTT &rrt, const vector <TNode> &nodes, const vector< ST > &traj, CPainterBase *pa, MapR &map, RT &robot,
        const ST &initState, const ST &goalState) {
    if (pa) {
        for(int i=0;i<(int)traj.size();i++) {
            pa->begin();
            drawMap(map,pa);
            drawRRT_2DControl(rrt,robot,pa, rrt.getEdgeSamples());
            drawTrajectory2DControl(nodes,robot,pa, rrt.getEdgeSamples());
            drawRobot(robot.getShape(traj[i]),pa);
            draw(robot.getRefPoint(initState),5,1,CColor("black"),CColor("blue"),pa);
            draw(robot.getRefPoint(goalState),5,1,CColor("black"),CColor("cyan"),pa);
            pa->close();
        }
    }
}


/** animating robot moving along the trajectory */
template<typename ST, typename RRTT>
static void animateTrajectory( RRTT &rrt, const vector <ST> &trajNodes, const vector< ST > &traj, CPainterBase *pa, MapR &map, CRobot2D &robot,
        const ST &initState, const ST &goalState) {
    if (pa) {
        for(int i=0;i<(int)traj.size();i++) {
            pa->begin();
            drawMap(map,pa);
            drawRRT_2D(rrt,robot,pa);
            drawTrajectory2Dtn(trajNodes,robot,pa,rrt.getEdgeSamples());
            drawRobot(robot.getShape(traj[i]),pa);
            draw(robot.getRefPoint(initState),5,1,CColor("black"),CColor("blue"),pa);
            draw(robot.getRefPoint(goalState),5,1,CColor("black"),CColor("cyan"),pa);
            pa->close();
        }
    }
}

template<typename S, typename RT>
void read_states(std::vector<S> &init_states, const char *filename, RT &robot){
    std::ifstream ifs(filename);
	
	string line;
    WDEBUG("Reading targets");
	while(ifs) {
		getline(ifs,line);
        WDEBUG(line);
		std::vector<double> vd(lineToNum<double>(line));
        if(vd.size() > 0){
            S initState(robot.getState());
            initState[0] = vd[0];
            initState[1] = vd[1];
            initState[2] = vd[2];
            init_states.push_back(initState);
        }
		vd.clear();
	}
	ifs.close();
    WDEBUG("Targets read")
}

void dist2TSPfile(vector<double> lengths, int n_states, char * prefix){
    stringstream ss;
    std::string str(prefix);
    cout << "writing file " << prefix << " with tsp !!!! \n\n\n";

    char nameTSP[2000];
    snprintf(nameTSP, sizeof(nameTSP), "%s.tsp", prefix);
    {
        ofstream ofs(nameTSP);
        ofs << "NAME : KRRF\nTYPE : TSP\nDIMENSION : " << n_states << "\nEDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: UPPER_ROW\nEDGE_WEIGHT_SECTION\n";
        int k=0;
        for (int i=0; i<n_states-1; i++){
            for (int j=i+1; j<n_states; j++){
                ofs << (int)(10*lengths[k]) << " ";  // this is because EDGE_WEIGHT_SECTION must contains integers
                k++;
            }
            ofs << "\n";
        }
        ofs.close();
    }

    char namePAR[2000];
    snprintf(namePAR,sizeof(namePAR), "%s.par", prefix);
    {
        ofstream ofs(namePAR);
        ofs << "PROBLEM_FILE = " << prefix << ".tsp" << "\nOUTPUT_TOUR_FILE = " << prefix << "_out.txt";
    }
}

void TSPresultRead(int n_states, vector<int> &order, char * prefix){
    std::string str(prefix);
    std::string res = str + "_out.txt";
    ifstream ifs(res.c_str());
    string line;
    
    // First line contains column names, skip it
    for (int i=0; i<6; i++){
        std::getline(ifs, line);
    }
    for (int i=0; i<n_states; i++){
        std::getline(ifs, line);
        order.push_back(std::stoi(line)-1);
    }
}

template<typename RT, typename MT, typename ST>
void generateRandomStartGoals(RT &robot, MT &map, std::vector<ST> &initStates, const int number,
        const double radius) {
    typedef ST State;

    initStates.clear();
    
    double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
    RAPID_model *mmap = map.getRapidModel(rMap,vMap);
    RAPID_model *mrobot;
            
    for(int i=0;i<number;i++) {
        bool f2 = true;
        State init;
        do {
            bool f = true;
            do {
                init = robot.getRandomState(map.getDimension());
                mrobot = robot.getRapidModel(rRobot,vRobot,init);
                RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
                f = (RAPID_num_contacts == 0);
            } while (!f);

            // now try to little bit move the init state around

            f2 = true;
            for(int j=0;j<20 && f2; j++) {
                const double angle = j*2*M_PI/20.0;
                State tmp(init);
                tmp[0] += radius*cos(angle);
                tmp[1] += radius*sin(angle);
                mrobot = robot.getRapidModel(rRobot,vRobot,tmp);
                RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
                f2 = (RAPID_num_contacts == 0);
            }
        } while (!f2);
        initStates.push_back(init);
    }
    WDEBUG("Generated " << initStates.size() << " init");
    WDEBUG("Used sgRadius: " << radius);
    
}

template<typename ST>
void saveStartGoalStates(const std::vector<ST> &initStates, const char *filename) {
    if (initStates.size() == 0) {
        WDEBUG("Cannot save start/goal states, as initStates.size=" << initStates.size());
        exit(0);
    }
    ofstream ofs(filename);
    for(int i=0;i<initStates.size();i++) {
        ofs << printString(initStates[i]) << "\n";
    }
    ofs.close();
}
template<typename RT>
void outputPointsToFile(const std::vector<::vector< TNode > > &nodes, RT &robot, const char *filename) {
    WDEBUG("Printing final route to file.");
    typedef typename RT::State State;
    std::vector< State > rs;
    ofstream ofs(filename);
    for(int i=0;i<(int)nodes.size();i++) {
        for(int j=0;j<(int)nodes[i].size();j++){
            if (nodes[i][j].pstate.size() != 0) {
                const int numPointOnEdge = nodes[i][j].KRRF_edgeSamplesGet();
                robot.setState(nodes[i][j].pstate);
                const double t = nodes[i][j].KRRF_timeGet() / (double) numPointOnEdge;
                std::vector< State > rs2(robot.control(nodes[i][j].input,t, numPointOnEdge));
                //std::vector< TPoint > pts2( states2pts2(rs, robot) );
                std::copy(rs2.begin(), rs2.end(), std::back_inserter(rs));
                //WDEBUG("State " << nodes[i][j].state[0] << " "  << nodes[i][j].state[1] << " " << nodes[i][j].pstate[0] << " "  << nodes[i][j].pstate[1]);
            }
        }
        for (int j=0; j<(int)rs.size(); j++){
            for (int k=0; k<(int)rs[j].size(); k++){
                ofs << (rs[j][k]) << " ";
            }
            ofs << (i) << " ";
            ofs << "\n";
        }
        //WDEBUG("State " << pts[0].x << " " << pts[0].y << " " << pts[1].x << " " << pts[1].y  << " " << pts[pts.size()-1].x << " " << pts[pts.size()-1].y);
        rs.clear();
        
    }
    ofs.close();
    WDEBUG("Done printing final route to file.");
    
}



static void test_KRRF_basic(int argc, char **argv, const int id) {
	intro;

    CommonOptions2D copts;
	{
        CmdOptions o;
        addCommon2DOptions(o,copts);
        addCommon2DControlOptions(o,copts);

		if (!o.parse(argc,argv)) {
			cerr << o.makeCmdLine() << "\n";
			cerr << o.printHelp() << "\n";
			exit(0);
		}
	}

    initLogger( copts.userLog == 0 ? NULL : copts.prefix , argc,argv);
    setRandomSeed(copts.rseed);

    const char *prefix = copts.prefix;

	typedef TPoint Point;
	typedef MapR Map;

    typedef TGlobalRobot Robot;
	typedef Robot::State State;
	typedef Robot::Input Input;

	typedef KRRF<Robot> TRRT;

	Robot robot;	
    robot.enableBackward(false);
    // vector<vector< TPoint> > triangles(robot.getTriangles());
    // robot.scaleTriangles(triangles,copts.initRobotScale);
    // robot.setRapidModel(triangles);

    //TODO read init states from the file
    std::vector<State> init_states{};
    read_states(init_states, copts.targets, robot);


	Map map;
	map.loadMapTriangles(copts.mapFile);
	TRRT rrt(&map,&robot);

	char name[2000];


	CPainterBase *pa = initPainter((TPainter)copts.painterType,map.getDimension().getVector(),prefix,NULL,copts.painterScale);

    rrt.pa = pa; //for painting

    // generating random starts and goals
    if (copts.generateSGs > 0) {
        vector<State> initStates{};
        generateRandomStartGoals(robot, map, initStates, copts.generateSGs,copts.sgRadius);
        snprintf(name,sizeof(name),"%s.txt",prefix);
        saveStartGoalStates(initStates, name);
        char name2[2000];
        snprintf(name2,sizeof(name2),"%s.sginfo",prefix);
        //        getSGInfo(name,robot,map,copts.distanceToGoal, name2);

        if (pa && copts.distanceToGoal >= 0) {
            CColor initc;
            initc.setFloatRGB(0.978,1,0,100);
            CColor goalc;
            goalc.setFloatRGB(0.541,0.558,1,100);
            CColor linec("black");
            const double dtg = copts.distanceToGoal;
            pa->begin();
            drawMap(map,pa);
            for(int i=0;i<(int)initStates.size();i++) {
                drawRobot(robot.getShape(initStates[i]),pa);
                draw(robot.getRefPoint(initStates[i]),dtg,1,linec,initc,pa);
            }
            pa->close();
        }
        exit(0);
    }

    WDEBUG("Map dimension is " << map.getDimension());

    if (copts.drawSolution && pa) {
        pa->begin();
        drawMap(map,pa);
        for (int t=0; t<init_states.size(); t++){
            drawRobot(robot.getShape(init_states[t]),pa);
            draw(robot.getRefPoint(init_states[t]),copts.distanceToGoal,1,CColor("black", 128),CColor("orange", 128),pa);
            draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa);
        }
        pa->close();
    }


	rrt.setEdgeSamples(copts.edgeSamples);
	rrt.setEdgeTime(copts.edgeTime);
	rrt.setGoalBias(copts.goalBias);
	rrt.setSamplesPerInput(copts.inputBranching);
    rrt.setDistanceToGoalThreshold(copts.distanceToGoal);

    rrt.setMaxEdgeTime(copts.maxEdgeTime);
    rrt.setExploringProb(copts.exploringProba);
    rrt.setMonteCarloProp(copts.N_mcp);
    rrt.setExploitingRegion(copts.regExpl);
    rrt.setVideoGeneration(copts.generateVideo);
    rrt.setAMAX(copts.amax);

    char prefix2[2000];
    snprintf(prefix2,sizeof(prefix2),"./Video/vid_%s",prefix);

    
    CPainterBase *pa_video = initPainter((TPainter)copts.painterType,map.getDimension().getVector(),prefix2,NULL,copts.painterScale);

    rrt.pa_video = pa_video; //for painting
//  rrt.setExpansionStep(copts.expansionStep);
    
    snprintf(name,sizeof(name),"%s.stat",prefix);
    ofstream ofstat(name);

	CStat statistic;

    char name3[2000];
    snprintf(name3,sizeof(name3),"%s",prefix);

    char final_route[2000];
    snprintf(final_route,sizeof(final_route),"%s.nodes",prefix);

	for(int iter=0;iter<copts.numIter; iter++) {
        WDEBUG("Iteration " << iter);

        if (copts.drawSolution && pa_video) {
            
            pa_video->begin();
            drawMap(map,pa_video);
            for (int t=0; t<init_states.size(); t++){
                draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa_video);
            }
            pa_video->close();
        }
        rrt.generate(init_states, copts.rrtSize);

		vector<vector<TNode>> nodes{};
        vector<double> lengths{};
        vector<int> emp{};

        rrt.getTrajectories(nodes, lengths, emp);

        WDEBUG("Trajectory extracted");

        //TODO - uncomment
        if (copts.drawSolution && pa) {
            WDEBUG("drawing trajectory");
            for(int i=0;i<4;i++) {
                pa->begin();
                drawMap(map,pa);
                if (i % 2) {
                    drawKRRF_2DControl(rrt,robot,pa,false);
                }
                if (i / 2) {
                    drawTrajectoryKRRF2DControl(nodes,robot,pa,false);
                }
                for (int t=0; t<init_states.size(); t++){
                    //drawRobot(robot.getShape(init_states[t]),pa);
                    draw(robot.getRefPoint(init_states[t]),copts.distanceToGoal,1,CColor("black", 128),CColor("orange", 128),pa);
                    draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa);
                }
                pa->close();
            }
        }

        //for video purpose
        if (copts.drawSolution && pa_video) {
            WDEBUG("drawing trajectory");
            for(int i=1;i<4;i++) {
                pa_video->begin();
                drawMap(map,pa_video);
                if (i % 2) {
                    drawKRRF_2DControl(rrt,robot,pa_video,false);
                }
                if (i / 2) {
                    drawTrajectoryKRRF2DControl(nodes,robot,pa_video,false);
                }
                for (int t=0; t<init_states.size(); t++){
                    //drawRobot(robot.getShape(init_states[t]),pa);
                    draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa_video);
                }
                pa_video->close();
            }
        }
        
        /*TSP computation*/

        struct rusage t1,t2;
	    getTime(&t1);

        //Create file for LKH
        dist2TSPfile(lengths, rrt.n_states, name3);

        //Compute TSP using LKH
        std::string str(name3);
        string res = "./LKH " + str + ".par";
        //Compute TSP using LKH
        system(res.c_str());

        //Read the order
        std::vector<int> order{};
        TSPresultRead(rrt.n_states, order, name3);

        getTime(&t2);

        if (copts.drawSolution && pa_video) {
            WDEBUG("drawing trajectory");
            for(int i=1;i<4;i++) {
                pa_video->begin();
                drawMap(map,pa_video);
                if (i % 2) {
                    drawKRRF_2DControl(rrt,robot,pa_video,false);
                }
                if (i / 2) {
                    drawTrajectoryKRRF2DControl(nodes,robot,pa_video,false);
                }
                for (int t=0; t<init_states.size(); t++){
                    //drawRobot(robot.getShape(init_states[t]),pa_video);
                    draw(robot.getRefPoint(init_states[order[t]]),10,1,CColor("black"),CColor("cyan"),pa_video);
                    std::string number_str = std::to_string(t+1);
                    char const *num_char = number_str.c_str();
                    pa_video->draw(num_char, tgp(robot.getRefPoint(init_states[order[t]]),false));
                }
                pa_video->close();
            }
        }

        //Recomputing of the trajectory along all TSP-computed edges
        WDEBUG("Computing the tour using guiding");

        if (rrt.generate_TSP_guiding(order, copts.rrtSize)){
            //rrt.generate_TSPTree(order, copts.rrtSize);

            //for video purpose

            vector<vector<TNode>> TSP_nodes{};
            vector<double> TSP_lengths{};

            rrt.getTrajectories(TSP_nodes, TSP_lengths, order);

            statistic = rrt.getStatistic();
            statistic["NodesGuiding"] = 0;
        
            for (int i = 0; i < TSP_nodes.size(); i++){
                statistic["NodesGuiding"] = statistic["NodesGuiding"] + rrt._tree_TSP[i].size();
            }

            if (copts.drawSolution && pa_video) {
                pa_video->begin();
                drawMap(map,pa_video);
                drawKRRF_2DControl(rrt,robot,pa_video,true);
                for (int i = 0; i<init_states.size() ; i++){
                    int F_tree = order[i];
                    int B_tree = order[(i+1)%init_states.size()];
                    
                    std::vector<std::vector<TNode>> guide_nodes{};
                    std::vector<TNode> guider{};
                    if (rrt.shortest_trajs[F_tree*init_states.size() + B_tree].size() > 0){
                        for (int j = 0; j<rrt.shortest_trajs[F_tree*init_states.size() + B_tree].size(); j++){
                            guider.push_back(*rrt.shortest_trajs[F_tree*init_states.size() + B_tree][j]);
                        }
                        std::reverse(guider.begin(), guider.end());
                        guide_nodes.push_back(guider);
                    } else {
                        for (int j = 0; j<rrt.shortest_trajs[B_tree*init_states.size() + F_tree].size(); j++){
                            guider.push_back(*rrt.shortest_trajs[B_tree*init_states.size() + F_tree][j]);
                        }
                        std::reverse(guider.begin(), guider.end());
                        guide_nodes.push_back(guider);
                    }
                    drawTrajectoryKRRF2DControl(guide_nodes,robot,pa_video,false);

                }        
                for (int t=0; t<init_states.size(); t++){
                    //drawRobot(robot.getShape(init_states[t]),pa);
                    draw(robot.getRefPoint(init_states[order[t]]),10,1,CColor("black"),CColor("cyan"),pa_video);
                    std::string number_str = std::to_string(t+1);
                    char const *num_char = number_str.c_str();
                    pa_video->draw(num_char, tgp(robot.getRefPoint(init_states[order[t]]),false));
                }
                pa_video->close();
            }

            
            if (copts.drawSolution && pa) {
                WDEBUG("HEEEERE drawing trajectory");
                for(int i=1;i<4;i++) {
                    pa->begin();
                    drawMap(map,pa);
                    if (i % 2) {
                        drawKRRF_2DControl(rrt,robot,pa,true);
                    }
                    if (i / 2) {
                        drawTrajectoryKRRF2DControl(TSP_nodes,robot,pa,true);
                    }
                    for (int t=0; t<init_states.size(); t++){
                        //drawRobot(robot.getShape(init_states[t]),pa);
                        draw(robot.getRefPoint(init_states[t]),copts.distanceToGoal,1,CColor("black", 128),CColor("orange", 128),pa);
                        draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa);
                        //draw(robot.getRefPoint(TSP_nodes[t][0].state),1,1,CColor("green"),CColor("green"),pa);
                        //draw(robot.getRefPoint(TSP_nodes[t][TSP_nodes[t].size()-1].state),1,1,CColor("red"),CColor("red"),pa);
                    }
                    pa->close();
                }
            }

            if (copts.drawSolution && pa_video) {
                WDEBUG("drawing trajectory");
                for(int i=1;i<4;i++) {
                    pa_video->begin();
                    drawMap(map,pa_video);
                    if (i % 2) {
                        drawKRRF_2DControl(rrt,robot,pa_video,true);
                    }
                    if (i / 2) {
                        drawTrajectoryKRRF2DControl(TSP_nodes,robot,pa_video,true);
                    }
                    for (int t=0; t<init_states.size(); t++){
                        //drawRobot(robot.getShape(init_states[t]),pa_video);
                        draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa_video);
                    }
                    pa_video->close();
                }
            }
            
            statistic["TSP_Computation"] = getTime(t1,t2);

            // Output final routes into file

            //final_route
            outputPointsToFile(TSP_nodes, robot, final_route);
            double final_length = 0;
            for (size_t i = 0; i<rrt._tree_TSP.size(); i++){
                final_length = final_length + rrt._tree_TSP[i].back()->KRRF_hGet();
            }
            statistic["Final_Time"] = statistic["TSP_Computation"] + statistic["TSP_RTTreeBuildTime"] + statistic["KRRFTreeBuildTime"];
            statistic["Final_path_length"] = final_length;
        } else {
            statistic = rrt.getStatistic();
            statistic["NodesGuiding"] = 0;
            statistic["TSP_Computation"] = getTime(t1,t2);
            double final_length = 0;
            for (size_t i = 0; i<rrt._tree_TSP.size(); i++){
                final_length = final_length + rrt._tree_TSP[i].back()->KRRF_hGet();
            }
            statistic["Final_Time"] = 1000;
            statistic["Final_path_length"] = 10000000;
            statistic["NodesGuiding"] = 0;

        }
        statistic["NodesKRRF"] = 0;
        for (int t = 0; t < rrt._tree.size(); t++){
            statistic["NodesKRRF"] = statistic["NodesKRRF"] + rrt._tree[t].size();
        }
        
        ofstat << statistic.printFull() << "\n";
		ofstat.flush();

        /*if (pa && copts.drawSolution > 0) {
            animateTrajectory(rrt, nodes, traj, pa, map, robot, initState, goalState);
        }
        if (copts.drawSolution > 1 && pa) {
            const vector< TNode *> &tree(rrt.getTreeFwd());
//            drawTreeGrowingProcess(tree, pa, map, robot,initState, goalState, copts.growSpeed);
		}*/
    }
	
    ofstat.close();

}




static void runAlgorithms(int argc, char **argv, const int k) {

    test_KRRF_basic(argc,argv,k); // 1
}


int main(int argc, char **argv) {

	const int k = argc < 2?-1:atoi(argv[1]);

    initRandomNumberGenerator(); // very important
	const int seed = setRandom();
//	char logName[200];
//	snprintf(logName,sizeof(logName),"_r2.%d.log",getpid());
//	__wlog = new _WLog(logName,cerr);

	WDEBUG("command line arguments:");
    WDEBUG("seed is " << seed); 

	for(int i=0;i<argc;i++) {
		WDEBUG("argv["<<i<<"]:"<<argv[i]);
	}

#ifdef __OPTIMIZE__
    WDEBUG("Optimization turned on");
#else 
    WDEBUG("Optimization turned off");
#endif

	if (k > 0) {
		argc--;
		argv++;
	}

	runAlgorithms(argc,argv,k);
    cerr << "\n";


    freeRandomNumberGenerator();


}

