

#include "types.h"
#include "mapr.h"
#include "mesh.h"
#include "rrt2D.h"
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

//#include "CRobotDiff.h"
#include "CRobot2D.h"
#include "CRobotCarLike.h"
//#include "CRobotBike.h"
#include "ut.h"
#include "polygonUtils.h"
//#include "Voronoi.h"
#include "CStat.h"
//#include "quasiRand.h"

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
    trajectory.clear();
    
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

template<typename S, typename RT>
void read_traj_states(std::vector<S> &traj_states, const char *filename, RT &robot){
    std::ifstream ifs(filename);
	
	string line;
    WDEBUG("Reading trajectory");
	while(ifs) {
		getline(ifs,line);
        //WDEBUG(line);
		std::vector<double> vd(lineToNum<double>(line));
        if(vd.size() > 0){
            S initState(robot.getState());
            initState[0] = vd[0];
            initState[1] = vd[1];
            initState[2] = vd[2];
            traj_states.push_back(initState);
        }
		vd.clear();
	}
	ifs.close();
    WDEBUG("trajectory read")
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

static void visualize_SFF(int argc, char **argv, const int id) {
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

	//typedef RRT2D<Robot> TRRT;

	Robot robot;	
    robot.enableBackward(false);

    //TODO read init states from the file
    std::vector<State> init_states{};
    read_states(init_states, copts.targets, robot);

    std::vector<State> traj_states{};
    read_traj_states(traj_states, copts.trajs, robot);

	Map map;
	map.loadMapTriangles(copts.mapFile);
	//TRRT rrt(&map,&robot);




	char name[2000];


	CPainterBase *pa = initPainter((TPainter)copts.painterType,map.getDimension().getVector(),prefix,NULL,copts.painterScale);

    WDEBUG("drawing trajectory");
    for(int i=0;i<1;i++) {
        pa->begin();
        drawMap(map,pa);
        drawTrajectory2Dtn(traj_states,robot,pa);
        for (int t=0; t<init_states.size(); t++){
            draw(robot.getRefPoint(init_states[t]),copts.distanceToGoal,1,CColor("black", 128),CColor("orange", 128),pa);
            draw(robot.getRefPoint(init_states[t]),10,1,CColor("black"),CColor("cyan"),pa);
        }
        pa->close();
    }
}

static void runAlgorithms(int argc, char **argv, const int k) {

    visualize_SFF(argc,argv,k); // 1
}


int main(int argc, char **argv) {

	const int k = argc < 2?-1:atoi(argv[1]);

    initRandomNumberGenerator(); // very important
	const int seed = setRandom();

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

