
#include "rrt2D.h"

#include "types.h"
#include "rrtstate.h"
#include "mapr.h"
#include "WLog.h"
#include "ut.h"
#include "rapid/RAPID.H"
#include "CStat.h"
#include <vector>
#include <list>
#include <fstream>
#include <algorithm>
#include <map>
#include <string>
#include "dijkstra/dijkstra_lite.h"

#include "painting.h"

/** implementation of several RRT algorithms for 2D robota (2D workspace). 
  Collision detection is realized using Rapid. 
  K-nn is realized using mpnn2 library
  
  * this class containts:
  * basic RRT (generate)
  * RRT-Connect (generateConnect)
  * RRT-Blossom (generateBlossom)
  * RRT-star (generateOptimal)

  *
  * and some testing version of rrt - see method generate*()
  */

/** FOR GEOMETRIC PLANNING */

namespace rrtPlanning {


    RRT2D::RRT2D(MapR *m, CRobot2D *r):
robot(r), map(m), distanceToGoalTh(-1), goalBias(-1),
robotDistanceDimension(robot->getDistanceDim())	{
    _initState.clear();
    _goalState.clear();
    _kdTree = NULL;

    _guidingDistance = 150;
    _guidingSensitivity = 0.1;
    _randomPoints.clear();
    _outerGoalDistance = -1;
    _resolution = 5; // bylo 0.1
    _inputResolution = 0.01;
    _pa = NULL;
    _expansionStep = 0.5;
}


RRT2D::~RRT2D() {
    map = NULL;
    robot = NULL;
    _initState.clear();
    _goalState.clear();
    if (_kdTree != NULL) {
        delete _kdTree;
    }
    _randomPoints.clear();
    deleteTree(_tree);
}


std::vector<TNode> RRT2D::getCopyTree() const {
    vector<TNode> a;
    for(int i=0;i<(int)_tree.size();i++) {
    }
}

void RRT2D::addToKdTree(KDTreeInt *kdTree, const State &s, const int index) {
	MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		annpt[0][i] = s[i];
	}
	kdTree->AddPoint(annpt[0],index);
	MPNN::annDeallocPts(annpt);
}





void RRT2D::deleteTree(std::vector<TNode *> &tree) const {
    for(int i=0;i<(int)tree.size();i++) {
        if (tree[i]) {
            delete tree[i];
        }
    }
    tree.clear();
}


void RRT2D::clearStatistics() {
	_statistic.zero();

	_statistic["time"] = 0;
	_statistic["iterations"] = 0;
   _statistic["treeSize"] = 0;
   _statistic.setStrValue("algorithm", "");
	

}

template<typename T>
struct myLessVD_rrt2 {
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




void RRT2D::getVoronoi(const std::vector<TPoint> &inPts, vector< TPoint > &edges) const {
	typedef Voronoi::Point VPoint;
    vector<VPoint> vpts;
    vpts.reserve(inPts.size());
    for(int i=0;i<(int)inPts.size();i++) {
        vpts.push_back(VPoint(inPts[i].x, inPts[i].y));
    }
	sort(vpts.begin(),vpts.end(), myLessVD_rrt2<VPoint>());
	vector<VPoint>::iterator q = unique(vpts.begin(),vpts.end());
	vector<VPoint> vptsu;
	copy(vpts.begin(),q,back_inserter(vptsu));
	cerr << "After uniq: " << vptsu.size() << "\n";
	cerr << "Generating voronoi diagram .. ";
	
	vector<double> dim(map->getDimension().getVector());
	Voronoi::VoronoiDiagramGenerator vdg;
	vdg.generateVoronoi(&vptsu,dim[0],dim[1],dim[2],dim[3]);	
	edges = getVoronoiSegments(vdg);
}




/* node expansion. Also saves contact informations, if the expansion leads to a collision */
void RRT2D::expandNodeWithContacs(const int nearidx, const State &rstate, vector< TNode *> &newNodes, SContact &contact) {

    TNode *tn = _tree[ nearidx ];
    const State &nearState( tn->state );

    const int num = sqrt(robot->distance(nearState, rstate)) / _expansionStep;

    vector<State> line;
    if (num <= 2) {
        line.push_back(tn->state);
        line.push_back(rstate);
    } else {
        approximateStates( tn->state, rstate , num, line);
    }
    newNodes.clear();

    if (line.size() < 2) {
        return;
    }
    
    if (canBeConnectedWithContact( tn->state,line[1],_resolution, contact)) {
        newNodes.push_back( new TNode() );
        newNodes.back()->RRT2D_setDefault();
        newNodes.back()->state = line[1];
        newNodes.back()->parent = nearidx;
        newNodes.back()->pstate = tn->state;
    }
}






/** gext next state which is on line tn,rstate.
  * this is faster than getReachableStates or getReachableStatesRandom which generates the reachable states using predefined set of inputs */
void RRT2D::getReachableStatesDirectoGeometryGrowingWithContact(const std::vector<TNode *> &tree, const int nearidx, const State &rstate, vector< TNode *> &newNodes, State &lastFree) {
    lastFree.clear();

    TNode *tn = tree[ nearidx ];
    const State &nearState( tn->state );

    const int num = sqrt(robot->distance(nearState, rstate)) / _expansionStep;

    vector<State> line;
    if (num <= 2) {
        line.push_back(tn->state);
        line.push_back(rstate);
    } else {
        approximateStates( tn->state, rstate , num, line);
    }
    newNodes.clear();

    if (line.size() < 2) {
        return;
    }
    if (canBeConnected( tn->state,line[1],_resolution)) {
        newNodes.push_back( new TNode() );
        newNodes.back()->RRT2D_setDefault();
        newNodes.back()->state = line[1];
        newNodes.back()->parent = nearidx;
        newNodes.back()->pstate = tn->state;
    } else {
        lastFree = line[1];
    }
}



/** gext next state which is on line tn,rstate.
  * this is faster than getReachableStates or getReachableStatesRandom which generates the reachable states using predefined set of inputs */
void RRT2D::getReachableStatesDirectoGeometryGrowing(const std::vector<TNode *> &tree, const int nearidx, const State &rstate, vector< TNode *> &newNodes) {

    TNode *tn = tree[ nearidx ];
    const State &nearState( tn->state );

    const int num = sqrt(robot->distance(nearState, rstate)) / _expansionStep;

    vector<State> line;
    if (num <= 2) {
        line.push_back(tn->state);
        line.push_back(rstate);
    } else {
        approximateStates( tn->state, rstate , num, line);
    }
    newNodes.clear();

    if (line.size() < 2) {
        return;
    }
    if (canBeConnected( tn->state,line[1],_resolution)) {
        newNodes.push_back( new TNode() );
        newNodes.back()->RRT2D_setDefault();
        newNodes.back()->state = line[1];
        newNodes.back()->parent = nearidx;
        newNodes.back()->pstate = tn->state;
    }
}










/** gext next state which is on line tn,rstate.
  * this is faster than getReachableStates or getReachableStatesRandom which generates the reachable states using predefined set of inputs */
void RRT2D::getReachableStatesDirectoGeometryGrowing(const int nearidx, const State &rstate, vector< TNode *> &newNodes) {

    TNode *tn = _tree[ nearidx ];
    const State &nearState( tn->state );

    const int num = sqrt(robot->distance(nearState, rstate)) / _expansionStep;

    vector<State> line;
    if (num <= 2) {
        line.push_back(tn->state);
        line.push_back(rstate);
    } else {
        approximateStates( tn->state, rstate , num, line);
    }
    newNodes.clear();

    if (line.size() < 2) {
        return;
    }
    
    if (canBeConnected( tn->state,line[1],_resolution)) {
        newNodes.push_back( new TNode() );
        newNodes.back()->RRT2D_setDefault();
        newNodes.back()->state = line[1];
        newNodes.back()->parent = nearidx;
        newNodes.back()->pstate = tn->state;
    }
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



/* basic RRT */
void RRT2D::generate(const State &fromState, const State &toState, const int size) {

	clearStatistics();
    _statistic.setStrValue("algorithm", "rrt");

	struct rusage t1,t2;
	getTime(&t1);


	_initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2D_setDefault();
    _tree.back()->state = fromState;


	if (_kdTree != NULL) {
		delete _kdTree;
		_kdTree = NULL;
	}

    _kdTree = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale(), size*2);
    addToKdTree(_kdTree, _tree[0]->state, 0);

    const SDimension &mapDimension(map->getDimension());

    if (0) {
        // test matrix
        WDEBUG("Testing metric!");
        for(int i=0;i<10;i++) {
			State randomState = robot->getRandomState(mapDimension);

            double distToNear;
            const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

            const double rdist = sqrt( robot->distance(randomState, _tree[nearIDX]->state));

            WDEBUG("step " << i << " kdtree-dist=" << distToNear << ", robot-dist=" << rdist << ", diff= " << (rdist-distToNear));

        }
    }


	State randomState;


	struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);

    const double distanceToGoalTh2= distanceToGoalTh >=0 ?  distanceToGoalTh*distanceToGoalTh : -1;


	for(int iter = 0; iter < size; iter++) {
/*
		getTime(&someTime);
		if (getTime(timeOfLastInfo,someTime) > 0.2) {
			std::cerr << "[" << iter << "/" << _tree.size() << "] ";
			getTime(&timeOfLastInfo);
		}
        */
		if (robot->distance(_tree.back()->state,toState) < distanceToGoalTh2) {
			WDEBUG("goal state reached to distance: " << robot->distance(_tree.back()->state,toState));
			break;
		}	
        if (_outerGoalDistance > 0 && robot->distance(_tree.back()->state, fromState) > _outerGoalDistance*_outerGoalDistance) {
			WDEBUG("OUTERgoal state reached to distance: " << robot->distance(_tree.back()->state,fromState));
            WDEBUG("threshold: " << _outerGoalDistance*_outerGoalDistance);
            _outerGoal = _tree.back()->state;
			break;
        }

		if (goalBias > 0 && getRandom(0,1) < goalBias) {
			randomState = toState;
		} else {
			randomState = robot->getRandomState(mapDimension);
		}

        double distToNear = -1;
        const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

		if (nearIDX >= 0) {
                
            vector<TNode *> newNodes;
            
			getReachableStatesDirectoGeometryGrowing(nearIDX,randomState, newNodes);
            int nearestToRand = findNearestState(newNodes, randomState);

			if (nearestToRand != -1) {
				// add nearestNode to a tree
				_tree.push_back( newNodes[ nearestToRand ] );
                addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                newNodes[ nearestToRand ] = NULL;
			}
            deleteTree(newNodes);
		} 
		_statistic["iterations"] = iter;

        if ((iter % 20) == 0 && _pa) {
            _pa->begin();
            drawMap(*map,_pa);
            drawRobot(robot->getShape(_initState),_pa);
            drawRobot(robot->getShape(_goalState),_pa);
            drawRRT_2D(*this,*robot,_pa);

            if (0) {
                // draw vorooni diagram
                typedef Voronoi::Point VPoint;
                vector<VPoint> vpts;
                vector<TPoint> treepts;
                for(int i=0;i<(int)_tree.size();i++) {
                    const State &state(_tree[i]->state);
                    const TPoint p(robot->getRefPoint(state));
                    vpts.push_back( VPoint(p.x, p.y ) );
                    treepts.push_back(robot->getRefPoint(state));
                }
                sort(vpts.begin(),vpts.end(), myLessVD<VPoint>());
                vector<VPoint>::iterator q = unique(vpts.begin(),vpts.end());
                vector<VPoint> vptsu;
                copy(vpts.begin(),q,back_inserter(vptsu));
                Voronoi::VoronoiDiagramGenerator vdg;
                const vector<double> dim(map->getDimension().getVector());
                vdg.generateVoronoi(&vptsu,dim[0],dim[1],dim[2],dim[3]);	
                vector<TPoint> vsegments(getVoronoiSegments(vdg));

                for(int i=0;i<(int)vsegments.size()-1;i+=2) {
                    vector<TPoint> lp;
                    lp.push_back(vsegments[i]);
                    lp.push_back(vsegments[i+1]);
                    drawPl(lp,2,CColor("grey50"),_pa);
                    lp.clear();
                }
                drawPts(treepts,5,1,CColor("orange"),CColor("orange"),_pa);

            }
            _pa->close();
        }
	}

	getTime(&t2);
	_statistic["time"] = getTime(t1,t2);
    _statistic["treeSize"] = _tree.size();

}


void RRT2D::getRandomOnLine(const State &from, const State &to, const double radius, State &result) const {
    const double t = getRandom(0,1);

    State s(robot->getStateSize(),0);
    for(int i=0;i<(int)s.size();i++) {
        s[i] = from[i]*(1-t) + to[i]*(t);
    }

    result[0] = getRandomGauss(s[0], radius);
    result[1] = getRandomGauss(s[1], radius);
    result[2] = getRandomGauss(s[2], M_PI/4);

}






void RRT2D::generateConnect(const State &fromState, const State &toState, const int size) {

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);


	_initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2D_setDefault();
    _tree.back()->state = fromState;


    const SDimension &mapDimension(map->getDimension());
	

	State randomState;


	if (_kdTree != NULL) {
		delete _kdTree;
		_kdTree = NULL;
	}

    _kdTree = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTree, _tree[0]->state, 0);

	struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);

    const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;

	for(int iter = 0; iter < size; iter++) {

		getTime(&someTime);
		if (getTime(timeOfLastInfo,someTime) > 0.2) {
			std::cerr << "[" << iter << "/" << _tree.size() << "] ";
			getTime(&timeOfLastInfo);
		}
		if (robot->distance(_tree.back()->state,toState) < distanceToGoalTh2) {
			WDEBUG("goal state reached to distance: " << robot->distance(_tree.back()->state,toState));
			break;
		}	
        if (_outerGoalDistance > 0 && robot->distance(_tree.back()->state, fromState) > _outerGoalDistance*_outerGoalDistance) {
			WDEBUG("OUTERgoal state reached to distance: " << robot->distance(_tree.back()->state,fromState));
            WDEBUG("threshold: " << _outerGoalDistance*_outerGoalDistance);
            _outerGoal = _tree.back()->state;
			break;
        }

		if (goalBias > 0 && getRandom(0,1) < goalBias) {
			randomState = toState;
		} else {
			randomState = robot->getRandomState(mapDimension);
		}

        double distToNear;
        int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

		if (nearIDX >= 0) {
            bool wasAdded = false;
            do {
                wasAdded = false;

                vector<TNode *> newNodes;

                getReachableStatesDirectoGeometryGrowing(nearIDX,randomState, newNodes);

                const int nearestToRand = findNearestState(newNodes, randomState);

                if (nearestToRand != -1) {
                    const double distToRand = robot->distance(randomState, newNodes[ nearestToRand ]->state);
                    if ( distToRand > 0.01) {
                        // add nearestNode to a tree
                        _tree.push_back( newNodes[ nearestToRand ] );
                        addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                        wasAdded = true;
                        newNodes[ nearestToRand ] = NULL;
//                        if (!_useGeometryGrowing) {
  //                          _tree[nearIDX]->next.push_back( (int)_tree.size()-1);
    //                    }
                        nearIDX = (int)_tree.size()-1;
                    }
                }
                deleteTree(newNodes);
            } while (wasAdded);
		} 
		_statistic["iterations"] = iter;
	}
	getTime(&t2);
	_statistic["time"] = getTime(t1,t2);
	WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
}






/** traverse from startIndex to tree[0] and report total cost at startIndex */
double RRT2D::getCost(int startIndex) const {

    double totalCost = 0;
    while(startIndex >= 0) {
        const int prevIndex = _tree[startIndex]->parent;
        if (prevIndex != -1) {
            totalCost += sqrt( robot->distance(_tree[prevIndex]->state, _tree[startIndex]->state));
        } else {
            break;
        }
        startIndex = prevIndex;
    }
                
    return totalCost;
}


bool RRT2D::blossomUseNode(const State &s, const State &parentState) const {
    // return true if the node is NOT growing into the tree
		
    const double distanceToParent = robot->distance(s, parentState);
    double distToNear; // after sqrt
    const int nearIDX = _findNearestState(_kdTree, s, distToNear);
    return (distToNear*distToNear >= distanceToParent);
}

void RRT2D::blossomPrunning(const vector<TNode *> &nodes, const State &parentState, vector< TNode *> &prunedNodes) {

    prunedNodes.clear();    
    prunedNodes.reserve(nodes.size());

	for(int i=0;i<(int)nodes.size();i++) {
		
		const double parentDistance = robot->distance(nodes[i]->state,parentState);
        
        double distToNear; // after sqrt
        const int nearIDX = _findNearestState(_kdTree, nodes[i]->state, distToNear);
//		Tree_iterator ti = _findNearestState((*ni).state);:
//		if (robot->distance((*ni).state, (*ti).state) >= parentDistance) 
        if (distToNear*distToNear >= parentDistance) {
			prunedNodes.push_back(nodes[i]);
		}
	}
}






void RRT2D::generateBlossom(const State &fromState, const State &toState, const int size) {

    clearStatistics();

    struct rusage t1,t2;
    getTime(&t1);


    _initState = fromState;
    _goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2D_setDefault();
    _tree.back()->state = fromState;


    const SDimension &mapDimension(map->getDimension());


    State randomState;


    if (_kdTree != NULL) {
        delete _kdTree;
        _kdTree = NULL;
    }

    _kdTree = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTree, _tree[0]->state, 0);

    struct rusage timeOfLastInfo, someTime;
    getTime(&timeOfLastInfo);

    //    ofstream ofs("paa.rrtpts");
    const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;

    for(int iter = 0; iter < size; iter++) {

        getTime(&someTime);
        if (getTime(timeOfLastInfo,someTime) > 0.2) {
            std::cerr << "[" << iter << "/" << _tree.size() << "] ";
            getTime(&timeOfLastInfo);
        }
        if (robot->distance(_tree.back()->state,toState) < distanceToGoalTh2) {
            WDEBUG("goal state reached to distance: " << robot->distance(_tree.back()->state,toState));
            break;
        }	
        if (_outerGoalDistance > 0 && robot->distance(_tree.back()->state, fromState) > _outerGoalDistance*_outerGoalDistance) {
            WDEBUG("OUTERgoal state reached to distance: " << robot->distance(_tree.back()->state,fromState));
            WDEBUG("threshold: " << _outerGoalDistance*_outerGoalDistance);
            _outerGoal = _tree.back()->state;
            break;
        }

        if (goalBias > 0 && getRandom(0,1) < goalBias) {
            randomState = toState;
        } else {
            randomState = robot->getRandomState(mapDimension);
        }

        double distToNear;
        const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

        if (nearIDX >= 0) {
            vector<TNode *> newNodes;

            getReachableStatesDirectoGeometryGrowing(nearIDX,randomState, newNodes);

            int nearestToRand = findNearestState(newNodes, randomState);

            for(int i=0;i<(int)newNodes.size();i++) {
                if (blossomUseNode(newNodes[i]->state, _tree[ nearIDX ]->state) || i == nearestToRand) {
                    // add nearestNode to a tree
                    _tree.push_back( newNodes[ i ] );
                    addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                    newNodes[ i ] = NULL;
//                    if (!_useGeometryGrowing) {
  //                      _tree[nearIDX]->next.push_back( (int)_tree.size()-1);
    //                }
                }
            }
            deleteTree(newNodes);
        } 
        _statistic["iterations"] = iter;
    }
    getTime(&t2);
    _statistic["time"] = getTime(t1,t2);
    WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
}



/** basic RRT* - RRT-star
  * a node stores: cumulativeLength = distance in the tree from start to node
  *
  * after a new node is introduced, it's neighbor is search and rewired. rewiring a node 'x' to have 'y' as parent if:
  * d = distance
    d(start,y) + d(y,x) < di(start, x) 
  */
void RRT2D::generateOptimal(const State &fromState, const State &toState, const int size) {
	
    CPainterBase *pa = new CPainterCairo("rrsa",CPainterCairo::PNG,map->getDimension().getVector());
    pa = NULL;

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);


    _initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2DStar_setDefaults();
    _tree.back()->state = fromState;


    const SDimension &mapDimension(map->getDimension());
	
	State randomState;

//    Input maxv(robot->getMinInputValues());
  //  const double maxstep = fabs(maxv[0]);



	if (_kdTree != NULL) {
		delete _kdTree;
		_kdTree = NULL;
	}

    const int nearestNeighbors = 16; // mpnn cannot do more than 16
    _kdTree = new KDTreeInt( robot->getDistanceDim(),nearestNeighbors, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTree, _tree[0]->state, 0);


    const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;

	struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);
//    Input emptyInput(robot->getInputSize(),0);



	for(int iter = 0; iter < size; iter++) {

        if (1 && pa && (iter % 500) == 0) {
            pa->begin();
            drawMap(*map,pa);
            
            vector<double> costs;
            costs.reserve(_tree.size());
            for(int i=0;i<(int)_tree.size();i++) {
                costs.push_back(0);
                if (i > 0) {
                    costs[i] = getCost(i);
                }
            }
            const double mincost = *std::min_element(costs.begin(), costs.end());
            const double maxcost = *std::max_element(costs.begin(), costs.end());

            vector<TPoint> pts;
            CColorMap cm;
            cm.setRange(mincost, maxcost);
            for(int ti = 0; ti < (int)_tree.size(); ti++) {
                if (_tree[ ti ]->parent != -1) {
                    pts.push_back(robot->getRefPoint(_tree[ ti ]->state));
                    pts.push_back(robot->getRefPoint(_tree[ ti ]->pstate));
//                    pts.push_back(robot->getRefPoint(_tree[ _tree[ti]->parent ]->state));
                    drawPl(pts, 2, cm.getColor(costs[ti]),pa);
                    pts.clear();
                }
            }
            //drawRRT(*this,*robot,pa);
            pa->close();

        }

		getTime(&someTime);
		if (getTime(timeOfLastInfo,someTime) > 0.2) {
			std::cerr << "[" << iter << "/" << _tree.size() << "] ";
			getTime(&timeOfLastInfo);
		}
		if (distanceToGoalTh >= 0 && robot->distance(_tree.back()->state,toState) < distanceToGoalTh2) {
			WDEBUG("goal state reached to distance: " << robot->distance(_tree.back()->state,toState));
			break;
		}	

		if (goalBias > 0 && getRandom(0,1) < goalBias) {
			randomState = toState;
		} else {
			randomState = robot->getRandomState(mapDimension);
		}
	
		//Tree_iterator n = _findNearestState(randomState);
        double distToNear;
        const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

		if (nearIDX >= 0) {

            vector<TNode *> newNodes;

            getReachableStatesDirectoGeometryGrowing(nearIDX,randomState, newNodes);

            int nearestToRand = findNearestState(newNodes, randomState);

            if (nearestToRand != -1) {
                TNode *qnew = newNodes[ nearestToRand ];

                // first find the best parent for this node
                vector<int> nearNodes;
                if (_tree.size() < nearestNeighbors+2) {
                    nearNodes.push_back(nearIDX);
                } else {
                    _findNearestStateMultiple(qnew->state, nearestNeighbors, nearNodes);
                }

                int bi = -1;
                double bestCost = -1;
                for(int j=0;j<(int)nearNodes.size();j++) {
                    const int idx = nearNodes[j];

                    const double distToNew = sqrt( robot->distance(_tree[ idx]->state, qnew->state) );
                    const double cost = _tree [idx]->RRT2DStar_getCost() + distToNew;
                    //WDEBUG("inspecting[" << j << ", cost=" << cost);
                    if (cost < bestCost || bi == -1) {
                        if (canBeConnected( _tree[idx]->state, qnew->state, _resolution)) {
                            bestCost = cost;
                            bi = idx;
                        }
                    }
                }
                //                WDEBUG("Best is " << bi << ", cost=" << bestCost);

                // best parent of NEW is nearNodes[ bi ];
                int parent = bi;
                vector<State> states;
                if (parent != -1) {
                    const double d = sqrt(robot->distance(_tree[ parent ]->state, qnew->state) );
                    approximateStates(_tree[ parent ]->state,qnew->state,(int)lround(2.0*d/_expansionStep), states);
                }

                for(int ssi = 0; ssi < (int)states.size();ssi++) {

                    _tree.push_back( new TNode() );
                    _tree.back()->RRT2DStar_setDefaults();
                    _tree.back()->state = states[ssi];
                    _tree.back()->parent = parent;
                    _tree.back()->pstate = _tree[ parent ]->state;
                    _tree[ parent ]->next.push_back((int) _tree.size() - 1 );

                    const double dtopar = robot->distance(_tree.back()->pstate, _tree.back()->state);
                    _tree.back()->RRT2DStar_cost() = _tree[ _tree.back()->parent ]->RRT2DStar_getCost() + sqrt(dtopar);

                    //_tree.back()->RRT2DStar_cost() = getCost( (int)_tree.size()-1);

                    addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );


                    parent = (int)_tree.size()-1;


                    // now rewiring
                    if (1 && _tree.size() > nearestNeighbors+2) {
                        vector<int> nearNodes;
                        _findNearestStateMultiple(_tree.back()->state, nearestNeighbors, nearNodes);
                        //const double lastCost = getCost( (int)_tree.size()-1);
                        const double lastCost = _tree[ parent ]->RRT2DStar_getCost();

                        for(int i=0;i<(int)nearNodes.size();i++) {
                            const int candidateIdx = nearNodes[i]; 
                            //                            const double oldcost = getCost(candidateIdx);
                            const double oldcost = _tree[ candidateIdx ]->RRT2DStar_getCost();
                            const double newcost = lastCost + sqrt(robot->distance(_tree.back()->state, _tree[ candidateIdx]->state));

                            if (newcost < oldcost) {
                                if (canBeConnected( _tree.back()->state, _tree[candidateIdx]->state, _resolution)) {

                                    TNode *c = _tree[ candidateIdx];
                                    // remove candidateIdx from it's parent's next
                                    if (c->parent != -1) {
                                        _tree[ c->parent ]->RRT2DStar_removeNextIdx( candidateIdx );
                                    }

                                    c->parent = (int)_tree.size()-1;
                                    _tree[ c->parent ]->next.push_back(candidateIdx);

                                    c->RRT2DStar_cost() = newcost;
                                    c->pstate = _tree.back()->state;
                                    updateCostForward( c->parent );
                                }
                            }
                        }
                    }
                }

            }
		} // n != tree.end()
		_statistic["iterations"] = iter;
	}
	getTime(&t2);
	_statistic["time"] = getTime(t1,t2);
	WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
}

struct HistoryTreeNode {
    typedef CRobot2D::State State;

    HistoryTreeNode(){

    }

    ~HistoryTreeNode() {
        /*
        for(int i=0;i<(int)lowTrees.size();i++) {
            for(int j=0;j<(int)lowTrees[i].size();j++) {
                delete lowTrees[i][j];
                lowTrees[i][j] = NULL;
            }
            lowTrees[i].clear();
        }
        */
        lowTrees.clear();
        distributionCenter.clear();
    }



    vector< vector<TNode *> > lowTrees;
    vector< State > distributionCenter;
    vector< int > previous;
};



void RRT2D::updateCostForward(const int startIdx) {

    vector<int> open;
    open.reserve(5000);
    open.push_back(startIdx);

    while(open.size() > 0) {
        const int idx = open.back();
        open.pop_back();
        //WDEBUG("updating, open.size=" << open.size() << ", actual=" << idx);

        if (_tree[ idx ]->parent != -1) {
            const double parentCost = _tree[ _tree[ idx ]->parent ]->RRT2DStar_getCost();
            const double parentToNode = sqrt( robot->distance( _tree[ _tree[idx]->parent]->state, _tree[idx]->state) );
            _tree[ idx ]->RRT2DStar_cost() = parentCost + parentToNode;
        }
        for(int i=0;i<(int)_tree[idx]->next.size(); i++) {
            open.push_back(_tree[idx]->next[i]);
        }
    }
}




/** fill 'dist' accroding to integer distribution in 'disti' */
void RRT2D::updateDistribution(const std::vector<int> &disti, std::vector<double> &dist) {
	int s = std::accumulate(disti.begin(),disti.end(),0);
	if (s == 0) {
		s = 1;
	}
	for(int i=0;i<(int)dist.size();i++) {
		dist[i] = disti[i] / (double)s;
	}
}




void RRT2D::updateDistribution(const std::vector< std::vector<int> > &disti, 
		std::vector< std::vector<double> > &dist) {
	int s = 0;
	for(int i=0;i<(int)disti.size();i++) {
		for(int j=0;j<(int)disti[i].size();j++) {
			s+= disti[i][j];
		}
	}

	for(int i=0;i<(int)dist.size();i++) {
		for(int j=0;j<(int)dist[i].size();j++) {
			dist[i][j] = (double)disti[i][j] / (double)s;
		}
	}
}





void RRT2D::_findNearestStateMultiple(const State &s, const int numNeighbors, vector< int > &output ) {


	int *bestIdx = new int[numNeighbors];
	int *bestIIdx = new int[numNeighbors];
	MPNN::ANNpoint bestDist = MPNN::annAllocPt(numNeighbors);

	MPNN::ANNpoint query = MPNN::annAllocPt(robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		query[i] = s[i];
	}
	_kdTree->NearestNeighbor(query,bestDist, bestIdx, bestIIdx);
    
    output.clear();
    output.reserve(numNeighbors);
    for(int i=0; i < numNeighbors; i++) {
        if (bestDist[i] != 0) {
            output.push_back(bestIIdx[i]);
        }
    }

    delete [] bestIdx;
    delete [] bestIIdx;
    MPNN::annDeallocPt(bestDist);
	MPNN::annDeallocPt(query);
}





/** dist should be AFTER sqrt */
int RRT2D::_findNearestState(KDTreeInt *tree, const State &s, double &dist) const {
	struct rusage t1,t2;

	int idx;
	dist = INFINITY;
	MPNN::ANNpoint query = MPNN::annAllocPt(robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		query[i] = s[i];
	}
	int nearest = (int)tree->NearestNeighbor(query,idx,dist);
	MPNN::annDeallocPt(query);
	return nearest;
}




/** return true if two configurations can be connected by straight line. The line is tested 
  * for 'n' points, where 'n = lenght/maxDistance'. lenght is lenght of the line */
bool RRT2D::canBeConnected(const State &s1, const State &s2, const double maxDistance) {

	//const double d = pointDistanceEucleid(robot->getRefPoint(s1),robot->getRefPoint(s2));
	const double d = sqrt(robot->distance(s1, s2));

	vector<State> states;
    approximateStates(s1,s2,(int)lround(1.0*d/maxDistance), states);

	double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
	RAPID_model *mmap = map->getRapidModel(rMap,vMap);
	RAPID_model *mrobot;

	bool isFree = true;
	for(int i=(int)states.size()-1;i >=0  && isFree;i--) {
		mrobot = robot->getRapidModel(rRobot,vRobot,states[i]);
		RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
		isFree = RAPID_num_contacts == 0;
	}

	return isFree;
}


/** return true if two configurations can be connected by straight line. The line is tested 
  * for 'n' points, where 'n = lenght/maxDistance'. lenght is lenght of the line 
  *
  * also report collision if found
  */
bool RRT2D::canBeConnectedWithContact(const State &s1, const State &s2, const double maxDistance, SContact &contact) {

	const double d = sqrt(robot->distance(s1, s2));

	vector<State> states;
    approximateStates(s1,s2,(int)lround(1.0*d/maxDistance), states);

	double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
	RAPID_model *mmap = map->getRapidModel(rMap,vMap);
	RAPID_model *mrobot;

	bool isFree = true;
	for(int i=(int)states.size()-1;i >=0  && isFree;i--) {
        //_randomPoints.push_back(states[i]);
		mrobot = robot->getRapidModel(rRobot,vRobot,states[i]);
		//RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
		RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_ALL_CONTACTS);  // !!!!!!!!!!!!!!!!!! all contacts
		isFree = RAPID_num_contacts == 0;
        if (!isFree) {
            contact.contactPairs.clear();
            contact.state = states[i];
            for(int j=0;j<(int)RAPID_num_contacts;j++) {
                contact.contactPairs.push_back(std::make_pair(RAPID_contact[j].id1, RAPID_contact[j].id2) );
            }
        }
	}


	return isFree;
}







/**
  * return path from RRT2D tree from node 'n' to node 'm', where
  * 'n' has nearest state to 'from' and
  * 'm' has nearest state to 'to'
  *
  W* nearest state is measured with metric defined in a robot (distance() method) 
  */
void RRT2D::getTrajectory(const State &fromState, const State &toState, std::vector<TNode> &traj) const {


    const int nearestFrom = findNearestState(_tree, fromState);
    const int nearestTo = findNearestState(_tree, toState);
    WDEBUG("from " << printString(fromState));
    WDEBUG("to " << printString(toState));
    WDEBUG("tree size " << _tree.size() << ", from idx " << nearestFrom << ", toidx " << nearestTo);

    traj.clear();

    if (nearestTo == -1 or nearestFrom == -1) {
        return;
    }

    vector< TNode * > tmpResult;

    int tmp = nearestTo;

	while(tmp != -1 && tmp != nearestFrom ) {
		tmpResult.push_back(_tree[ tmp ]);
		tmp = _tree[ tmp ]->parent;
	}

	if (tmp >= 0) {
		tmpResult.push_back(_tree[tmp]);
	}

    for(int i=(int)tmpResult.size()-1; i >= 0; i--) {
        traj.push_back( *tmpResult[i] );
        traj.back().parent = -1;
    }
    tmpResult.clear();

}


int RRT2D::findNearestState(const std::vector<TNode *> &nodes, const State &s) const {

    if (nodes.size() == 0) {
        return -1;
    }

	double bestDist = robot->distance(nodes.front()->state,s);
    int nearestidx = 0;

    for(int i=0;i<(int)nodes.size();i++) {
        const double d = robot->distance(nodes[i]->state, s);
        if (d < bestDist) {
            bestDist = d;
            nearestidx = i;
        }
    }
	return nearestidx;

}

void RRT2D::computeDistanceToRoot(vector<double> &nodesValues) const {
    nodesValues.clear();
    nodesValues = vector<double>(_tree.size(), -1);
    for(int i=0;i<(int)_tree.size();i++) {
        double s = 0;
        int tmp = i;
        while (tmp != -1) {
            if (nodesValues[tmp] != -1) {
                s += nodesValues[tmp];
                break;
            }
            if (_tree[tmp]->pstate.size() != 0) {
                const State &prevState = _tree[tmp]->pstate;
                const double dist = sqrt( robot->distance(_tree[tmp]->state, prevState) );
                s += dist;
            }
            tmp = _tree[tmp]->parent;
        }
        nodesValues[i] = s;
    }
}

double RRT2D::someFunction(const State &s, const vector<double> &nodesValues) {
    // mapping s -> R for testing derivation and newtom method
    // R is e.g. distance to root
    double distToNear = 0;
    const int nearIDX = _findNearestState(_kdTree, s, distToNear);

    if (nearIDX == -1) {
        return -1;
    }
    return distToNear + nodesValues[nearIDX];
}





} // namespace



