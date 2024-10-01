#ifndef _RRT2D_control_h
#define _RRT2D_control_h

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

#include "painting.h"

/** implementation of several RRT algorithms for 2D robota (2D workspace). 
  Collision detection is realized using Rapid. 
  K-nn is realized using mpnn2 library
  
  * collision between robot and map can be computed for each dt on an edge all for only itas subset
  * see collisionCheckFreq variable 
  *
  * this class containts:
  * basic RRT (generate)
  * RRT-Connect (generateConnect)
  * RRT-Blossom (generateBlossom)
  * RRT-star (generateOptimal)

  *
  * and some testing version of rrt - see method generate*()
  */
namespace rrtPlanning {

template<typename R>
class RRT2DControl {

	public:

	typedef R Robot;
	typedef typename Robot::State State;
	typedef typename Robot::Input Input;

    typedef MPNN::MultiANN<int> KDTreeInt;

	RRT2DControl(MapR *m, R *r):
		robot(r), map(m), edgeTime(0), distanceToGoalTh(-1),edgeSamples(5), goalBias(-1),
		samplesPerInput(5), robotDistanceDimension(robot->getDistanceDim())
		{
			_initState.clear();
			_goalState.clear();
			_kdTree = NULL;
			collisionCheckFreq = 1;

			_guidingDistance = 150;
			_guidingSensitivity = 0.1;
            _randomPoints.clear();
            _outerGoalDistance = -1;
            _resolution = 5; // bylo 0.1
            _inputResolution = 0.01;
            _pa = NULL;
		}


	~RRT2DControl() {
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




	void generate(const State &from, const State &to, const int size);

	void generateWithFirstBias(const State &from, const State &to, const int size);
	void generateConnect(const State &fromState, const State &toState, const int size);
    void generateBlossom(const State &fromState, const State &toState, const int size);

    void generateOptimal(const State &fromState, const State &toState, const int size); // RRT*
    void generateSweptVolume(const State &fromState, const State &toState, const int size,
            const vector< R *> &robots,
        const vector< State > &motionPrimitives);

    void generateWithSamplingPrimitives(const State &fromState, const State &toState, const int size,
        const std::vector<SamplingPrimitive> &primitives);


    int  _findNearestState(KDTreeInt *tree, const State &s, double &dist) const;

    void getInputCombinations(std::vector<Input> &inputs) const;

    bool canBeConnected(const State &s1, const State &s2, const double maxDistance);

    double getCoverage(const State &s, std::vector<TAreaOfInterest> &areas, std::vector<double> &areasCoverage, 
            std::vector<double> &actualCoverage,
            std::vector<int> &robotContribution) const;

	
    vector<int> getInputSlices() const;




    void getTrajectory(const State &fromState, const State &toState, std::vector<TNode> &traj) const;

	void setEdgeTime(const double t) { edgeTime = t; }
	double getEdgeTime() const { return edgeTime; }
	void setEdgeSamples(const int s) { edgeSamples = s; }
	int getEdgeSamples() const { return edgeSamples; }
	void setGoalBias(const double gb) { goalBias = gb; }
	double getGoalBias() const { return goalBias; }
	int getSamplesPerInput() const { return samplesPerInput; }
	void setSamplesPerInput(const int s) { samplesPerInput = s; }
	void setDistanceToGoalThreshold(const double th) { distanceToGoalTh = th; }
	double getDistanceToGoalThreshold() const { return distanceToGoalTh; }
	State getGoalState() const { return _goalState; }
	State getInitState() const { return _initState; }
	int getTreeSize() const { return _tree.size();}
	CStat getStatistic() const { return statistic;};
	void setCollisionCheckFreq(const int f) { collisionCheckFreq = f; }
	int getCollisionCheckFreq() const { return collisionCheckFreq; }

        
    const std::vector< TNode *> &getTree() const { return _tree; }

	R *robot;
    std::vector<TNode *> _tree;

	double _guidingSensitivity, _guidingDistance;

    inline void addToKdTree(KDTreeInt *kdTree, const State &s, const int index);

    void setOuterGoalDistance(const double val) { _outerGoalDistance = val; }
    double getOuterGoalDistance() const { return _outerGoalDistance; }
    
    State getOuterGoal() const { return _outerGoal; }
    std::list<State> _randomPoints;

    void setPainter(CPainterBase *pa) { _pa = pa; }
    CPainterBase *_pa;

	private:
	MapR *map;
	double edgeTime, distanceToGoalTh;
	int edgeSamples;
	double goalBias;
    double _outerGoalDistance;
	int samplesPerInput;
	int collisionCheckFreq;
	State _initState, _goalState;
    State _outerGoal;
    KDTreeInt *_kdTree;
	int realIteration;
    double _resolution, _inputResolution;
	const int robotDistanceDimension;



    
    void deleteTree(std::vector<TNode *> &tree) const;
	int findNearestState(const std::vector<TNode *> &nodes, const State &s) const;


    bool isUniqueInput(const std::vector< TNode *> &tree, const int idx,  const Input &input, const double resolution) const;
    void blossomPrunning(const vector<TNode *> &nodes, const State &parentState, vector< TNode *> &prunedNodes);
    bool blossomUseNode(const State &s, const State &parentState) const;

	void clearStatistics();
	void updateDistribution(const std::vector<int> &disti, std::vector<double> &dist);

	mutable CStat statistic;	


	// just for testing purposes
	std::vector<int> getNeighborsCells(const int ix, const int iy, const std::vector<int> &numxslices, const std::vector<int> &numyslices);
	void updateDistribution(const std::vector< std::vector<int> > &disti, std::vector< std::vector<double> > &dist);
	std::vector<int> getNeighborsCells2(const int x, const int y, const std::vector< std::vector<int> > &disti);

	double getRobotBloomDistance(const State &initState) const;

    void getVoronoi(const std::vector<TPoint> &inPts, vector< TPoint > &edges) const;

    double getCost(int startIndex) const;
    void _findNearestStateMultiple(const State &s, const int numNeighbors, vector< int > &output );
    void updateCostForward(const int startIdx);
    void getReachableStatesDirectoGeometryGrowing(const int nearidx, const State &rstate, vector< TNode *> &newNodes);
    void getReachableStates(  const int nearIdx, std::vector< TNode * > &newNodes  ) const;

    void getRandomOnLine(const State &from, const State &to, const double radius, State &result) const;

};


template<typename R>
inline void RRT2DControl<R>::addToKdTree(KDTreeInt *kdTree, const State &s, const int index) {
	MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		annpt[0][i] = s[i];
	}
	kdTree->AddPoint(annpt[0],index);
	MPNN::annDeallocPts(annpt);
}





template<typename R>
void RRT2DControl<R>::deleteTree(std::vector<TNode *> &tree) const {
    for(int i=0;i<(int)tree.size();i++) {
        if (tree[i]) {
            delete tree[i];
        }
    }
    tree.clear();
}


template<typename R>
void RRT2DControl<R>::clearStatistics() {
	statistic.zero();

	statistic["RRTTreeBuildTime"] = 0;
	statistic["realIteration"] = 0;
	

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




template<typename R>
void RRT2DControl<R>::getVoronoi(const std::vector<TPoint> &inPts, vector< TPoint > &edges) const {
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

 
template<typename R>
bool RRT2DControl<R>::isUniqueInput(const std::vector< TNode *> &tree, const int idx,  const Input &input, const double resolution) const {

    for(int i=0;i<(int)tree[idx]->next.size();i++) {
        const int nextidx = tree[idx]->next[i];
        const Input &in(tree[ nextidx ]->input);
        if (in.size() == input.size()) {
            bool isSame = true;
            for(int j=0;j<(int)in.size() && isSame;j++) {
                if (fabs(in[j]-input[j]) > resolution) {
                    isSame = false;
                }
            }
            if (isSame) {
                //WDEBUG("inputs " << printString(input) << " and " << printString(in) << " are same!");
                return false;
            }
        }
    }

    return true;

}





template<typename R>
void RRT2DControl<R>::getReachableStates(  const int nearIdx, std::vector< TNode * > &newNodes  ) const {

	const double dt = edgeTime / (double)edgeSamples;
    using namespace std;

    TNode *tnear = _tree[ nearIdx ];

    newNodes.clear();

	vector<int> perm(robot->getInputSize(),0);
	vector<int> slices(getInputSlices());

	const vector<double> maxInputs(robot->getMaxInputValues());
	const vector<double> minInputs(robot->getMinInputValues());

	Input in(robot->getInputSize(),0);
	bool isFree;

	double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
	RAPID_model *mmap = map->getRapidModel(rMap,vMap);
	RAPID_model *mrobot;
	struct rusage t1,t2;

	while(getNextInputPermutation(perm, slices)) {
		for(int i=0;i<(int)perm.size();i++) {
			in[i] = minInputs[i] + perm[i]*( (maxInputs[i]-minInputs[i])/(double)slices[i] );
		}


		if (!robot->isInputValid(in) ||
			(1 && !isUniqueInput(_tree,  nearIdx ,in, _inputResolution)) ) {
			continue;
		}

		robot->setState(tnear->state);
		vector<State> states(robot->control(in,dt,edgeSamples));

		isFree = true;

		// check for self collision or for velocity constraints
		for(int i=0;i<(int)states.size() && isFree; i++) {
				isFree = robot->isValidState(states[i]);
		}

		// check for collision with obstacles
		if (isFree) {

			if (collisionCheckFreq == 0) {
				// check only last position
				mrobot = robot->getRapidModel(rRobot,vRobot,states.back());
				RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
				isFree = (RAPID_num_contacts == 0);
			} else if (collisionCheckFreq == 1) {
				// check all states
				for(int i=(int)states.size()-1; i>=0 && isFree; i--) {

                    mrobot = robot->getRapidModel(rRobot,vRobot,states[i]);
                    RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
                    isFree = (RAPID_num_contacts == 0);
				}

			} 
		}
	
		if (isFree) {
            newNodes.push_back( new TNode() );
            newNodes.back()->RRT2DControl_setDefault();
            newNodes.back()->RRT2DControl_time() = edgeTime;
            newNodes.back()->input = in;
            newNodes.back()->state = states.back();
            newNodes.back()->parent = nearIdx;
            newNodes.back()->pstate = tnear->state;
		} 

		states.clear();

	}
	perm.clear();
	slices.clear();
	in.clear();
	mmap = NULL;
	mrobot = NULL;
}






/* basic RRT */
template<typename R>
void RRT2DControl<R>::generate(const State &fromState, const State &toState, const int size) {

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);


	_initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2DControl_setDefault();
    _tree.back()->state = fromState;

	if (_kdTree != NULL) {
		delete _kdTree;
		_kdTree = NULL;
	}

    _kdTree = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTree, _tree[0]->state, 0);

    const SDimension &mapDimension(map->getDimension());

    if (0) {
        // test metrix
        WDEBUG("Testing metric!");
        for(int i=0;i<10;i++) {
			State randomState = robot->getRandomState(mapDimension);

            double distToNear;
            const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

            const double rdist = sqrt( robot->distance(randomState, _tree[nearIDX]->state));

            WDEBUG("step " << i << " kdtree-dist=" << distToNear << ", robot-dist=" << rdist << ", diff= " << (rdist-distToNear));

        }
//        exit(0);
    }


	realIteration = 0;
	State randomState;


	struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);

//    ofstream ofs("paa.rrtpts");
    //const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;
    const double distanceToGoalTh2= distanceToGoalTh >=0 ?  distanceToGoalTh*distanceToGoalTh : -1;

    std::map< int, int> node2iteration; // key is node idx, it's value is the iteration when it was created
    std::map< double, int> node2iterationHistogram; // key is iteration ater being created, value is the count
    node2iteration[0] = 0;

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

        double distToNear;
        const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

		if (nearIDX >= 0) {
                
            if (0)  {
//                const int afteriteration = iter - node2iteration[nearIDX];
                const double afteriteration = 1.0*(_tree.size() - nearIDX) / iter;
                node2iterationHistogram[afteriteration]++;
             //   std::cout << "DWW " <<  nearIDX << " " << node2iteration[nearIDX] << " " << iter - node2iteration[nearIDX] << "\n";
            }
            vector<TNode *> newNodes;
            
                getReachableStates(nearIDX, newNodes );

			
            int nearestToRand = findNearestState(newNodes, randomState);


			if (nearestToRand != -1) {
				// add nearestNode to a tree
				_tree.push_back( newNodes[ nearestToRand ] );
                addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                newNodes[ nearestToRand ] = NULL;

                //node2iteration[ (int)_tree.size()-1 ] = iter;

                    _tree[nearIDX]->next.push_back( (int)_tree.size()-1);
//                WDEBUG("next[" << nearIDX << "]=" << printString(_tree[nearIDX]->next));
                
			}
            deleteTree(newNodes);
		}
        if (1 && (iter % 100) == 0 && _pa) {
            //WDEBUG("drawing trajectory");
            _pa->begin();
            drawMap(*map,_pa);
            drawNodeTreeControl(_tree,*robot,_pa, CColor("green4"), getEdgeSamples());
            draw(robot->getRefPoint(fromState),5,1,CColor("black"),CColor("blue"),_pa);
            draw(robot->getRefPoint(toState),5,1,CColor("black"),CColor("cyan"),_pa);
            _pa->close();
        }
		statistic["realIteration"] = iter;
	}
	getTime(&t2);
	statistic["RRTTreeBuildTime"] = getTime(t1,t2);
	WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
    statistic["treeSize"] = _tree.size();

    if (0) {
        for(typename std::map<double,int>::iterator i = node2iterationHistogram.begin(); i != node2iterationHistogram.end();i++) {
            const double key = i->first;
            const int count = i->second;
            std::cout << "DWWH " << key << " " << count << "\n";
        }
    }

}







/* basic RRT, only the first sample is generated in the direction of goal. Just to demonstrate ARO students how big effect
   it can have. On potholes -problem 300 and 301 */
template<typename R>
void RRT2DControl<R>::generateWithFirstBias(const State &fromState, const State &toState, const int size) {

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);


	_initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2DControl_setDefault();
    _tree.back()->state = fromState;

	if (_kdTree != NULL) {
		delete _kdTree;
		_kdTree = NULL;
	}

    _kdTree = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTree, _tree[0]->state, 0);

    const SDimension &mapDimension(map->getDimension());

    if (1) {
        // test metrix
        WDEBUG("Testing metric!");
        for(int i=0;i<10;i++) {
			State randomState = robot->getRandomState(mapDimension);

            double distToNear;
            const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

            const double rdist = sqrt( robot->distance(randomState, _tree[nearIDX]->state));

            WDEBUG("step " << i << " kdtree-dist=" << distToNear << ", robot-dist=" << rdist << ", diff= " << (rdist-distToNear));

        }
//        exit(0);
    }


	realIteration = 0;
	State randomState;


	struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);

//    ofstream ofs("paa.rrtpts");
    //const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;
    const double distanceToGoalTh2= distanceToGoalTh >=0 ?  distanceToGoalTh*distanceToGoalTh : -1;

    std::map< int, int> node2iteration; // key is node idx, it's value is the iteration when it was created
    std::map< double, int> node2iterationHistogram; // key is iteration ater being created, value is the count
    node2iteration[0] = 0;

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
    
        if (iter < 2) {
            WDEBUG("First bias!  ============================================================== ");
			randomState = toState;
            WDEBUG("First bias!  ============================================================== ");

        }

        double distToNear;
        const int nearIDX = _findNearestState(_kdTree, randomState, distToNear);

		if (nearIDX >= 0) {
                
            if (0)  {
//                const int afteriteration = iter - node2iteration[nearIDX];
                const double afteriteration = 1.0*(_tree.size() - nearIDX) / iter;
                node2iterationHistogram[afteriteration]++;
             //   std::cout << "DWW " <<  nearIDX << " " << node2iteration[nearIDX] << " " << iter - node2iteration[nearIDX] << "\n";
            }
            vector<TNode *> newNodes;
            
            getReachableStates(nearIDX, newNodes );

			
            int nearestToRand = findNearestState(newNodes, randomState);


			if (nearestToRand != -1) {
				// add nearestNode to a tree
				_tree.push_back( newNodes[ nearestToRand ] );
                addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                newNodes[ nearestToRand ] = NULL;

                //node2iteration[ (int)_tree.size()-1 ] = iter;

                    _tree[nearIDX]->next.push_back( (int)_tree.size()-1);
//                WDEBUG("next[" << nearIDX << "]=" << printString(_tree[nearIDX]->next));
                
			}
            deleteTree(newNodes);
		} 
		statistic["realIteration"] = iter;
	}
	getTime(&t2);
	statistic["RRTTreeBuildTime"] = getTime(t1,t2);
	WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
    statistic["treeSize"] = _tree.size();

    if (0) {
        for(typename std::map<double,int>::iterator i = node2iterationHistogram.begin(); i != node2iterationHistogram.end();i++) {
            const double key = i->first;
            const int count = i->second;
            std::cout << "DWWH " << key << " " << count << "\n";
        }
    }

}











template<typename R>
void RRT2DControl<R>::getRandomOnLine(const State &from, const State &to, const double radius, State &result) const {
    const double t = getRandom(0,1);

    State s(robot->getStateSize(),0);
    for(int i=0;i<(int)s.size();i++) {
        s[i] = from[i]*(1-t) + to[i]*(t);
    }

    result[0] = getRandomGauss(s[0], radius);
    result[1] = getRandomGauss(s[1], radius);
    result[2] = getRandomGauss(s[2], M_PI/4);

}





template<typename R>
void RRT2DControl<R>::generateConnect(const State &fromState, const State &toState, const int size) {

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);


	_initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2DControl_setDefault();
    _tree.back()->state = fromState;


    const SDimension &mapDimension(map->getDimension());
	

	realIteration = 0;
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

                    getReachableStates(nearIDX, newNodes );

                const int nearestToRand = findNearestState(newNodes, randomState);

                if (nearestToRand != -1) {
                    const double distToRand = robot->distance(randomState, newNodes[ nearestToRand ]->state);
                    if ( distToRand > 0.01) {
                        // add nearestNode to a tree
                        _tree.push_back( newNodes[ nearestToRand ] );
                        addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                        wasAdded = true;
                        newNodes[ nearestToRand ] = NULL;
                            _tree[nearIDX]->next.push_back( (int)_tree.size()-1);
                        nearIDX = (int)_tree.size()-1;
                    }
                }
                deleteTree(newNodes);
            } while (wasAdded);
		} 
		statistic["realIteration"] = iter;
	}
	getTime(&t2);
	statistic["RRTTreeBuildTime"] = getTime(t1,t2);
	WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
}






/** traverse from startIndex to tree[0] and report total cost at startIndex */
template<typename R>
double RRT2DControl<R>::getCost(int startIndex) const {

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


template<typename R>
bool RRT2DControl<R>::blossomUseNode(const State &s, const State &parentState) const {
    // return true if the node is NOT growing into the tree
		
    const double distanceToParent = robot->distance(s, parentState);
    double distToNear; // after sqrt
    const int nearIDX = _findNearestState(_kdTree, s, distToNear);
    return (distToNear*distToNear >= distanceToParent);
}

template<typename R>
void RRT2DControl<R>::blossomPrunning(const vector<TNode *> &nodes, const State &parentState, vector< TNode *> &prunedNodes) {

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






template<typename R>
void RRT2DControl<R>::generateBlossom(const State &fromState, const State &toState, const int size) {

    clearStatistics();

    struct rusage t1,t2;
    getTime(&t1);


    _initState = fromState;
    _goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2DControl_setDefault();
    _tree.back()->state = fromState;


    const SDimension &mapDimension(map->getDimension());


    realIteration = 0;
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

                getReachableStates(nearIDX, newNodes );

            int nearestToRand = findNearestState(newNodes, randomState);

            for(int i=0;i<(int)newNodes.size();i++) {
                if (blossomUseNode(newNodes[i]->state, _tree[ nearIDX ]->state) || i == nearestToRand) {
                    // add nearestNode to a tree
                    _tree.push_back( newNodes[ i ] );
                    addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                    newNodes[ i ] = NULL;
                        _tree[nearIDX]->next.push_back( (int)_tree.size()-1);
                }
            }
            deleteTree(newNodes);
        } 
        statistic["realIteration"] = iter;
    }
    getTime(&t2);
    statistic["RRTTreeBuildTime"] = getTime(t1,t2);
    WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
}





















/** basic RRT* - RRT-star
  * a node stores: cumulativeLength = distance in the tree from start to node
  *
  * after a new node is introduced, it's neighbor is search and rewired. rewiring a node 'x' to have 'y' as parent if:
  * d = distance
    d(start,y) + d(y,x) < di(start, x) 
  */
template<typename R>
void RRT2DControl<R>::generateOptimal(const State &fromState, const State &toState, const int size) {
	
    CPainterBase *pa = new CPainterCairo("rrsa",CPainterCairo::PNG,map->getDimension().getVector());
    pa = NULL;

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);


    _initState = fromState;
	_goalState = toState;

    deleteTree(_tree);
    _tree.push_back(new TNode() );
    _tree.back()->RRT2DControlStar_setDefaults();
    _tree.back()->state = fromState;


    const SDimension &mapDimension(map->getDimension());
	
	realIteration = 0;
	State randomState;

    Input maxv(robot->getMinInputValues());
    const double maxstep = fabs(maxv[0]);



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
    Input emptyInput(robot->getInputSize(),0);
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

            getReachableStates(nearIDX, newNodes );

            int nearestToRand = findNearestState(newNodes, randomState);

            if (nearestToRand != -1) {
                // add nearestNode to a tree
                //				_tree.push_back( newNodes[ nearestToRand ] );
                //                _tree.back()->RRT2DControlStar_cost() = getCost( (int)_tree.size()-1);
                //                _tree.back()->RRT2DControlStar_time() = edgeTime;
                //                addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );
                //                newNodes[ nearestToRand ] = NULL;
                //                deleteTree(newNodes);
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
                    const double cost = _tree [idx]->RRT2DControlStar_getCost() + distToNew;
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
                    approximateStates(_tree[ parent ]->state,qnew->state,(int)lround(2.0*d/maxstep), states);
                }

                for(int ssi = 0; ssi < (int)states.size();ssi++) {

                    _tree.push_back( new TNode() );
                    _tree.back()->RRT2DControlStar_setDefaults();
                    _tree.back()->state = states[ssi];
                    _tree.back()->parent = parent;
                    _tree.back()->pstate = _tree[ parent ]->state;
                    _tree.back()->input = emptyInput;
                    _tree[ parent ]->next.push_back((int) _tree.size() - 1 );
                    inputFromStates(_tree.back()->pstate, _tree.back()->state, edgeTime, _tree.back()->input, edgeTime);
                    _tree.back()->RRT2DControlStar_time() = edgeTime;

                    const double dtopar = robot->distance(_tree.back()->pstate, _tree.back()->state);
                    _tree.back()->RRT2DControlStar_cost() = _tree[ _tree.back()->parent ]->RRT2DControlStar_getCost() + sqrt(dtopar);

                    //_tree.back()->RRT2DControlStar_cost() = getCost( (int)_tree.size()-1);

                    addToKdTree(_kdTree, _tree.back()->state, (int)_tree.size()-1  );


                    parent = (int)_tree.size()-1;


                    // now rewiring
                    if (1 && _tree.size() > nearestNeighbors+2) {
                        vector<int> nearNodes;
                        _findNearestStateMultiple(_tree.back()->state, nearestNeighbors, nearNodes);
                        //const double lastCost = getCost( (int)_tree.size()-1);
                        const double lastCost = _tree[ parent ]->RRT2DControlStar_getCost();

                        for(int i=0;i<(int)nearNodes.size();i++) {
                            const int candidateIdx = nearNodes[i]; 
                            //                            const double oldcost = getCost(candidateIdx);
                            const double oldcost = _tree[ candidateIdx ]->RRT2DControlStar_getCost();
                            const double newcost = lastCost + sqrt(robot->distance(_tree.back()->state, _tree[ candidateIdx]->state));

                            if (newcost < oldcost) {
                                if (canBeConnected( _tree.back()->state, _tree[candidateIdx]->state, _resolution)) {

                                    TNode *c = _tree[ candidateIdx];
                                    // remove candidateIdx from it's parent's next
                                    if (c->parent != -1) {
                                        _tree[ c->parent ]->RRT2DControlStar_removeNextIdx( candidateIdx );
                                    }

                                    c->parent = (int)_tree.size()-1;
                                    _tree[ c->parent ]->next.push_back(candidateIdx);

                                    c->RRT2DControlStar_cost() = newcost;
                                    c->pstate = _tree.back()->state;
                                    c->input = emptyInput;
                                    inputFromStates(c->pstate, c->state, edgeTime, c->input, edgeTime);
                                    updateCostForward( c->parent );
                                }
                            }
                        }
                    }
                }

            }
		} // n != tree.end()
		statistic["realIteration"] = iter;
	}
	getTime(&t2);
	statistic["RRTTreeBuildTime"] = getTime(t1,t2);
	WDEBUG("distance to goal state is " << robot->distance(_tree.back()->state,toState));
//    WDEBUG("--------- end of " << __FUNCTION__ << " ----- ");
}



template<typename R>
void RRT2DControl<R>::updateCostForward(const int startIdx) {

    vector<int> open;
    open.reserve(5000);
    open.push_back(startIdx);

    while(open.size() > 0) {
        const int idx = open.back();
        open.pop_back();
        //WDEBUG("updating, open.size=" << open.size() << ", actual=" << idx);

        if (_tree[ idx ]->parent != -1) {
            const double parentCost = _tree[ _tree[ idx ]->parent ]->RRT2DControlStar_getCost();
            const double parentToNode = sqrt( robot->distance( _tree[ _tree[idx]->parent]->state, _tree[idx]->state) );
            _tree[ idx ]->RRT2DControlStar_cost() = parentCost + parentToNode;
        }
        for(int i=0;i<(int)_tree[idx]->next.size(); i++) {
            open.push_back(_tree[idx]->next[i]);
        }
    }
}




/** fill 'dist' accroding to integer distribution in 'disti' */
template<typename R>
void RRT2DControl<R>::updateDistribution(const std::vector<int> &disti, std::vector<double> &dist) {
	int s = std::accumulate(disti.begin(),disti.end(),0);
	if (s == 0) {
		s = 1;
	}
	for(int i=0;i<(int)dist.size();i++) {
		dist[i] = disti[i] / (double)s;
	}
}



template<typename R>
std::vector<int> RRT2DControl<R>::getNeighborsCells(const int ix, const int iy, 
		const std::vector<int> &numxslices, const std::vector<int> &numyslices) {
	// return 4-neighborhood for given cell
	std::vector<int> result;

	for(int i=ix-1;i<=ix+1;i++) {
		for(int y=iy-1;y<=iy+1;y++) {
			if ( (i >= 0) && (i < (int)numxslices.size()) && (y>=0) && (y < (int)numyslices.size()) ) {
				result.push_back(numxslices[i]*numyslices[y]);			
			}
		}
	}
	return result;
}






template<typename RT>
void RRT2DControl<RT>::updateDistribution(const std::vector< std::vector<int> > &disti, 
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


template<typename RT>
std::vector<int> RRT2DControl<RT>::getNeighborsCells2(const int x, const int y, const std::vector< std::vector<int> > &disti) {
	std::vector<int> result;

	for(int i=x-1;i<=x+1;i++) {
		for(int j=y-1;j<=y+1;j++) {
			if (i >= 0 && i < (int)disti.size() && j >= 0 && j< (int)disti[i].size()) {
				result.push_back(disti[i][j]);
			}
		}
	}
	return result;
}












template<typename R>
void RRT2DControl<R>::_findNearestStateMultiple(const State &s, const int numNeighbors, vector< int > &output ) {


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
template<typename R>
int RRT2DControl<R>::_findNearestState(KDTreeInt *tree, const State &s, double &dist) const {
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






/** determine max distance between a inital node and its most distance child */
template<typename R>
double RRT2DControl<R>::getRobotBloomDistance(const State &initState) const {
	vector<int> perm(robot->getInputSize(),0);
	vector<int> slices(robot->getInputSize(),0);
	for(int i=0;i<(int)slices.size()/2;i++) {
		slices[2*i+0] = 1;
		slices[2*i+1] = 2;
	}
	const vector<double> maxInputs(robot->getMaxInputValues());
	const vector<double> minInputs(robot->getMinInputValues());

	Input in(robot->getInputSize(),0);
	//bool isFree;

	const double dt = edgeTime / (double)edgeSamples;
	double maxDistance = -1;

	while(getNextInputPermutation(perm, slices)) {

		for(int i=0;i<(int)perm.size();i++) {
			if (slices[i] > 0) {
				in[i] = minInputs[i] + perm[i]*( (maxInputs[i]-minInputs[i])/(double)slices[i] );
			} else {
				in[i] = maxInputs[i];
			}
		}

		robot->setState(initState);
		vector<State> states(robot->control(in,dt,edgeSamples));

		const double d = robot->distance(initState, states.back());
		if (d > maxDistance) {
			maxDistance = d;
		}
	}
	return maxDistance;
}




/** compute all input combinations. Just for testing purpose */
template<typename R>
void RRT2DControl<R>::getInputCombinations(std::vector<Input> &inputs) const {

	const double dt = edgeTime / (double)edgeSamples;

	vector<int> perm(robot->getInputSize(),0);
	vector<int> slices(getInputSlices());

	const vector<double> maxInputs(robot->getMaxInputValues());
	const vector<double> minInputs(robot->getMinInputValues());

	Input in(robot->getInputSize(),0);

	while(getNextInputPermutation(perm, slices)) {

		for(int i=0;i<(int)perm.size();i++) {
			in[i] = minInputs[i] + perm[i]*( (maxInputs[i]-minInputs[i])/(double)slices[i] );
		}
        inputs.push_back(in);
    }
}



/** return true if two configurations can be connected by straight line. The line is tested 
  * for 'n' points, where 'n = lenght/maxDistance'. lenght is lenght of the line */
template<typename R>
bool RRT2DControl<R>::canBeConnected(const State &s1, const State &s2, const double maxDistance) {

	//const double d = pointDistanceEucleid(robot->getRefPoint(s1),robot->getRefPoint(s2));
	const double d = sqrt(robot->distance(s1, s2));

	vector<State> states;
    approximateStates(s1,s2,(int)lround(1.0*d/maxDistance), states);
//	WDEBUG("approximation by " << states.size() << " states");

	double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
	RAPID_model *mmap = map->getRapidModel(rMap,vMap);
	RAPID_model *mrobot;

	bool isFree = true;
    /*
  //  int cnt = 0;
	for(int i=0;i<(int)states.size() && isFree;i++) {
        _randomPoints.push_back(states[i]);
		mrobot = robot->getRapidModel(rRobot,vRobot,states[i]);
		RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
		isFree = RAPID_num_contacts == 0;
    //    cnt++;
	}
    */
	for(int i=(int)states.size()-1;i >=0  && isFree;i--) {
        _randomPoints.push_back(states[i]);
		mrobot = robot->getRapidModel(rRobot,vRobot,states[i]);
		RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
		isFree = RAPID_num_contacts == 0;
    //    cnt++;
	}


//    WDEBUG("tested " << cnt);
	return isFree;
}




template<typename R>
vector<int> RRT2DControl<R>::getInputSlices() const {
	return vector<int>(robot->getInputSize(),samplesPerInput);
}




/**
  * return path from RRT2DControl tree from node 'n' to node 'm', where
  * 'n' has nearest state to 'from' and
  * 'm' has nearest state to 'to'
  *
  W* nearest state is measured with metric defined in a robot (distance() method) 
  */
template<typename R>
void RRT2DControl<R>::getTrajectory(const State &fromState, const State &toState, std::vector<TNode> &traj) const {


    const int nearestFrom = findNearestState(_tree, fromState);
    const int nearestTo = findNearestState(_tree, toState);

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


template<typename R>
int RRT2DControl<R>::findNearestState(const std::vector<TNode *> &nodes, const State &s) const {

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







} // namespace

#endif


