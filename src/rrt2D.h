#ifndef _RRT_2_
#define _RRT_2_

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
#include "CRobot2D.h"

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

class RRT2D {

	public:

	typedef CRobot2D Robot;
	typedef typename Robot::State State;

    typedef MPNN::MultiANN<int> KDTreeInt;


    struct SContact {
        int treeNode;
        std::vector< std::pair<int, int> > contactPairs; // contactPairs[i].first (=map) and .second (robot) are IDs of map/robot geometry
        State state;

        SContact():treeNode(-1) {}
        SContact(const SContact &rhs):treeNode(rhs.treeNode), contactPairs(rhs.contactPairs), state(rhs.state) {}
        ~SContact(){
            contactPairs.clear();
        }   

    };



	RRT2D(MapR *m, Robot *r);

	~RRT2D();



	void generate(const State &from, const State &to, const int size);
    
    void generateAROcheckCD(const State &fromState, const State &toState, const int size);

	void generateWithFirstBias(const State &from, const State &to, const int size);
	void generateConnect(const State &fromState, const State &toState, const int size);
    void generateBlossom(const State &fromState, const State &toState, const int size);

    void generateAndReportCollisions(const State &fromState, const State &toState, const int size, std::vector< SContact > &contacts);

    void generateWitnessPlanning(const State &fromState, const State &toState, const int size, const int wsize);
    
    void generateWithHistory(const State &fromState, const State &toState, const int size, const double radiusT, const double radiusR, const int distSize, const double distanceToDistributionTH);

    void generateWitnessBFSPlanning(const State &fromState, const State &toState, const int size, const int wsize);


    void BFSRRTStep(const std::vector<State> &initStates, const State &toState, const int size, const double maxDistance);

    void branchedRRTStep(
        std::vector< std::vector< TNode *> > &localTrees, const std::vector< KDTreeInt * > &localTreesKD,
        const int treeIdx, 
        const State &toState, const int size, std::vector< State > &newRoots, std::vector<int> &newParents);

    void generateBFSRRT(const State &initState, const State &toState, const int size, const double minDistToKnown);
    void generateBranchedRRT(const State &initState, const State &toState, const int size, const double minDistToKnown, std::vector<State> &path);
    void generateTwoPlus(const State &fromState, const State &toState, const int size);

    void getAllChildrenOf(const std::vector<TNode *> &tree, const int predecessorIdx, std::vector<int> &idx) const;


    void generateOptimal(const State &fromState, const State &toState, const int size); // RRT*
    void generateSweptVolume(const State &fromState, const State &toState, const int size,
        const std::vector< Robot *> &robots,
        const std::vector< State > &motionPrimitives);

    void generateWithSamplingPrimitives(const State &fromState, const State &toState, const int size,
        const std::vector<SamplingPrimitive> &primitives);



    int  _findNearestState(KDTreeInt *tree, const State &s, double &dist) const;

    bool canBeConnected(const State &s1, const State &s2, const double maxDistance);

    double getCoverage(const State &s, std::vector<TAreaOfInterest> &areas, std::vector<double> &areasCoverage, 
            std::vector<double> &actualCoverage,
            std::vector<int> &robotContribution) const;

	
    void expandMotionPrimitives(
        const int nearidx, const State &rstate, std::vector< TNode *> &newNodes,
        const std::vector< Robot * > &robots,
        const std::vector<State> &motionPrimitives);





    void getTrajectory(const State &fromState, const State &toState, std::vector<TNode> &traj) const;

	void setGoalBias(const double gb) { goalBias = gb; }
	double getGoalBias() const { return goalBias; }
	void setDistanceToGoalThreshold(const double th) { distanceToGoalTh = th; }
	double getDistanceToGoalThreshold() const { return distanceToGoalTh; }
	State getGoalState() const { return _goalState; }
	State getInitState() const { return _initState; }
	int getTreeSize() const { return _tree.size();}
	CStat getStatistic() const { return _statistic;};

    void setExpansionStep(const double v) { _expansionStep = v; }
    double getExpansionStep() const { return _expansionStep; }
        
    const std::vector< TNode *> &getTree() const { return _tree; }

    std::vector<TNode> getCopyTree() const;

	Robot *robot;
    std::vector<TNode *> _tree;

	double _guidingSensitivity, _guidingDistance;

    inline void addToKdTree(KDTreeInt *kdTree, const State &s, const int index);

    void setOuterGoalDistance(const double val) { _outerGoalDistance = val; }
    double getOuterGoalDistance() const { return _outerGoalDistance; }
    
    State getOuterGoal() const { return _outerGoal; }
    std::list<State> _randomPoints;

    void setPainter(CPainterBase *pa) { _pa = pa; }

    void setCDResolution(const double val) { _resolution = val; }


    void computeDistanceToRoot(vector<double> &nodesValues) const;
    double someFunction(const State &s, const vector<double> &nodesValues);

	private:
	MapR *map;
	double distanceToGoalTh;
	double goalBias;
    double _outerGoalDistance;
	State _initState, _goalState;
    State _outerGoal;
    KDTreeInt *_kdTree;
    double _resolution, _inputResolution;
	const int robotDistanceDimension;
    double _expansionStep;

    CPainterBase *_pa;

    
    void deleteTree(std::vector<TNode *> &tree) const;
	int findNearestState(const std::vector<TNode *> &nodes, const State &s) const;


    void blossomPrunning(const std::vector<TNode *> &nodes, const State &parentState, std::vector< TNode *> &prunedNodes);
    bool blossomUseNode(const State &s, const State &parentState) const;

	void clearStatistics();
	void updateDistribution(const std::vector<int> &disti, std::vector<double> &dist);

	mutable CStat _statistic;	


	// just for testing purposes
	void updateDistribution(const std::vector< std::vector<int> > &disti, std::vector< std::vector<double> > &dist);


    void getVoronoi(const std::vector<TPoint> &inPts, std::vector< TPoint > &edges) const;

    double getCost(int startIndex) const;
    void _findNearestStateMultiple(const State &s, const int numNeighbors, std::vector< int > &output );
    void updateCostForward(const int startIdx);
    void getReachableStatesDirectoGeometryGrowing(const int nearidx, const State &rstate, std::vector< TNode *> &newNodes);
    void getReachableStatesDirectoGeometryGrowing(const std::vector<TNode *> &tree, const int nearidx, const State &rstate, std::vector< TNode *> &newNodes);
    void getReachableStatesDirectoGeometryGrowingWithContact(const std::vector<TNode *> &tree, const int nearidx, const State &rstate, std::vector< TNode *> &newNodes, State &lastFree);

    void expandNodeWithContacs(const int nearidx, const State &rstate, vector< TNode *> &newNodes, SContact &contact);

    bool canBeConnectedWithContact(const State &s1, const State &s2, const double maxDistance, SContact &contact);

    void getRandomOnLine(const State &from, const State &to, const double radius, State &result) const;

};


} // namespace

#endif


