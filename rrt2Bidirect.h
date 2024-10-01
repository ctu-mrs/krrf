#ifndef _RRTBIDIRECT_H_
#define _RRTBIDIRECT_H_

#include "types.h"
#include "rrtstate.h"
#include "mapr.h"
#include "WLog.h"
#include "ut.h"
#include "rapid/RAPID.H"
#include "multiann.h"
#include "ANN.h"
#include "CStat.h"
#include <vector>
#include <list>
#include <fstream>
#include <algorithm>


namespace rrtPlanning {

using std::vector;
using std::list;
using std::ofstream;
using std::ifstream;

template<typename R>
class RRTBidirect {

	public:

	typedef R Robot;
	typedef typename Robot::State State;
    typedef MPNN::MultiANN<int> KDTreeInt;

	RRTBidirect(MapR *m, R *r):
		map(m), robot(r), distanceToGoalTh(-1),goalBias(-1),
		robotDistanceDimension(robot->getDistanceDim())
		{
			_initState.clear();
			_goalState.clear();
			_kdTreeF = NULL;
			_kdTreeB = NULL;
			_treeDistanceThreshold = 1;
            _resolution = 0.1;
            _inputResolution = 0.01;
            _expansionStep = 0.5;
		}


	~RRTBidirect() {
		map = NULL;
		robot = NULL;
		_initState.clear();
		_goalState.clear();
		if (_kdTreeF != NULL) {
			delete _kdTreeF;
			_kdTreeF = NULL;
		}
		if (_kdTreeB != NULL) {
			delete _kdTreeB;
			_kdTreeB = NULL;
		}
        deleteTree(_treeF);
        deleteTree(_treeB);
	}

	void generate(const State &fromState, const State &toState, const int size);

    
    
    
    
    int  _findNearestState(KDTreeInt *tree, const State &s, double &dist) const;

    void getTrajectory(const State &fromState, const State &toState, int &lastForwardNode, vector<TNode> &result  ) const;


    const std::vector<TNode *> &getTreeFwd() const { return _treeF; }    
    const std::vector<TNode *> &getTreeBack() const { return _treeB; }    

	void setEdgeTime(const double t);
	double getEdgeTime() const;

	void setEdgeSamples(const int s);
	int getEdgeSamples() const;
	void setGoalBias(const double gb);
	double getGoalBias() const;

	int getSamplesPerInput() const;
	void setSamplesPerInput(const int s);

	void setDistanceToGoalThreshold(const double th);
	double getDistanceToGoalThreshold() const;

	State getGoalState() const;
	State getInitState() const;

	int getTreeSize() const { return _treeF.size() + _treeB.size(); }
	int getTreeFSize() const { return _treeF.size();}
	int getTreeBSize() const { return _treeB.size(); }

	CStat getStatistic() const { return statistic; }
	void setTreeDistanceThreshold(const double td) { _treeDistanceThreshold = td; }
	double getTreeDistanceThreshold() const { return _treeDistanceThreshold; }

    void setExpansionStep(const double v) { _expansionStep = v; }
    double getExpansionStep() const { return _expansionStep; }
        
    private:


	MapR  *map;
	R *robot;
	double distanceToGoalTh;
	double goalBias;
	double _treeDistanceThreshold;
    double _inputResolution, _resolution;
    double _expansionStep;

	State _initState, _goalState;
    std::vector<TNode *> _treeF, _treeB;

    KDTreeInt *_kdTreeF;
    KDTreeInt *_kdTreeB;

	const int robotDistanceDimension;
	mutable CStat statistic;

	void clearStatistics();



    double getTreeDistance(int &nearestF, int &nearestB) const;

    void deleteTree(std::vector<TNode *> &tree) const;
    inline void addToKdTree(KDTreeInt *kdTree, const State &s, const int index);

    int _findNearestState(const State &s, int &treeIdx, int &nearestF, int &nearestB) const;
	int findNearestState(const std::vector<TNode *> &nodes, const State &s) const;


    void getReachableStatesDirectoGeometryGrowing(TNode *nearest, const int nearidx, const State &rstate, vector< TNode *> &newNodes);
    bool canBeConnected(const State &s1, const State &s2, const double maxDistance);
};



template<typename R>
int RRTBidirect<R>::findNearestState(const std::vector<TNode *> &nodes, const State &s) const {

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


/** gext next state which is on line tn,rstate.
  * this is faster than getReachableStates or getReachableStatesRandom which generates the reachable states using predefined set of inputs */
template<typename R>
void RRTBidirect<R>::getReachableStatesDirectoGeometryGrowing(TNode *nearest, const int nearidx, const State &rstate, vector< TNode *> &newNodes) {

    const State &nearState( nearest->state );
    const int num = sqrt( robot->distance(nearState, rstate) ) / _expansionStep;
    vector<State> line;
    if (num <= 2) {
        line.push_back(nearState);
        line.push_back(rstate);
    } else {
        approximateStates( nearState, rstate , num, line);
    }
    newNodes.clear();


    if (line.size() < 2) {
        return;
    }
    
    if (canBeConnected(nearState,line[1],_resolution)) {
        newNodes.push_back( new TNode() );
        newNodes.back()->RRT2D_setDefault();
        newNodes.back()->state = line[1];
        newNodes.back()->parent = nearidx;
        newNodes.back()->pstate = nearState;
    }
}






/** dist should be AFTER sqrt */
template<typename R>
int RRTBidirect<R>::_findNearestState(KDTreeInt *tree, const State &s, double &dist) const {
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



template<typename R>
inline void RRTBidirect<R>::addToKdTree(KDTreeInt *kdTree, const State &s, const int index) {
	MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		annpt[0][i] = s[i];
	}
	kdTree->AddPoint(annpt[0],index);
	MPNN::annDeallocPts(annpt);
}



template<typename R>
void RRTBidirect<R>::deleteTree(std::vector<TNode *> &tree) const {
    for(int i=0;i<(int)tree.size();i++) {
        if (tree[i]) {
            delete tree[i];
        }
    }
    tree.clear();
}




/** return distance of two trees
  * also return pair (ifF, ifB) which holds index between two nearest vertices in treeF and treeB
  */
template<typename R>
double RRTBidirect<R>::getTreeDistance(int &nearestF, int &nearestB) const {
	
	// brute force search
	nearestB = -1;
	nearestF = -1;
        
    double bestDist = -1;
    if (_treeF.size() < _treeB.size()) {
        for(int ti=0;ti<(int)_treeF.size();ti++) {
            double distToNear;
            const int nearIDX = _findNearestState(_kdTreeB, _treeF[ti]->state, distToNear);

            if (distToNear < bestDist || bestDist == -1) {
                bestDist = distToNear;
                nearestB = nearIDX;	
                nearestF = ti;
            }
        }
    } else {
        for(int ti=0;ti<(int)_treeB.size();ti++) {
            double distToNear;
            const int nearIDX = _findNearestState(_kdTreeF, _treeB[ti]->state, distToNear);
            if (distToNear < bestDist || bestDist == -1) {
                bestDist = distToNear;
                nearestF = nearIDX;	
                nearestB = ti;
            }
        }
    }
	return bestDist;
}

#if 0
/** return distance between treeF and treeB, this function is incremental and suppose
  * that previousNearestF, previousNearestB are actual and no new vertices were added to neither 
  * treeF not treeB 
  *
  * newAddedF .. iterator to vertex being added to treeF as last, or treeF.end() if freeF is unchanged
  * newAddedB ..  ------------- || ----------------             , or treeB.end();
  *
  * returns: distance of two nearest verices between trees and their iterators
  */
template<typename R>
double RRTBidirect<R>::getTreeDistance(
		Tree_iterator &previousNearestF, Tree_iterator &previousNearestB, 
		const Tree_iterator &newAddedF, const Tree_iterator &newAddedB) {

	
	const double oldDistance = robot->distance(previousNearestF->state, previousNearestB->state);
	double treeDist = oldDistance;
	
	if (newAddedF == treeF.end() && newAddedB != treeB.end()) {
		Tree_iterator near = findNearestStateInSubTree(newAddedB->state,0);
		const double dist = robot->distance(near->state,newAddedB->state);
		if (dist < oldDistance) {
			previousNearestF = near;
			previousNearestB = newAddedB;
			treeDist = dist;
		}
	} else if (newAddedF != treeF.end() && newAddedB == treeB.end()) {
		Tree_iterator near = findNearestStateInSubTree(newAddedF->state,1);
		const double dist = robot->distance(newAddedF->state, near->state);
		if (dist < oldDistance) {
			previousNearestF = newAddedF;
			previousNearestB = near;
			treeDist = dist;
		}

	} else {
		WDEBUG("incremental distance between tress cannost be computed with incremental algorithm");
		WDEBUG("slow linear search is used instead");
		treeDist = getTreeDistance(previousNearestF, previousNearestB);	
	}
	return treeDist;
}
#endif



/** this function should contain all possible values for staistic - it is important for header genration */
template<typename R>
void RRTBidirect<R>::clearStatistics() {
	statistic.zero();

	statistic["kdTree0SearchTime"] = 0;
	statistic["kdTree0BuildTime"] = 0;
	statistic["kdTree1SearchTime"] = 0;
	statistic["kdTree1BuildTime"] = 0;
	statistic["RRTTreeBuildTime"] = 0;
	statistic["collisionFalseTime"] = 0;
	statistic["collisionTrueTime"] = 0;
	statistic["collisionTestTime"] = 0;
	statistic["numberOfCollisionFalse"]=0;
	statistic["numberOfCollisionTrue"]=0;
	statistic["numberOfCollisionTests"]=0;
	statistic["linearSearchTime0"] = 0;
	statistic["linearSearchTime1"] = 0;
	statistic["realIteration"] = 0;
	

}


template<typename R>
void RRTBidirect<R>::generate(const State &fromState, const State &toState, const int size) {

	clearStatistics();

	struct rusage t1,t2;
	getTime(&t1);

	_initState = fromState;
	_goalState = toState;

    deleteTree(_treeF);
    _treeF.push_back(new TNode() );
    _treeF.back()->RRT2D_setDefault();
    _treeF.back()->state = fromState;

     deleteTree(_treeB);
    _treeB.push_back(new TNode() );
    _treeB.back()->RRT2D_setDefault();
    _treeB.back()->state = toState;

    const SDimension &mapDimension(map->getDimension());

    int lastAddedF = -1;
    int lastAddedB = -1;
    int previousNearestF = -1;
    int previousNearestB = -1;
	

	State randomState;
	// distance between trees and iterator of vertices that are close to B from F resp. from F to B
	double treeDistance = robot->distance(fromState,toState);

	if (_kdTreeF != NULL) {
		delete _kdTreeF;
		_kdTreeF = NULL;
	}


	if (_kdTreeB != NULL) {
		delete _kdTreeB;
		_kdTreeB = NULL;
	}

    _kdTreeF = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTreeF, _treeF[0]->state, 0);
    
    _kdTreeB = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTreeB, _treeB[0]->state, 0);

	struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);

	int treeIdx;
	int lastAdded = 0; // last expanded tree was F

	for(int iter = 0; iter < size; iter++) {

		getTime(&someTime);
		if (getTime(timeOfLastInfo,someTime) > 0.2) {
			std::cerr << "[" << iter << "/" << _treeF.size() <<':'<< _treeB.size() << "] ";
			getTime(&timeOfLastInfo);
		}

		if (treeDistance < _treeDistanceThreshold) {
			WDEBUG("trees reach each other to distance " << treeDistance);
			break;
		}

		if (robot->distance(toState, _treeF.back()->state) < distanceToGoalTh) {
			WDEBUG("fwd tree reaches the goal to distance " << robot->distance(toState,_treeF.back()->state));
			break;
		} 

		if (robot->distance(fromState, _treeB.back()->state) < distanceToGoalTh) {
			WDEBUG("back tree reaches the init state to distance " << robot->distance(fromState,_treeB.back()->state));
			break;
		} 

        if ((iter % 500) == 0) {
            int ntf, ntb;
	        const double treeDistance = getTreeDistance(ntf,ntb);	
            if (treeDistance < _treeDistanceThreshold) {
                if (ntf != -1 && ntb != -1 && canBeConnected(_treeF[ntf]->state, _treeB[ntf]->state,_resolution)) {
                    WDEBUG("Trees are close to each other, distance=" << treeDistance << ", th=" << _treeDistanceThreshold);
                    WDEBUG("Trees can be connected!");
                    break;
                }
            }
                
        }



		if (goalBias > 0 && getRandom(0,1) < goalBias) {
			randomState = toState;
		} else {
			randomState = robot->getRandomState(mapDimension);
		}

		int nearestF, nearestB;
		int nearIDX = _findNearestState(randomState,treeIdx,nearestF,nearestB);
	
        TNode *nearest = NULL;
		if (lastAdded == 0) {
			nearIDX = nearestB;
			treeIdx = 1;
            nearest = _treeB[ nearIDX];
		} else {
			nearIDX = nearestF;
			treeIdx = 0;
            nearest = _treeF[ nearIDX];
		}

		if (nearIDX != -1) {

            vector<TNode *> newNodes;
            
			getReachableStatesDirectoGeometryGrowing(nearest, nearIDX,randomState, newNodes);

            int nearestToRand = findNearestState(newNodes, randomState);

			if (nearestToRand != -1) {
				// add nearestNode to a tree
				if (treeIdx == 0) {

                    _treeF.push_back( newNodes[ nearestToRand ] );
                    addToKdTree(_kdTreeF, _treeF.back()->state, (int)_treeF.size()-1  );
                    newNodes[ nearestToRand ] = NULL;
                    lastAdded = 0;
                    
				} else {
                    _treeB.push_back( newNodes[ nearestToRand ] );
                    addToKdTree(_kdTreeB, _treeB.back()->state, (int)_treeB.size()-1  );
                    newNodes[ nearestToRand ] = NULL;
					lastAdded = 1;
				}
			}
            deleteTree(newNodes);
//			states.clear();
		} // n != tree.end()
		statistic["realIteration"] = iter;
		//realIteration = iter;
	}
//	cerr << size << " " << treeSize << "\n";

	getTime(&t2);
	statistic["RRTTreeBuildTime"] = getTime(t1,t2);
}


#if 0
template<typename R>
typename RRTBidirect<R>::Tree_iterator RRTBidirect<R>::findNearestStateInSubTree(const State &s, const int treeIdx) {

	double d,bestDist;

	Tree_iterator tbegin, tend, nearest;
	
	if (treeIdx == 0) {
		tbegin = treeF.begin();
		tend = treeF.end();
		bestDist = robot->distance(treeF.front().state,s);
		nearest = treeF.begin();
	} else {
		tbegin = treeB.begin();
		tend = treeB.end();
		bestDist = robot->distance(treeB.front().state,s);
		nearest = treeB.begin();
	}

	for(Tree_iterator tmp = tbegin; tmp != tend; ++tmp) {
			d = robot->distance(tmp->state,s);
			if (d < bestDist) {
				bestDist = d;
				nearest = tmp;
			}
	}

	return nearest;

}
#endif

#if 0

template<typename R>
typename RRTBidirect<R>::Const_tree_iterator RRTBidirect<R>::findNearestState(const State &s, const int treeIdx) const {

	double d,bestDist;
	struct rusage t1,t2;

	Const_tree_iterator tbegin, tend, nearest;
	if (treeIdx == 0) {
		tbegin = treeF.begin();
		tend = treeF.end();
		bestDist = robot->distance(treeF.front().state,s);
		nearest = treeF.begin();
	} else {
		tbegin = treeB.begin();
		tend = treeB.end();
		bestDist = robot->distance(treeB.front().state,s);
		nearest = treeB.begin();
	}

	getTime(&t1);
	for(Const_tree_iterator tmp = tbegin; tmp != tend; ++tmp) {
			d = robot->distance(tmp->state,s);
			if (d < bestDist) {
				bestDist = d;
				nearest = tmp;
			}
	}
	getTime(&t2);
	if (treeIdx == 0) {
		statistic["linearSearchTime0"] += getTime(t1,t2);
	} else {
		statistic["linearSearchTime1"] += getTime(t1,t2);
	}

	return nearest;

}
#endif







/** search forward and backward kd trees for nearest vertex to a given state s,
  * return iterator ti nearest vertex and type of the tree 
  */
template<typename R>
int RRTBidirect<R>::_findNearestState(const State &s, int &treeIdx, int &nearestF, int &nearestB) const {
    nearestF = -1;
    nearestB = -1;

	int idx;
	double dann = INFINITY;
	double dist1, dist2;
	MPNN::ANNpoint query = MPNN::annAllocPt(robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		query[i] = s[i];
	}
	nearestF = (int)_kdTreeF->NearestNeighbor(query,idx,dann);
	dist1 = robot->distance(s,_treeF[ nearestF ]->state);

	dann = INFINITY;
	nearestB = (int)_kdTreeB->NearestNeighbor(query,idx,dann);
	dist2 = robot->distance(s,_treeB[ nearestB ]->state);

	MPNN::annDeallocPt(query);

	int result = -1;
	if (dist1 <= dist2) { // byloa <=
		treeIdx = 0;
		result = nearestF;
	} else {
		treeIdx = 1;
		result = nearestB;
	}
	return result;

}





/** return true if two configurations can be connected by straight line. The line is tested 
  * for 'n' points, where 'n = lenght/maxDistance'. lenght is lenght of the line */
template<typename R>
bool RRTBidirect<R>::canBeConnected(const State &s1, const State &s2, const double maxDistance) {

	const double d = pointDistanceEucleid(robot->getRefPoint(s1),robot->getRefPoint(s2));

	vector<State> states;
    approximateStates(s1,s2,(int)lround(1.0*d/maxDistance), states);
//	WDEBUG("approximation by " << states.size() << " states");

	double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
	RAPID_model *mmap = map->getRapidModel(rMap,vMap);
	RAPID_model *mrobot;

	bool isFree = true;
	for(int i=0;i<(int)states.size() && isFree;i++) {
		mrobot = robot->getRapidModel(rRobot,vRobot,states[i]);
		RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
		isFree = RAPID_num_contacts == 0;
	}
	return isFree;
}



/*
template<typename R>
bool RRTBidirect<R>::getNextInputPermutation(vector<int> &perm, const vector<int> &inputSlices) const {
	// return true if input is valid, otherwise return false (last permutation was reached)
	
	int j = perm.size()-1;
	bool p = false;
	do {
		p = false;
		perm[j]++;
		if (perm[j] > inputSlices[j]) {
			perm[j] = 0;
			j--;
			p = true;
		}
	} while (p && j >= 0);

	if (p && j < 0)
		return false;

	return true;
}
*/



template<typename R>
double RRTBidirect<R>::getGoalBias() const {
	return goalBias;
}	

template<typename R>
void RRTBidirect<R>::setGoalBias(const double gb) {
	goalBias = gb;
}	


/**
  * return path from RRTBidirect tree from node 'n' to node 'm', where
  * 'n' has nearest state to 'from' and
  * 'm' has nearest state to 'to'
  *
  * nearest state is measured with metric defined in a robot (distance() method) 
  * also returns last index of node from first tree ('forward') tree,
  * so vertices [0 .. lastForwardNode] are from treeF, the rest if from treeB
  */
template<typename R>
void RRTBidirect<R>::getTrajectory(const State &fromState, const State &toState, int &lastForwardNode, vector<TNode> &result ) const {


    result.clear();
	//Const_tree_iterator ntf, ntb;
	//const Const_tree_iterator nearestToGoal = findNearestState(to,0);
	//const Const_tree_iterator nearestToInit = findNearestState(from,1);

    double distToGoal, distToInit;
    int nearestToGoal = _findNearestState(_kdTreeF, toState, distToGoal);
    int nearestToStart = _findNearestState(_kdTreeB, fromState, distToInit);

//	const double distanceToGoal = robot->distance(to, nearestToGoal->state);
//	const double distanceToInit = robot->distance(from, nearestToInit->state);

	// has at least one tree reached the goal?
	if (distToGoal < distanceToGoalTh || distToInit < distanceToGoalTh) {
		if (distToGoal <= distToInit) {
			// fwd tree reached the goal, search the trajectory in fwd tree
            int tmp = nearestToGoal;
			while(tmp >= 0 ) {
				result.push_back( *_treeF[tmp] );
                result.back().parent = -1;
                tmp = _treeF[ tmp]->parent;
			}
			std::reverse(result.begin(),result.end());
			lastForwardNode = result.size();
			return;
		} else {
			// back treee reach the init
            int tmp = nearestToStart;
			while(tmp >= 0 ) {
				result.push_back( *_treeB[ tmp ]);
                result.back().parent = -1;
                tmp = _treeB[ tmp]->parent;
			}
//			std::reverse(result.begin(),result.end());
			lastForwardNode = 0;
			return;
		}
	} 
    int ntf, ntb;
	const double treeDistance = getTreeDistance(ntf,ntb);	

	if (treeDistance > _treeDistanceThreshold) {
		lastForwardNode = 0;
		return;
	}

	vector<TNode> result1;
	int tmp = ntf;
	while(tmp >= 0 ) {
		result1.push_back(*_treeF[ tmp ]);
        result1.back().parent = -1;
        tmp = _treeF[tmp]->parent;
	}

	tmp = ntb;
	vector<TNode> result2;
	while(tmp >= 0) {
		result2.push_back( *_treeB[ tmp ] );
        result2.back().parent = -1;
        tmp = _treeB[ tmp ]->parent;
	}

	std::reverse(result1.begin(), result1.end());
	lastForwardNode = result1.size();
	for(int i=0;i<(int)result2.size();i++) {
		result1.push_back(result2[i]);
	}
    result = result1;
    result1.clear();
	result2.clear();
}



template<typename R>
void RRTBidirect<R>::setDistanceToGoalThreshold(const double th) {
	distanceToGoalTh = th;
}

template<typename R>
double RRTBidirect<R>::getDistanceToGoalThreshold() const {
	return distanceToGoalTh;
}







} // namespace


#endif

