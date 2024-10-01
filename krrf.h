#ifndef _KRRF_H_
#define _KRRF_H_

#include "types.h"
#include "rrtstate.h"
#include "mapr.h"
#include "WLog.h"
#include "ut.h"
#include "libs/rapid/RAPID.H"
#include "multiann.h"
#include "CStat.h"
#include "ANN.h"
#include "CStat.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <flann/flann.hpp>
#include <random>
#include <bits/stdc++.h>
#include <vector>
#include <list>
#include <fstream>
#include <algorithm>
#include <ctime>
#include <string>

#include "painting.h"

namespace rrtPlanning { //general namespacing of all planning algos

using std::vector;
using std::list;
using std::ofstream;
using std::ifstream;
using std::priority_queue;
using namespace flann;

class Q_Node {
    public:
    double q_cost;
    int fwd_index;
    TNode * node;
};

struct CompareQ_Node {
    bool operator()(Q_Node const & p1, Q_Node const & p2) {
        // return "true" if "p1" is ordered before "p2"
        return p1.q_cost > p2.q_cost;
    }
};

typedef std::pair<double, int> QueuePoint; //priority and index to fwd tree

class Pqueue {
    public:
    Pqueue() {
        size = -1;
    }
    std::vector<QueuePoint> vec{};
    // Function to return the index of the
    // parent node of a given node
    int size;
    std::map<int, int> pos2idx; //position to index mapping
    int parent(int i)
    {
        return (i - 1) / 2;
    }
    
    // Function to return the index of the
    // left child of the given node
    int leftChild(int i)
    {
        return ((2 * i) + 1);
    }
    
    // Function to return the index of the
    // right child of the given node
    int rightChild(int i)
    {
        return ((2 * i) + 2);
    }
    
    // Function to shift up the node in order
    // to maintain the heap property
    void shiftUp(int i)
    {
        while (i > 0 && vec[parent(i)] < vec[i]) {
    
            // Swap parent and current node
            int pi = parent(i);
            pos2idx[vec[pi].second] = i;
            pos2idx[vec[i].second] = pi;

            swap(vec[pi], vec[i]);

    
            // Update i to parent of i
            i = parent(i);
        }
    }
    
    // Function to shift down the node in
    // order to maintain the heap property
    void shiftDown(int i)
    {
        int maxIndex = i;
    
        // Left Child
        int l = leftChild(i);
    
        if (l <= size && vec[l] > vec[maxIndex]) {
            maxIndex = l;
        }
    
        // Right Child
        int r = rightChild(i);
    
        if (r <= size && vec[r] > vec[maxIndex]) {
            maxIndex = r;
        }
    
        // If i not same as maxIndex
        if (i != maxIndex) {
            pos2idx[vec[i].second] = maxIndex;
            pos2idx[vec[maxIndex].second] = i;
            swap(vec[i], vec[maxIndex]);
            shiftDown(maxIndex);
        }
    }
    
    // Function to insert a new element
    // in the Binary Heap
    void insert(QueuePoint p)
    {
        size = size + 1;
        vec.push_back(p);
        pos2idx[p.second] = size;
    
        // Shift Up to maintain heap property
        shiftUp(size);
    }
    
    // Function to extract the element with
    // maximum priority
    QueuePoint extractMax()
    {
        QueuePoint result = vec[0];
        // Replace the value at the root
        // with the last leaf
        vec[0] = vec[size];
        size = size - 1;
        // Shift down the replaced element
        // to maintain the heap property
        vec.pop_back();
        shiftDown(0);
        //WDEBUG("Queue extracting " << result.first << " " << result.second << " queue size " << size << "\n");
        pos2idx.erase(result.second);
        return result;
    }
    
    // Function to change the priority
    // of an element on position i in _treeF and priority p
    void changePriority(int i, int p) //position in ftree and priority
    {
        int q_idx = pos2idx[i];
        //WDEBUG("Qindex " << q_idx << " " << size);
        QueuePoint oldp = vec[q_idx];
        vec[q_idx].first = p;
    
        if (p > oldp.first) {
            shiftUp(q_idx);
        }
        else {
            shiftDown(q_idx);
        }
    }

    //Function to print the queue
    void printQueue()
    {
        int j = 0;
        while (j <= size) {
            cout << "(" << vec[j].first << " " << vec[j].second << ")\n";
            j++;
        }
        cout << "\n";
    }
};

template<typename R>
class KRRF { //so far only a 2D version
    
    public: //here are public methods and variables
    
    typedef R Robot;
	typedef typename Robot::State State;
    typedef typename Robot::Input Input;
    typedef MPNN::MultiANN<int> KDTreeInt;
    typedef flann::Index<L2<double> > FlannTree;

    KRRF(MapR *m, R *r):
    map(m), robot(r), distanceToGoalTh(-1),goalBias(-1), edgeTime(0), edgeSamples(5),
    samplesPerInput(5), robotDistanceDimension(robot->getDistanceDim())
    {
        _initState.clear();
        _goalState.clear();
        //_kdTreeF = NULL;
        //_kdTreeB = NULL;
        collisionCheckFreq = 1;

        _guidingDistance = 100;
        _treeDistanceThreshold = 1;
        _resolution = 0.1;
        _inputResolution = 0.01;
        _expansionStep = 0.5;
        _N_mcp = 100;
        T_max = 2;
        r_k_input = 50;
        timeResolution = 0.1;
        animate = true;
        amax = 1;
    }


	~KRRF() {
		map = NULL;
		robot = NULL;
		_initState.clear();
		_goalState.clear();
		
        if (_tree.size() != 0) {
			deleteTree(_tree);
			
		}
		if (index.size() != 0) {
			deleteFlann(index);
			
		}
        if (_tree_TSP.size() != 0) {
            deleteTree(_tree_TSP);
            
		}
		if (index_TSP.size() != 0) {
			deleteFlann(index_TSP);
			
		}
	}

    double distance(Q_Node qnode1, Q_Node qnode2);

	void generate(const std::vector<State> &init_states, const int size);
    bool generate_TSP_guiding(vector<int> order, const int size);
    void generate_TSPTree(vector<int> order, const int size);
    void generate_TSPTree2(vector<int> order, const int size);

    void deleteFlann(std::vector<FlannTree *> &index) const;
    void init_flann(std::vector<FlannTree *> & index) const;
    void init_flann(std::vector<FlannTree *> & index, int length) const;
    void add_next_index(std::vector<FlannTree *> & index, const State &st, int idx) const;

    int _findNearestState(KDTreeInt *tree, const State &s, double &dist);
    int _findNearestStateFlann(Index<L2<double> > *index, const State &s, double &dist);
    void _findNearestStateFlannToPath(Index<L2<double> > *index, const std::vector<TNode *> &s, std::vector<std::vector<double> > &dists, std::vector<std::vector<size_t> > &indeces);
    vector<size_t> _findRangeStatesFlann(Index<L2<double> > *index, const State &s, const double range);
    vector<size_t> _findKNearestStatesFlann(Index<L2<double> > *index, const State &s,  std::vector<double>&dist, int k);


    int findNearestState(const std::vector<TNode *> &nodes, const State &s) const;

    inline void addToKdTree(KDTreeInt *kdTree, const State &s, const int index);
    inline void addToFlannTree(Index<L2<double> > *index, const State &s);

    //void updatePriorityQueue(Q_Node x_rev);
    //void insertToPriorityQueue(Q_Node x_for);
    void deleteQueue(std::vector<Pqueue *> &queue) const;
    void init_queue(size_t k, std::vector<Pqueue *> &queue) const;
    void updatePriorityQueue(TNode * x_rev);
    void updatePriorityQueue_KNN(TNode * x_rev);
    void updatePriorityQueue_range(TNode * rev);
    void insertToPriorityQueue(bool rev);
    //void getTrajectory(const State &fromState, const State &toState, int &lastForwardNode, vector<TNode> &result) const;

    bool RevSrchFastExplore(const State randomState, TNode *&result);
    bool ForSrchFastExplore(const State randomState, TNode *&result);
    
    //bool ForSrchExploit(Q_Node randomState, TNode *&result);
    bool ForSrchExploit(QueuePoint randomState, TNode *&result);
    bool ForSrchExploit(QueuePoint x_pop, TNode *&result, bool reversed);
    
    bool ForSrchRandomExplore(State randomState, TNode *&result, bool isFwd);

    bool BestInputProp(const int nearState, const State randomState, TNode *&result, bool isFwd);

    bool MonteCarloProp(const int nearState, TNode *&node, bool isFwd);

    void generateRandomInput(Input &randomInput);

    void getTrajectory(const State &fromState, const State &toState, const int j, const int k, std::vector<TNode> &traj) const;
    void getTrajectories(std::vector<std::vector<TNode> > &traj, vector<double> &lengths, vector<int> order = nullptr) const;
    void extractTrajectories();

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
    //int getTreeBSize() const { return _treeB.size();}
    void setAMAX(const int AMAX){ amax = AMAX; }

    CStat getStatistic() const { return statistic;};
    void clearStatistics();

    void setCollisionCheckFreq(const int f) { collisionCheckFreq = f; }
	int getCollisionCheckFreq() const { return collisionCheckFreq; }

    void setMaxEdgeTime(const double t) { T_max = t; }
    double getMaxEdgeTime() const { return T_max; }
    void setExploringProb(const double p) { q = p; }
    double setExploringProb() const { return q; }
    void setMonteCarloProp(const double N) { _N_mcp = N; }
    int setMonteCarloProp() const { return _N_mcp; }
    void setExploitingRegion(const double r) { r_k_input = r; }
    double setExploitingRegion() const { return r_k_input; }

    void setVideoGeneration(const int t) { generate_video = t; }
    double setVideoGeneration() const { return generate_video; }

    int update_nb = 0;
    int flann_nb = 0;
    int amax;
	
    
    void init_tree(std::vector<std::vector<TNode *> > &tree, std::vector<State> st) const;
    void init_tree(std::vector<std::vector<TNode *> > &tree, int length) const;
    void init_tree(std::vector<std::vector<TNode *> > &tree) const;
    void add_next_tree(std::vector<std::vector<TNode *> > &tree, const State &st, int idx) const;
    void push_index(std::vector<FlannTree *> &sh_index, const std::vector<TNode *> &tree, int idx);
    void push_tree(std::vector<std::vector<TNode *> > &sh_tree, std::vector<TNode *> &tree, int idx) const;
    
    //const std::vector< TNode *> &getTreeFwd(int k) const { return _treeF[k]; }
    
    //const std::vector< TNode *> &getTreeBack(int k) const { return _treeB[k]; }

    const std::vector< TNode *> &getTree(int k) const { return _tree[k]; } //for drawing
    const std::vector<std::vector< TNode *> > &getTrees() const; //for drawing

    std::vector<std::vector<TNode *> > _tree{};//, _treeB;
    std::vector<FlannTree *> index{};//(dataset, flann::KDTreeIndexParams(1));

    size_t n_states;

    //std::vector<std::vector<Pqueue*> > pqueue;
    std::vector<Pqueue *> pqueue{}; //n**2 queue array

    /*TSP part*/
    std::vector<std::vector<TNode *>> _tree_TSP{};
    std::vector<FlannTree *> index_TSP{};
    std::vector<Pqueue *> pqueue_TSP{};

    CPainterBase *pa;
    CPainterBase *pa_video;

    std::vector<std::vector<TNode *> > shortest_trajs{};

    private: //here are private methods and variables

    MapR  *map;
	R *robot;
    double edgeTime;
	int edgeSamples;
	double distanceToGoalTh;
    int samplesPerInput;
    int collisionCheckFreq;
	double goalBias;
	double _treeDistanceThreshold;
    double _inputResolution, _resolution;
    double _expansionStep;
    double _guidingDistance;
    int _N_mcp;
    double timeResolution;
    const int robotDistanceDimension;
    double r_k;
    double r_k_input;
    double T_max;
    double q;
    bool animate;
    int Ftree_rand; //used in first stage as random tree and in second as first tree in order
    int Btree_rand; //used in first stage as random tree and in second as second tree in order
    int planning_stage;
    int act_seq;
    int generate_video;

    mutable CStat statistic;

	std::vector<State> _initState{}, _goalState{};
    std::vector<State> _states{};
    std::vector<size_t> min_lengths{};
    
    //std::vector<std::vector<TNode *> > shortest_trees;
    //std::vector<FlannTree *> shortest_index;
    
    //std::vector<KDTreeInt *> _kdTreeF;
    //std::vector<KDTreeInt *> _kdTreeB;

    void deleteTree(std::vector<std::vector<TNode *> > &tree) const;
    void deleteTree(std::vector<TNode *> &tree) const;
};

template<typename R>
void KRRF<R>::generate(const std::vector<State> &init_states, const int size) {
    std::vector<const char *> colors{"green3", "DimGrey", "DodgerBlue3", "orange", "salmon", "peru", "khaki3", "brown", "black", "DarkOliveGreen", "sienna", "tomato2", "IndianRed", "SkyBlue", "cyan3"};
    
    //First generating trees among pairs
    planning_stage = 0;

    //Statistics and Time usage
    clearStatistics();

    n_states = init_states.size();

	struct rusage t1,t2;
	getTime(&t1);

    // init traj_vector for TSP
    init_tree(shortest_trajs);

    _states = init_states;


    
    //std::vector<std::vector<Pqueue*> > pqueue{n_states,vector<Pqueue*>(n_states,0)};
    deleteQueue(pqueue);
    init_queue(n_states, pqueue);
    //WDEBUG("QUEUE inited " << pqueue.size() << " "); //pqueue[0].size() << " ");

    //WDEBUG("Queue cleaned");
    
    deleteTree(_tree);
    //WDEBUG("FTree cleaned");
    init_tree(_tree, _states);

    const SDimension &mapDimension(map->getDimension());

    //KDTrees initialization
    /*_kdTreeF = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTreeF, _treeF[0]->state, 0);
    
    _kdTreeB = new KDTreeInt( robot->getDistanceDim(),1, (int *)robot->getDistanceTopology(), (MPNN::ANNpoint)robot->getDistanceScale());
    addToKdTree(_kdTreeB, _treeB[0]->state, 0);*/
    r_k = r_k_input;

    WDEBUG("Trying Flann initialization\n");
    //FlannTrees initialization
    deleteFlann(index);
    init_flann(index);
    WDEBUG("Flann size " << index.size());

    WDEBUG("Finished Flann initialization\n");
    
    //const int M_iter = size;//10;
    WDEBUG("Params " << r_k_input << " " << _N_mcp << " " << q << " " << T_max << " " << " " << n_states << " " << distanceToGoalTh);
    const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;
    bool isEdge = false;

    double size_free_env = pow(std::min(mapDimension.getSize(0),mapDimension.getSize(1)), 2.0);
    //r_k_input*1.5;//

    //OPTIMAL REGION
    double cst = 3.0;//2*pow(1+1/((double)robotDistanceDimension), 1/((double)robotDistanceDimension));
    double gamma_size = cst*pow(size_free_env/M_PI, 1.0/2.0);

    //WDEBUG("GAMMA, SIZE_ENV " << gamma_size << " " << size_free_env << " " << r_k_input << " " << mapDimension.getSize(1));
    //r_k = std::min(200*pow(std::log10(_treeB.size())/_treeB.size(),1/((double)robotDistanceDimension+1)), r_k_input);

    std::vector<std::vector<int>> pairs{};
    for (int tf=0; tf < n_states-1; tf++){
        for (int tb=tf+1; tb < n_states; tb++){
            std::vector<int> idx_pair = {tf,tb};
            pairs.push_back(idx_pair);
            //WDEBUG("Pairs " << tf << " " << tb);
        }
    }


    struct rusage timeOfLastInfo, someTime;
	getTime(&timeOfLastInfo);
    for (int k = 0; k < n_states*size; k++){

    //     if (0 & pa_video && (k % 100) == 0) {
    //         pa_video->begin();
    //         drawMap(*map,pa_video);
            
    //         for(int t=0; t<n_states; t++){
    //             vector<double> costs{};
    //             costs.reserve(_tree[t].size());
    //             for(int i=0;i<(int)_tree[t].size();i++) {
    //                 costs.push_back(0);
    //                 if (i > 0) {
    //                     costs[i] = _tree[t][i]->KRRF_hGet();//getCost(i);
    //                 }
    //             }
    //             const double mincost = *std::min_element(costs.begin(), costs.end());
    //             const double maxcost = *std::max_element(costs.begin(), costs.end());

    //             vector<TPoint> pts{};
    //             CColorMap cm;
    //             cm.setRange(mincost, maxcost);
    //             for(int ti = 0; ti < (int)_tree[t].size(); ti++) {
    //                 if (_tree[t][ ti ]->parent != -1) {
    //                     pts.push_back(robot->getRefPoint(_tree[t][ ti ]->state));
    //                     pts.push_back(robot->getRefPoint(_tree[t][ ti ]->pstate));
    // //                    pts.push_back(robot->getRefPoint(_tree[ _tree[ti]->parent ]->state));
    //                     drawPl(pts, 2, cm.getColor(costs[ti]),pa_video);
    //                     pts.clear();
    //                 }
    //             }
    //         }
    //         //drawRRT(*this,*robot,pa);
    //         pa_video->close();

    //     }
        
        getTime(&someTime);
		if (getTime(timeOfLastInfo,someTime) > 0.2) {
			//std::cerr << "[" << k << "/" << " Forward " << _treeF.size() << " Backward " << _treeB.size() << "] ";
			std::cerr << "[" << k << "/"  << "] ";
            getTime(&timeOfLastInfo);
		}

        //define R_K as in RRT optimal
        //r_k = std::min(gamma_size*pow(std::log10(_treeB.size())/_treeB.size(),1.0/3.0), r_k_input);
        r_k = r_k_input;

        //get index of random tree which will be expanded
        int rand_idx = (int)floor(getRandom(0,(double)pairs.size()));
        Ftree_rand = pairs[rand_idx][0];//(int)floor(getRandom(0,(double)n_states-1));
        Btree_rand = pairs[rand_idx][1];//(int)floor(getRandom((double)Ftree_rand+1,(double)n_states));

        //Reverse tree expansion - RevSrchFastExplore
        State randomState = robot->getRandomState(mapDimension);
        TNode *result =  new TNode;
        isEdge = RevSrchFastExplore(randomState, result); //get the result trajectory - exploration

        //originally check collisions instead of checking them before
        if (isEdge){
            _tree[Btree_rand].push_back(result);
            addToFlannTree(index[Btree_rand], _tree[Btree_rand].back()->state);

            updatePriorityQueue(result);
        }
        double c_rand = getRandom(0.0,1.0);//distr_double(mt);
        isEdge = false;
        TNode * result_for = new TNode;
        if (c_rand < q){
            QueuePoint x_pop;
            if (pqueue[Ftree_rand*n_states + Btree_rand]->size >= 0){
                //WDEBUG("queue elem " << pqueue.vec[0].first << " " << pqueue.vec[0].second);
                x_pop = pqueue[Ftree_rand*n_states + Btree_rand]->extractMax(); //already with popping
                isEdge = ForSrchExploit(x_pop, result_for); //returns the forward node nearest to the backward tree node
            }

            if (!isEdge){
                randomState = robot->getRandomState(mapDimension);
                //WDEBUG("State of the resulting node in ForSrchFastExplore " << randomState[0] << " " << randomState[1]);
                isEdge = ForSrchFastExplore(randomState, result_for); //forward result from rand node using MC
            }
        }
        //WDEBUG("Have I found an edge? " << isEdge);
        if (c_rand >= q || !isEdge){
            randomState = robot->getRandomState(mapDimension);
            isEdge = ForSrchRandomExplore(randomState, result_for, true); //forward result from rand node using MC
        }
        //WDEBUG("IsEdge " << isEdge);
        if (isEdge){
            //possible collision check, but it was already done in the MC method
            _tree[Ftree_rand].push_back(result_for);
            
            addToFlannTree(index[Ftree_rand], _tree[Ftree_rand].back()->state);

            bool finished = true;
            /*for (int tf=0; tf < n_states-1; tf++){
                for (int tb=tf+1; tb < n_states; tb++){
                    double dist;
                    int nearIdx = (int)_findNearestStateFlann(index[tf], _states[tb], dist);
                    if (distanceToGoalTh >= 0 && robot->distance(_tree[tf][nearIdx]->state,_states[tb]) >= distanceToGoalTh2) {
                        finished = false;
                    }
                }
            }*/
            double dist;
            int nearIdx = (int)_findNearestStateFlann(index[Ftree_rand], _states[Btree_rand], dist);
            if (distanceToGoalTh >= 0 && robot->distance(_tree[Ftree_rand][nearIdx]->state,_states[Btree_rand]) >= distanceToGoalTh2) {
                finished = false;
            } else if (robot->distance(_tree[Ftree_rand][nearIdx]->state,_states[Btree_rand]) < distanceToGoalTh2) {
                pairs.erase(pairs.begin() + rand_idx);
            }
            
            if (pairs.size()>0){
                finished = false;
            }
            // TODO - the problem is that when we run out of iterations,
            //        the paths do not have to be correctly computed in the terms of length
            if (finished){
                break;
            }
            //WDEBUG("Queue size before inserting " << pqueue[Ftree_rand*n_states + Btree_rand]->size);
            insertToPriorityQueue(true);
            //WDEBUG("Queue size after inserting " << pqueue[Ftree_rand*n_states + Btree_rand]->size);
        }
        //animating of the trajectory
        if (generate_video && pa_video) { //&& k % 50 == 0
            //WDEBUG("drawing trajectory");
            pa_video->begin();
            drawMap(*map,pa_video);
            for(int t=0; t<n_states; t++){
                //drawNodeTreeKRRFControl(_tree, *robot, pa, CColor("green4"));
                drawNodeTreeKRRFControl(_tree[t], *robot, pa_video, CColor(colors[t%colors.size()]));
            }            
            //remake point printing
            for (int t=0; t<init_states.size(); t++){
                //drawRobot(robot.getShape(init_states[t]),pa);
                draw(robot->getRefPoint(init_states[t]),4,1,CColor("black"),CColor("cyan"),pa_video);
            }
            //draw(robot->getRefPoint(fromState),5,1,CColor("black"),CColor("blue"),pa);
            //draw(robot->getRefPoint(toState),5,1,CColor("black"),CColor("cyan"),pa);
            pa_video->close();
        }
        statistic["IterationKRRF"] = k;
    }

    /*Extract trajectories to the traj vector*/
    extractTrajectories();

    getTime(&t2);
	statistic["KRRFTreeBuildTime"] = getTime(t1,t2);
	//WDEBUG("distance to goal state is " << robot->distance(_treeF.back()->state,_states[Btree_rand]));
    //statistic["treeSize"] = _tree.size();
}    

template<typename R>
bool KRRF<R>::generate_TSP_guiding(vector<int> order, const int size){
    std::vector<const char *> colors{"green3", "DimGrey", "DodgerBlue3", "orange", "salmon", "peru", "khaki3", "brown", "black", "DarkOliveGreen", "sienna", "tomato2", "IndianRed", "SkyBlue", "cyan3"};
    //Second generating tour tree

    planning_stage = 1;

    struct rusage t1,t2;
	getTime(&t1);

    bool success = false;
    double init_random_rate = 0.9;
    double random_rate = init_random_rate;
    int seq = 0;
    statistic["IterationGuiding"] = 0;
    
    deleteTree(_tree_TSP);
    add_next_tree(_tree_TSP, _states[order[0]], -1);
    
    deleteFlann(index_TSP);
    add_next_index(index_TSP, _states[order[0]], -1);

    bool isEdge = false;
    const double distanceToGoalTh2=distanceToGoalTh*distanceToGoalTh;

    const SDimension &mapDimension(map->getDimension());
    seq = 0;
    success = false;
    int max_n = amax;

    for (int iter = 0; iter < max_n; iter++){
        if (seq >= order.size()){
            break;
        }
        Ftree_rand = order[seq];
        Btree_rand = order[(seq+1)%n_states];

        act_seq = seq;

        std::vector<TNode *> guide_nodes{};
        if (shortest_trajs[Ftree_rand*n_states + Btree_rand].size() > 0){
            guide_nodes = shortest_trajs[Ftree_rand*n_states + Btree_rand];
            std::reverse(guide_nodes.begin(), guide_nodes.end());
        } else {
            guide_nodes = shortest_trajs[Btree_rand*n_states + Ftree_rand];
        }
        size_t ord = 0;
        TNode * act_goal{};
        act_goal = guide_nodes[ord];

        success = false;
        int it = 0;
        
        for (int k = 0; k < size; k++){
            it = k;
            double c_rand = getRandom(0.0,1.0);
            TNode * result_for = new TNode;
            State randomState;
            if (c_rand > random_rate){
                randomState = robot->getRandomState(mapDimension);
            } else {
                randomState = act_goal->state;
                randomState[0] = randomState[0] + getRandom(-_guidingDistance, _guidingDistance);
                randomState[1] = randomState[1] + getRandom(-_guidingDistance, _guidingDistance);
                randomState[2] = getRandom(0,2*M_PI);
            }
            isEdge = ForSrchFastExplore(randomState, result_for); //forward result from rand node using MC
            if (isEdge){
                _tree_TSP[seq].push_back(result_for);
                
                addToFlannTree(index_TSP[seq], _tree_TSP[seq].back()->state);
                
                //Check distance to the guiding path
                std::vector<std::vector<size_t> > indices{};
                std::vector<std::vector<double> > dists{};
                std::vector<TNode *> subvect = {guide_nodes.begin() + ord, guide_nodes.end()};
                _findNearestStateFlannToPath(index_TSP[seq], subvect, dists, indices);
                for (int q = dists.size()-1; q >= 0; q--){
                    if (distanceToGoalTh >= 0 && robot->distance(_tree_TSP[seq][indices[q][0]]->state, guide_nodes[ord+q]->state) < r_k_input*r_k_input) {
                        ord+=q;
                        act_goal = guide_nodes[ord];
                        break;
                    }
                }

                if (distanceToGoalTh >= 0 && robot->distance(_tree_TSP[seq].back()->state, _states[Btree_rand]) < distanceToGoalTh2){
                    add_next_index(index_TSP, _tree_TSP[seq].back()->state, -1);
                    add_next_tree(_tree_TSP, _tree_TSP[seq].back()->state, -1);
                    success = true;
                    random_rate = init_random_rate;
                    seq++;
                    break;
                }
                if (generate_video && pa_video) { //&& k % 50 == 0
                    // Only simple draw without guiding trajectory
                    // pa_video->begin();
                    // drawMap(*map,pa_video);
                    // for (int t=0; t<_tree_TSP.size(); t++){
                    //     drawNodeTreeKRRFControl(_tree_TSP[t], *robot, pa_video, CColor("DarkOliveGreen")); //colors[t%colors.size()]
                    // }
                    // for (int t=0; t<n_states; t++){
                    //     //drawRobot(robot.getShape(init_states[t]),pa);
                    //     draw(robot->getRefPoint(_states[order[t]]),4,1,CColor("black"),CColor("cyan"),pa_video);
                    //     std::string number_str = std::to_string(t+1);
                    //     char const *num_char = number_str.c_str();
                    //     pa_video->draw(num_char, tgp(robot->getRefPoint(_states[order[t]]),false));
                    // }
                    // //draw(robot->getRefPoint(act_goal->state),4,1,CColor("black"),CColor("cyan"),pa);
                    // //draw(robot->getRefPoint(_states[Btree_rand]),4,1,CColor("black"),CColor("blue"),pa);
                    // //draw(robot->getRefPoint(_states[Ftree_rand]),4,1,CColor("red"),CColor("blue"),pa);
                    // pa_video->close();


                    //Draw just actual guiding path
                    pa_video->begin();
                    drawMap(*map,pa_video);
                    for (int t=0; t<_tree_TSP.size(); t++){
                        drawNodeTreeKRRFControl(_tree_TSP[t], *robot, pa_video, CColor("DarkOliveGreen")); //colors[t%colors.size()]
                    }
                        
                    std::vector<std::vector<TNode>> guide_nodes{};
                    std::vector<TNode> guider{};
                    if (shortest_trajs[Ftree_rand*n_states + Btree_rand].size() > 0){
                        for (int j = 0; j<shortest_trajs[Ftree_rand*n_states + Btree_rand].size(); j++){
                            guider.push_back(*shortest_trajs[Ftree_rand*n_states + Btree_rand][j]);
                        }
                        std::reverse(guider.begin(), guider.end());
                        guide_nodes.push_back(guider);
                    } else {
                        for (int j = 0; j<shortest_trajs[Btree_rand*n_states + Ftree_rand].size(); j++){
                            guider.push_back(*shortest_trajs[Btree_rand*n_states + Ftree_rand][j]);
                        }
                        std::reverse(guider.begin(), guider.end());
                        guide_nodes.push_back(guider);
                    }
                    drawTrajectoryKRRF2DControl(guide_nodes,*robot,pa_video,false);

                          
                    for (int t=0; t<n_states; t++){
                        //drawRobot(robot.getShape(init_states[t]),pa);
                        draw(robot->getRefPoint(_states[order[t]]),4,1,CColor("black"),CColor("cyan"),pa_video);
                        std::string number_str = std::to_string(t+1);
                        char const *num_char = number_str.c_str();
                        pa_video->draw(num_char, tgp(robot->getRefPoint(_states[order[t]]),false));
                    }
                    pa_video->close();


                    // // Draw all guiding paths
                    // pa_video->begin();
                    // drawMap(*map,pa_video);
                    // for (int t=0; t<_tree_TSP.size(); t++){
                    //     drawNodeTreeKRRFControl(_tree_TSP[t], *robot, pa_video, CColor("DarkOliveGreen")); //colors[t%colors.size()]
                    // }
                    // for (int i = 0; i<n_states ; i++){
                    //     int F_tree = order[i];
                    //     int B_tree = order[(i+1)%n_states];
                        
                    //     std::vector<std::vector<TNode>> guide_nodes{};
                    //     std::vector<TNode> guider{};
                    //     if (shortest_trajs[F_tree*n_states + B_tree].size() > 0){
                    //         for (int j = 0; j<shortest_trajs[F_tree*n_states + B_tree].size(); j++){
                    //             guider.push_back(*shortest_trajs[F_tree*n_states + B_tree][j]);
                    //         }
                    //         std::reverse(guider.begin(), guider.end());
                    //         guide_nodes.push_back(guider);
                    //     } else {
                    //         for (int j = 0; j<shortest_trajs[B_tree*n_states + F_tree].size(); j++){
                    //             guider.push_back(*shortest_trajs[B_tree*n_states + F_tree][j]);
                    //         }
                    //         std::reverse(guider.begin(), guider.end());
                    //         guide_nodes.push_back(guider);
                    //     }
                    //     drawTrajectoryKRRF2DControl(guide_nodes,*robot,pa_video,false);

                    // }        
                    // for (int t=0; t<n_states; t++){
                    //     //drawRobot(robot.getShape(init_states[t]),pa);
                    //     draw(robot->getRefPoint(_states[order[t]]),4,1,CColor("black"),CColor("cyan"),pa_video);
                    //     std::string number_str = std::to_string(t+1);
                    //     char const *num_char = number_str.c_str();
                    //     pa_video->draw(num_char, tgp(robot->getRefPoint(_states[order[t]]),false));
                    // }
                    // pa_video->close();
                }
            }
        }
        statistic["IterationGuiding"] = statistic["IterationGuiding"]+it;
        if (0 && pa) {
            pa->begin();
            drawMap(*map,pa);
            for (int t=0; t<_tree_TSP.size(); t++){
                drawNodeTreeKRRFControl(_tree_TSP[t], *robot, pa, CColor(colors[t%colors.size()]));
            }
            draw(robot->getRefPoint(act_goal->state),4,1,CColor("black"),CColor("cyan"),pa);
            draw(robot->getRefPoint(_states[Btree_rand]),4,1,CColor("black"),CColor("blue"),pa);
            draw(robot->getRefPoint(_states[Ftree_rand]),4,1,CColor("red"),CColor("blue"),pa);
            pa->close();
        }
        if (!success && seq > 0 && random_rate == init_random_rate){
            WDEBUG("Replanning seq " << seq);
            random_rate = 1-init_random_rate;
            add_next_index(index_TSP, _tree_TSP[seq-1].back()->state, seq);
            add_next_tree(_tree_TSP, _tree_TSP[seq-1].back()->state, seq);
        } else if (!success && seq > 0){
            WDEBUG("Replanning seq " << seq);
            random_rate = init_random_rate;
            _tree_TSP[seq].clear();
            _tree_TSP.erase(_tree_TSP.begin() + seq);
            index_TSP.erase(index_TSP.begin() + seq);
            seq--;
            _tree_TSP[seq].clear();
            _tree_TSP.erase(_tree_TSP.begin() + seq);
            index_TSP.erase(index_TSP.begin() + seq);
            if (seq == 0){
                add_next_index(index_TSP, _states[order[0]], -1);
                add_next_tree(_tree_TSP, _states[order[0]], -1);
            }else {
                add_next_index(index_TSP, _tree_TSP[seq-1].back()->state, -1);
                add_next_tree(_tree_TSP, _tree_TSP[seq-1].back()->state, -1);
            }
        } else if (!success && seq == 0){
            _tree_TSP[seq].clear();
            _tree_TSP.erase(_tree_TSP.begin() + seq);
            index_TSP.erase(index_TSP.begin() + seq);
            add_next_index(index_TSP, _states[order[0]], -1);
            add_next_tree(_tree_TSP, _states[order[0]], -1);

        } else if (success) {
            WDEBUG("Successful segment " << seq-1);
        } else {
            WDEBUG("Another reason" << seq);
        }
        
    }
    //}
    getTime(&t2);
    statistic["TSP_RTTreeBuildTime"] = getTime(t1,t2);
    if (!success || seq < order.size()){
        return false;
    }
    return true;
}


template<typename R>
const std::vector<std::vector< TNode *> > &KRRF<R>::getTrees() const{
    if (planning_stage == 0){
        return _tree;
    }
    return _tree_TSP;
 }


template<typename R>
void KRRF<R>::deleteTree(std::vector<std::vector<TNode *> > &tree) const {
    if (tree.size() > 0){
        for(int t=tree.size()-1; t>=0;t--){
            if (tree[t].size() > 0){
                for(int i=tree[t].size()-1;i>=0;i--) {
                    if (tree[t][i]) {
                        delete tree[t][i];
                    }
                }
            }
            tree[t].clear();
        }
    }
    tree.clear();
}

template<typename R>
void KRRF<R>::deleteTree(std::vector<TNode *> &tree) const {
    for(int i=(int)tree.size()-1;i>=0;i--) {
        if (tree[i]) {
            delete tree[i];
        }
    }
    tree.clear();
}

template<typename R>
void KRRF<R>::deleteFlann(std::vector<FlannTree *> &ind) const {
    for(int i=(int)ind.size()-1;i>=0;i--) {
        ind.erase(ind.begin() + i);
    }
    ind.clear();
}

template<typename R>
void KRRF<R>::init_tree(std::vector<std::vector<TNode *> > &tree) const{
    tree.clear();
    for(size_t t=0; t<n_states*n_states; t++){
        std::vector<TNode *> subtree{};
        tree.push_back(subtree);
    }
}

template<typename R>
void KRRF<R>::init_tree(std::vector<std::vector<TNode *> > &tree, std::vector<State> st) const{
    for(size_t t=0; t<n_states; t++){
        std::vector<TNode *> subtree{};
        subtree.push_back(new TNode());
        subtree.back()->KRRF_setDefault();
        subtree.back()->state = st[t];
        tree.push_back(subtree);
    }
}

template<typename R>
void KRRF<R>::init_tree(std::vector<std::vector<TNode *> > &tree, int length) const{
    for(size_t t=0; t<length; t++){
        std::vector<TNode *> subtree{};
        tree.push_back(subtree);
        WDEBUG("Shortest tree " << subtree.size() << " " << tree.back().size());
    }
}

template<typename R>
void KRRF<R>::add_next_tree(std::vector<std::vector<TNode *> > &tree, const State &st, int idx) const{
    std::vector<TNode *> subtree{};
    subtree.push_back(new TNode());
    subtree.back()->KRRF_setDefault();
    subtree.back()->state = st;
    if (idx == -1){
        tree.push_back(subtree);
    } else {
        tree[idx] = subtree;
    }
}

template<typename R>
void KRRF<R>::push_tree(std::vector<std::vector<TNode *> > &sh_tree, std::vector<TNode *> &tree, int idx) const{
    for (int i = 0; i < tree.size(); i++){
        sh_tree[idx].push_back(tree[i]);
    }
}

template<typename R>
void KRRF<R>::push_index(std::vector<FlannTree *> &sh_index, const std::vector<TNode *> &tree, int idx){
    for (size_t i = 0; i < sh_index[idx]->size(); i++){
        sh_index[idx]->removePoint(i);
    }
    Matrix<double> dataset = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
    for(size_t i=0;i<(size_t)robotDistanceDimension;i++) {
        dataset[0][i] = tree[0]->state[i];
    }
    FlannTree *f = new FlannTree(dataset, flann::KDTreeIndexParams(1));
    sh_index[idx] = f;
    sh_index[idx]->buildIndex();
    for (size_t i = 1; i < tree.size(); i++){
        addToFlannTree(sh_index[idx], tree.back()->state);
    }
}

template<typename R>
void KRRF<R>::init_flann(std::vector<FlannTree *> &index) const{
    for(size_t t=0; t<n_states; t++){
        Matrix<double> dataset = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
        for(size_t i=0;i<(size_t)robotDistanceDimension;i++) {
            dataset[0][i] = _states[t][i];
        }
        FlannTree *f = new FlannTree(dataset, flann::KDTreeIndexParams(1));
        index.push_back(f);
        index.back()->buildIndex();
    }
}

template<typename R>
void KRRF<R>::init_flann(std::vector<FlannTree *> &index, int length) const{

    for(size_t t=0; t<length; t++){
        Matrix<double> dataset = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
        for(size_t i=0;i<(size_t)robotDistanceDimension;i++) {
            dataset[0][i] = _states[0][i];
        }
        FlannTree *f = new FlannTree(dataset, flann::KDTreeIndexParams(1));
        index.push_back(f);
        index.back()->buildIndex();
    }
}

template<typename R>
void KRRF<R>::add_next_index(std::vector<FlannTree *> &index, const State &st, int idx) const{
    Matrix<double> dataset = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
    for(size_t i=0;i<(size_t)robotDistanceDimension;i++) {
        dataset[0][i] = st[i];
    }
    FlannTree *f = new FlannTree(dataset, flann::KDTreeIndexParams(1));
    if (idx == -1){
        index.push_back(f);
        index.back()->buildIndex();
    } else {
        index[idx] = f;
        index[idx]->buildIndex();
    }
}

template<typename R>
void KRRF<R>::deleteQueue(std::vector<Pqueue *> &queue) const {
    for(int i=0;i<(int)queue.size();i++) {
        if (queue[i]) {
            delete queue[i];
        }
    }
    queue.clear();
}

template<typename R>
void KRRF<R>::init_queue(size_t k, std::vector<Pqueue *> &queue) const {
    for (size_t i=0; i<k; i++){
        for (size_t j=0; j<k; j++){
            Pqueue *p = new Pqueue();
            queue.push_back(p);
        }
    }
}

template<typename R>
int KRRF<R>::_findNearestState(KDTreeInt *tree, const State &s, double &dist){
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
int KRRF<R>::_findNearestStateFlann(Index<L2<double> > *index, const State &s, double &dist){
    struct rusage t1,t2;
    std::vector<std::vector<size_t> > indices{};
    std::vector<std::vector<double> > dists{};
    Matrix<double> query = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
    for(int i=0;i<robotDistanceDimension;i++) {
		query[0][i] = s[i];
	}
    index->knnSearch(query, indices, dists, 1, flann::SearchParams(FLANN_CHECKS_UNLIMITED));
    dist = sqrt(dists[0][0]);
    return indices[0][0];
}

template<typename R>
void KRRF<R>::_findNearestStateFlannToPath(Index<L2<double> > *index, const std::vector<TNode *> &s, std::vector<std::vector<double> > &dists, std::vector<std::vector<size_t> > &indeces){
    Matrix<double> query = Matrix<double>(new double[s.size()*robotDistanceDimension], s.size(), robotDistanceDimension);
    for(int j=0;j<s.size();j++){
        for(int i=0;i<robotDistanceDimension;i++) {
            query[j][i] = s[j]->state[i];
        }
    }
    index->knnSearch(query, indeces, dists, 1, flann::SearchParams(FLANN_CHECKS_UNLIMITED));
}

template<typename R>
std::vector<size_t> KRRF<R>::_findKNearestStatesFlann(Index<L2<double> > *index, const State &s,  std::vector<double>&dist, int k){
    struct rusage t1,t2;
    std::vector<std::vector<size_t> > indices{};
    std::vector<std::vector<double> > dists{};
    Matrix<double> query = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
    for(int i=0;i<robotDistanceDimension;i++) {
		query[0][i] = s[i];
	}
    index->knnSearch(query, indices, dists, k, flann::SearchParams(FLANN_CHECKS_UNLIMITED));
    dist = dists[0]; //warning - it is not square rooted
    return indices[0];
}

template<typename R>
vector<size_t> KRRF<R>::_findRangeStatesFlann(Index<L2<double> > *index, const State &s, const double range){
    struct rusage t1,t2;
    std::vector<std::vector<size_t> > indices{};
    std::vector<std::vector<double> > dists{};
    Matrix<double> query = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);
    for(int i=0;i<robotDistanceDimension;i++) {
		query[0][i] = s[i];
	}
    index->radiusSearch(query, indices, dists, range*range, flann::SearchParams(FLANN_CHECKS_UNLIMITED));
    return indices[0];
}

template<typename R>
inline void KRRF<R>::addToFlannTree(Index<L2<double> > *index, const State &s) {
    Matrix<double> point = Matrix<double>(new double[1*robotDistanceDimension], 1, robotDistanceDimension);

    //MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		point[0][i] = s[i];
	}
    index->addPoints(point);
}

template<typename R>
inline void KRRF<R>::addToKdTree(KDTreeInt *kdTree, const State &s, const int index) {
	MPNN::ANNpointArray annpt = MPNN::annAllocPts(1,robotDistanceDimension);
	for(int i=0;i<robotDistanceDimension;i++) {
		annpt[0][i] = s[i];
	}
	kdTree->AddPoint(annpt[0],index);
	MPNN::annDeallocPts(annpt);
}

template<typename R>
double KRRF<R>::distance(Q_Node qnode1, Q_Node qnode2){
    double dist;
    dist = sqrt( robot->distance(qnode1.node->state, qnode2.node->state) );
    return dist;
}

template<typename R>
void KRRF<R>::updatePriorityQueue(TNode * rev){
    //vector<Que> tmp_QNode;
    //Q_Node qnode;
    double dist;
    //int nearIdx1 = _findNearestState(_kdTreeF, rev->state, dist);
    int nearIdx = (int)_findNearestStateFlann(index[Ftree_rand], rev->state, dist);
    if (dist <= r_k && pqueue[Ftree_rand*n_states + Btree_rand]->size >= 0 && (pqueue[Ftree_rand*n_states + Btree_rand]->pos2idx.find(nearIdx) != pqueue[Ftree_rand*n_states + Btree_rand]->pos2idx.end())){
        //WDEBUG("Near index " << nearIdx << "\n");
        update_nb++;
        pqueue[Ftree_rand*n_states + Btree_rand]->changePriority(nearIdx, -1*(sqrt(robot->distance(rev->state, _tree[Ftree_rand][nearIdx]->state)) + rev->KRRF_hGet()));
    }
}

//Is not remade
template<typename R>
void KRRF<R>::updatePriorityQueue_KNN(TNode * rev){
    std::vector<double> dist{};
    
    std::vector<size_t> indexes = _findKNearestStatesFlann(index[Ftree_rand], rev->state, dist, 10);
    
    for (size_t i=0; i<indexes.size(); i++){
        if (sqrt(dist[i]) <= r_k && pqueue[Ftree_rand*n_states + Btree_rand]->size >= 0 && (pqueue[Ftree_rand*n_states + Btree_rand]->pos2idx.find(indexes[i]) != pqueue[Ftree_rand*n_states + Btree_rand]->pos2idx.end())){
            update_nb++;
            pqueue[Ftree_rand*n_states + Btree_rand]->changePriority(indexes[i], -1*(sqrt(robot->distance(rev->state, _tree[Ftree_rand][indexes[i]]->state)) + rev->KRRF_hGet()));
        }
    }
}

template<typename R>
void KRRF<R>::updatePriorityQueue_range(TNode * rev){
    std::vector<size_t> indexes = _findRangeStatesFlann(index[Ftree_rand], rev->state, r_k);
    
    for (size_t i=0; i<indexes.size(); i++){
        if (pqueue[Ftree_rand*n_states + Btree_rand]->size >= 0 && (pqueue[Ftree_rand*n_states + Btree_rand]->pos2idx.find(indexes[i]) != pqueue[Ftree_rand*n_states + Btree_rand]->pos2idx.end())){ //sqrt(dist[i]) <= r_k &&
            update_nb++;
            pqueue[Ftree_rand*n_states + Btree_rand]->changePriority(indexes[i], -1*(sqrt(robot->distance(rev->state, _tree[Ftree_rand][indexes[i]]->state)) + rev->KRRF_hGet()));
        }
    }
}

template<typename R>
void KRRF<R>::insertToPriorityQueue(bool rev){
    double dist;
    QueuePoint x_for;
    if (planning_stage == 0){
        x_for.second = (int)_tree[Ftree_rand].size()-1;        
        int nearIdx = (int)_findNearestStateFlann(index[Btree_rand], _tree[Ftree_rand][x_for.second]->state, dist);
        if (dist <= r_k){
            x_for.first = -1*(sqrt( robot->distance(_tree[Ftree_rand][x_for.second]->state, _tree[Btree_rand][nearIdx]->state) ) + _tree[Btree_rand][nearIdx]->KRRF_hGet());
            pqueue[Ftree_rand*n_states + Btree_rand]->insert(x_for);
        }
    } else {
        x_for.second = (int)_tree_TSP[act_seq].size()-1;        
        int nearIdx;
        if (rev){
            nearIdx = (int)_findNearestStateFlann(index[Btree_rand], _tree_TSP[act_seq][x_for.second]->state, dist);
        } else {
            nearIdx = (int)_findNearestStateFlann(index[Ftree_rand], _tree_TSP[act_seq][x_for.second]->state, dist);
        }
        if (dist <= r_k){
            if (rev) {
                x_for.first = -1*(sqrt( robot->distance(_tree_TSP[act_seq][x_for.second]->state, _tree[Btree_rand][nearIdx]->state) ) + _tree[Btree_rand][nearIdx]->KRRF_hGet());
            } else {
                const int nearestTo = findNearestState(_tree[Ftree_rand], _states[Btree_rand]); 
                double h_actB = _tree[Ftree_rand][nearestTo]->KRRF_hGet();
                x_for.first = -1*(sqrt( robot->distance(_tree_TSP[act_seq][x_for.second]->state, _tree[Ftree_rand][nearIdx]->state) ) + h_actB - _tree[Ftree_rand][nearIdx]->KRRF_hGet());
            }
            pqueue[act_seq]->insert(x_for);
        }
    }

}

template<typename R>
bool KRRF<R>::RevSrchFastExplore(const State randomState, TNode *&result){

    double dist;
    int nearIdx = (int)_findNearestStateFlann(index[Btree_rand], randomState, dist);
    
    //Best input propagation
    bool isEdge = BestInputProp(nearIdx, randomState, result, false); //back tree
    return isEdge;
}

template<typename R>
bool KRRF<R>::BestInputProp(const int nearIdx, const State randomState, TNode *&result, bool isFwd){
    bool isEdge = false;
    bool firstEdge = true;
    for (int k = 0;k < _N_mcp; k++){
        TNode *edge;
        isEdge = MonteCarloProp(nearIdx, edge, isFwd);
        if (isEdge){
            if (firstEdge){
                result = edge;
                firstEdge = false;
            }
            else {
                if (robot->distance(result->state, randomState) > robot->distance(edge->state, randomState)){
                    result = edge;
                }
            }
        }
    }
    return isEdge;    
}

template<typename R>
bool KRRF<R>::MonteCarloProp(const int nearIdx, TNode *&result, bool isFwd){
    double randomTime = getRandom(timeResolution, T_max);
    
    //computing number of steps
    int samples = (int) (randomTime/timeResolution);
    const double dt = randomTime/(double)samples;

    bool isFree = false;

    //collision stuff initialization
    double rRobot[3][3], rMap[3][3], vRobot[3], vMap[3];
	RAPID_model *mmap = map->getRapidModel(rMap,vMap);
	RAPID_model *mrobot;
	struct rusage t1,t2;

    
    //computing the random input
    Input inputRandom(robot->getInputSize(),0);
    generateRandomInput(inputRandom);

    TNode *nearest_tree_node;
    if (planning_stage == 0){
        if (!isFwd){ //back tree
            nearest_tree_node = _tree[Btree_rand][nearIdx];
        } else { //forw tree
            nearest_tree_node = _tree[Ftree_rand][nearIdx];
        }
    } else {
        nearest_tree_node = _tree_TSP[act_seq][nearIdx];
    }
    //control the robot for n steps from the near state
    robot->setState(nearest_tree_node->state);
    std::vector<State> st = robot->control(inputRandom, dt, samples);
    // check for self collision or for velocity constraints
    isFree = true;
    for(int i=0; i<(int)st.size() && isFree; i++) {
        isFree = robot->isValidState(st[i]);
    }
    if (isFree){
        // check all states for collision
        for(int i=(int)st.size()-1; i>=0 && isFree; i--) {
            mrobot = robot->getRapidModel(rRobot,vRobot,st[i]);
            RAPID_Collide(rMap,vMap,mmap,rRobot,vRobot,mrobot,RAPID_FIRST_CONTACT);
            isFree = (RAPID_num_contacts == 0);
        }
    }
    if (isFree){
        result = new TNode();
        result->KRRF_setDefault();
        result->KRRF_time() = randomTime;
        result->input = inputRandom;
        result->state = st.back();
        result->parent = nearIdx;
        result->pstate = nearest_tree_node->state;
        double totalCost = 0;
        totalCost += sqrt( robot->distance(nearest_tree_node->state, st[0]) );

        if (st.size() > 1){
            for (int i=0; i<st.size()-1; i++){
                totalCost += sqrt( robot->distance(st[i], st[i+1]) );
            }
        }
        result->KRRF_h() = nearest_tree_node->KRRF_hGet() + totalCost;
        result->KRRF_edgeSamples() = samples;
    }
    mmap = NULL;
    mrobot = NULL;
    return isFree;
}

template<typename R>
void KRRF<R>::generateRandomInput(Input &randomInput){
    const vector<double> inputMin = robot->getMinInputValues();
    const vector<double> inputMax = robot->getMaxInputValues();
    int inputSize = robot->getInputSize();
    bool inputValid = false;
    while (!inputValid){ 
        for (int i=0; i<randomInput.size(); i++){
            randomInput[i] = getRandom(inputMin[i], inputMax[i]);
        }
        if (robot->isInputValid(randomInput)){
            inputValid = true;
        }
    }
}

template<typename R>
bool KRRF<R>::ForSrchFastExplore(State randomState, TNode *&result){

    double dist;
    int nearIdx;
    if (planning_stage == 0){
        nearIdx = (int)_findNearestStateFlann(index[Ftree_rand], randomState, dist);
    } else {
        nearIdx = (int)_findNearestStateFlann(index_TSP[act_seq], randomState, dist);
    }
    //Best input propagation
    bool isEdge = BestInputProp(nearIdx, randomState, result, true);
    return isEdge;
}

template<typename R>
bool KRRF<R>::ForSrchRandomExplore(State randomState, TNode *&result, bool isFwd){

    double dist;
    int nearIdx;
    if (planning_stage == 0){
        nearIdx = (int)_findNearestStateFlann(index[Ftree_rand], randomState, dist);
    } else {
        nearIdx = (int)_findNearestStateFlann(index_TSP[act_seq], randomState, dist);
    }
    //just monte carlo
    bool isEdge = MonteCarloProp(nearIdx, result, isFwd);
    return isEdge;
}

template<typename R>
bool KRRF<R>::ForSrchExploit(QueuePoint x_pop, TNode *&result){


    vector<size_t> rev_near{};

    bool isEdge = false;

    //get k nearest neighbours by popping all from radius rk
    double dist = 0;

    flann_nb++;

    rev_near = _findRangeStatesFlann(index[Btree_rand], _tree[Ftree_rand][x_pop.second]->state, r_k);

    //selecting the one neighbour which minimizes the g + d + h    
    int min_near = 0;
    double min_near_heur = INFINITY;
    for (int i = 0; i < rev_near.size(); i++){
        double act_heuristic = _tree[Ftree_rand][x_pop.second]->KRRF_hGet() + sqrt( robot->distance(_tree[Ftree_rand][x_pop.second]->state, _tree[Btree_rand][rev_near[i]]->state)) + _tree[Btree_rand][rev_near[i]]->KRRF_hGet();
        if (min_near_heur > act_heuristic){
            min_near = i;
            min_near_heur = act_heuristic;
        }
        isEdge = true;
    }
    if (isEdge){
        //Best input propagation
        isEdge = BestInputProp(x_pop.second, _tree[Btree_rand][rev_near[min_near]]->state, result, true); // xpop, fwd_tree, xbest, "returned edge"
    }    
    return isEdge;
}

template<typename R>
bool KRRF<R>::ForSrchExploit(QueuePoint x_pop, TNode *&result, bool rev){


    vector<size_t> rev_near{};

    bool isEdge = false;

    //get k nearest neighbours by popping all from radius rk
    double dist = 0;

    
    if (rev){
        rev_near = _findRangeStatesFlann(index[Btree_rand], _tree_TSP[act_seq][x_pop.second]->state, r_k);
    } else {
        rev_near = _findRangeStatesFlann(index[Ftree_rand], _tree_TSP[act_seq][x_pop.second]->state, r_k);
    }
    //selecting the one neighbour which minimizes the g + d + h    
    int min_near = 0;
    double min_near_heur = INFINITY;
    for (int i = 0; i < rev_near.size(); i++){
        double act_heuristic;
        if (rev){
            act_heuristic = _tree_TSP[act_seq][x_pop.second]->KRRF_hGet() + sqrt( robot->distance(_tree_TSP[act_seq][x_pop.second]->state, _tree[Btree_rand][rev_near[i]]->state)) + _tree[Btree_rand][rev_near[i]]->KRRF_hGet();
        } else {
            const int nearestTo = findNearestState(_tree[Ftree_rand], _states[Btree_rand]);
            double h_actB = _tree[Ftree_rand][nearestTo]->KRRF_hGet(); //heuristic of the last node the nearest to the node
            act_heuristic = _tree_TSP[act_seq][x_pop.second]->KRRF_hGet() + sqrt( robot->distance(_tree_TSP[act_seq][x_pop.second]->state, _tree[Ftree_rand][rev_near[i]]->state)) + h_actB - _tree[Ftree_rand][rev_near[i]]->KRRF_hGet();
        }
        if (min_near_heur > act_heuristic){
            min_near = i;
            min_near_heur = act_heuristic;
        }
        isEdge = true;
    }
    if (isEdge){
        //Best input propagation
        if (rev) {
            isEdge = BestInputProp(x_pop.second, _tree[Btree_rand][rev_near[min_near]]->state, result, rev); // xpop, fwd_tree, xbest, "returned edge"
        } else {
            isEdge = BestInputProp(x_pop.second, _tree[Ftree_rand][rev_near[min_near]]->state, result, rev); // xpop, fwd_tree, xbest, "returned edge"
        }
    }    
    return isEdge;
}


/**
  * return path from RRT2DControl tree from node 'n' to node 'm', where
  * 'n' has nearest state to 'from' and
  * 'm' has nearest state to 'to'
  *
  W* nearest state is measured with metric defined in a robot (distance() method) 
  */
template<typename R>
void KRRF<R>::getTrajectory(const State &fromState, const State &toState, const int j, const int k, std::vector<TNode> &traj) const {


    const int nearestFrom = findNearestState(_tree[j], fromState);
    const int nearestTo = findNearestState(_tree[j], toState);

    traj.clear();

    if (nearestTo == -1 or nearestFrom == -1) {
        return;
    }

    vector< TNode * > tmpResult{};

    int tmp = nearestTo;

	while(tmp != -1 && tmp != nearestFrom ) {
		tmpResult.push_back(_tree[j][ tmp ]);
		tmp = _tree[j][ tmp ]->parent;
	}

	if (tmp >= 0) {
		tmpResult.push_back(_tree[j][ tmp ]);
	}

    for(int i=(int)tmpResult.size()-1; i >= 0; i--) {
        traj.push_back( *tmpResult[i] );
        traj.back().parent = -1;
    }
    tmpResult.clear();

}

template<typename R>
void KRRF<R>::extractTrajectories(){
    for (int j=0; j < n_states-1; j++){
        for (int k=j+1; k < n_states; k++){
            const int nearestFrom = findNearestState(_tree[j], _states[j]);
            const int nearestTo = findNearestState(_tree[j], _states[k]);
            if (nearestTo == -1 or nearestFrom == -1) {
                return;
            }

            vector< TNode * > tmpResult{};

            int tmp = nearestTo;
            
            while(tmp != -1 && tmp != nearestFrom ) {
                tmpResult.push_back(_tree[j][ tmp ]);
                tmp = _tree[j][ tmp ]->parent;
            }

            if (tmp >= 0) {
                tmpResult.push_back(_tree[j][ tmp ]);
            }
            shortest_trajs[j*n_states + k] = tmpResult;
        }
    }
}


template<typename R>
void KRRF<R>::getTrajectories(std::vector<std::vector<TNode> > &traj, vector<double> &lengths, vector<int> order) const {
    traj.clear();
    lengths.clear();
    if (planning_stage == 0){
        for (int j=0; j < n_states-1; j++){
            for (int k=j+1; k < n_states; k++){
                const int nearestFrom = findNearestState(_tree[j], _states[j]);
                const int nearestTo = findNearestState(_tree[j], _states[k]);
                if (nearestTo == -1 or nearestFrom == -1) {
                    return;
                }

                vector< TNode * > tmpResult{};

                int tmp = nearestTo;
                
                while(tmp != -1 && tmp != nearestFrom ) {
                    tmpResult.push_back(_tree[j][ tmp ]);
                    tmp = _tree[j][ tmp ]->parent;
                }

                if (tmp >= 0) {
                    tmpResult.push_back(_tree[j][ tmp ]);
                }
                vector<TNode> subtree{};
                
                for(int i=(int)tmpResult.size()-1; i >= 0; i--) {
                    subtree.push_back( *tmpResult[i] );
                    subtree.back().parent = -1;
                }
                traj.push_back(subtree);
                lengths.push_back(_tree[j][nearestTo]->KRRF_hGet());
            }
        }
    } else {
        for (int j=0; j < n_states; j++){
            const int nearestFrom = 0;
            const int nearestTo = _tree_TSP[j].size()-1;
            if (nearestTo == -1 or nearestFrom == -1) {
                return;
            }

            vector< TNode * > tmpResult{};

            int tmp = nearestTo;
            
            while(tmp != -1 && tmp != nearestFrom ) {
                tmpResult.push_back(_tree_TSP[j][ tmp ]);
                tmp = _tree_TSP[j][ tmp ]->parent;
            }
            WDEBUG("Index" << nearestFrom << " " << nearestTo << " " << tmp << " " << _tree_TSP[j].size() << " " << _tree_TSP[j][nearestTo]->KRRF_hGet());
            if (tmp >= 0) {
                tmpResult.push_back(_tree_TSP[j][ tmp ]);
            }
            vector<TNode> subtree{};
            
            for(int i=(int)tmpResult.size()-1; i >= 0; i--) {
                subtree.push_back( *tmpResult[i] );
                subtree.back().parent = -1;
            }
            traj.push_back(subtree);
            lengths.push_back(_tree_TSP[j][nearestTo]->KRRF_hGet());
        }
    }
}

template<typename R>
int KRRF<R>::findNearestState(const std::vector<TNode *> &nodes, const State &s) const {

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

template<typename R>
void KRRF<R>::clearStatistics() {
	statistic.zero();

	statistic["KRRFTreeBuildTime"] = 0;
	//statistic["realIteration"] = 0;

}

}


#endif
