/*
 * File name: dijkstra.h
 * Date:      2008/07/30 10:38
 * Author:    Jan Faigl
 */

#ifndef __DIJKSTRA_H__
#define __DIJKSTRA_H__

#include <string>

namespace imr {
   namespace dijkstra {

      class CDijkstra {
	 public:
	    CDijkstra();
	    ~CDijkstra();

	    void load(std::string filename);
	    void save(std::string filename);
	    void solve();
	 private:
	    bool shortestPathHeap(unsigned int start);
	    void solveInit(int number);
	    void addToHeap(int n);
	    int firstHeap(void);
	    void down(void);
	    int relaxation(int node_idx);
	    void updateHeap(int n);
	    void cleanGraph(void);
	    void checkHeap(int n);

	 struct SNode {
	    int cost;
	    int pred;
	 };

	 struct SHeap {
	    int IDX;
	    int node;
	    int cost;
	 };

	 struct SEdge {
	    int from;
	    int to;
	    int cost;
	 };

	 struct SGraphNode {
	    int start;
	    int end;
	 };

	 int numberEdges;
	 int numberGraphNodes;
	 SGraphNode * graphNodes;
	 SEdge * edges;
	 SNode * nodes;
	 SHeap * heap;

	 int * nodeCost;
	 int * nodePred;
	 int * heapLeft;
	 int * heapParent;
	 int heapNumber;
	 int nodeNumber;

	 bool hasSolution;

      };

   } //end namespace dijkstra
} //end namespace imr


#endif

/* end of dijkstra.h */
