/*
 * File name: dijkstra.cc
 * Date:      2008/07/30 10:47
 * Author:    Jan Faigl
 */

#include <cmath>
#include <fstream>
#include <iostream>

#include "dijkstra.h"
#include <stdlib.h>

using namespace imr::dijkstra;

#define CHECK_MEM(p) if (p == 0) {std::cerr << "Can not allocate memory" << std::endl; exit(-1); }

#define DELETE_ARRAY(x) if (x) {delete[] x; x = 0;}

#define EDGE_FROM(i) edges[i].from
#define EDGE_TO(i) edges[i].to
#define EDGE_COST(i) edges[i].cost

#define START_EDGE(i) graphNodes[i].start
#define END_EDGE(i) graphNodes[i].end

#define NODECOST(i) nodes[i].cost
#define NODEPRED(i) nodes[i].pred

#define GETPARENT(i) (i-1)>>1
#define GETLEFT(i) (i<<1)+1

#define GETCOST(i) heap[i].cost
#define HEAPCOST(i) heap[i].cost
#define HEAPNODE(i) heap[i].node
#define HEAPIDX(i) heap[i].IDX
#define HEAP_SWAP(i,j) HEAPIDX(HEAPNODE(i)) = j; HEAPIDX(HEAPNODE(j)) = i; t = HEAPNODE(i); HEAPNODE(i) = HEAPNODE(j); HEAPNODE(j) = t; t = HEAPCOST(i); HEAPCOST(i) = HEAPCOST(j); HEAPCOST(j) = t;

/// ----------------------------------------------------------------------------
/// Class CDijkstra
/// ----------------------------------------------------------------------------
CDijkstra::CDijkstra() {
   heapNumber = 0;
   nodeCost = 0;
   nodePred = 0;
   nodeNumber = 0;

   heapParent = 0;
   heapLeft = 0;

   heap = 0;

   numberEdges = 0;
   numberGraphNodes = 0;
   graphNodes = 0;
   nodes = 0;
   edges = 0;
}

/// ----------------------------------------------------------------------------
CDijkstra::~CDijkstra() {
}

/// ----------------------------------------------------------------------------
void CDijkstra::load(std::string filename) {
   int r;
   int edgeMultiplication = 4; //default value
   int last_node = 0;
   int number_edges = 0;
   int number_nodes = -1;
   int allocate_nodes = 1000; //pro 10 linux verze 
   int allocate_edges = edgeMultiplication*allocate_nodes;
   int from, to, cost;
   int t;

   cleanGraph();

   graphNodes = (SGraphNode*)malloc(allocate_nodes*sizeof(SGraphNode));
   edges = (SEdge*)malloc(allocate_edges*sizeof(SEdge));
   CHECK_MEM(graphNodes);
   CHECK_MEM(edges);
   for (r = 0; r < allocate_nodes; r++) {
      START_EDGE(r) = -1;
      END_EDGE(r) = -1;
   }
   number_edges = 0;
   std::ifstream in(filename.c_str());
   while(in >> from >> to >> cost) {
      r = 3;
      if (r == 3) {
	 EDGE_FROM(number_edges) = from;
	 EDGE_TO(number_edges) = to;
	 EDGE_COST(number_edges) = cost;
	 if (START_EDGE(from) == -1) {
	    START_EDGE(from) = number_edges;
	 }
	 if (last_node != from) {
	    END_EDGE(last_node) = number_edges -1;
	 }
	 last_node = from;
	 if (number_nodes <= from) {
	    number_nodes = from+1;
	 }
	 if (number_nodes <= to) {
	    number_nodes = to+1;
	 }
	 number_edges++;
	 if (number_edges == allocate_edges) {
	    t = allocate_edges;
	    int estimation = (int)ceil((1.0*number_edges)/last_node);
	    if (estimation > edgeMultiplication) {
	       edgeMultiplication = estimation;
	    }
	    if (allocate_edges < allocate_nodes*edgeMultiplication) {
	       allocate_edges = allocate_nodes*edgeMultiplication;
	    } else {
	       allocate_edges = allocate_edges + 50;
	    }
	    edges = (SEdge*)realloc(edges, allocate_edges*sizeof(SEdge)); 
	    CHECK_MEM(edges); 
	 }

	 if (number_nodes > allocate_nodes) {
	    t = allocate_nodes;
	    allocate_nodes = number_nodes;
	    graphNodes = (SGraphNode*)realloc(graphNodes, allocate_nodes*sizeof(SGraphNode));
	    CHECK_MEM(graphNodes); 
	    for ( r = t; r < allocate_nodes; r++) {
	       START_EDGE(r) = -1;
	       END_EDGE(r) = -1;
	    } 
	 }
      }
   } // end while
   if (number_edges > 0) {
      END_EDGE(last_node) = number_edges-1;
   }
   numberEdges = number_edges;
   numberGraphNodes = number_nodes;
}

/// ----------------------------------------------------------------------------
void CDijkstra::save(std::string filename) {
   std::fstream out;
   out.open(filename.c_str(), std::ios::out);
   for (int i = 0; i < nodeNumber; i++) {
      out << i << " " << NODECOST(i) << " " << NODEPRED(i) << std::endl;
   }
}

/// ----------------------------------------------------------------------------
void CDijkstra::solve() {
   shortestPathHeap(0);
}

/// ----------------------------------------------------------------------------
bool CDijkstra::shortestPathHeap(unsigned int start) {
   bool ret = false;
   int o;
   solveInit(numberGraphNodes);
   NODECOST(start) = 0;
   addToHeap(start);
   while((o = firstHeap()) != -1) {
      relaxation(o); 
      HEAPIDX(o) = -1;
   }
   ret = true;
   return ret;
}

/// ----------------------------------------------------------------------------
void CDijkstra::solveInit(int number) {
   DELETE_ARRAY(nodeCost);
   DELETE_ARRAY(nodePred);
   DELETE_ARRAY(heapLeft);
   DELETE_ARRAY(heapParent);
   nodeCost = new int[number];
   nodePred = new int[number];
   heapLeft = new int[number];
   heapParent = new int[number];

   nodeNumber = number;
   heapNumber = 0;


   DELETE_ARRAY(nodes);
   DELETE_ARRAY(heap);
   nodes = new SNode[number];

   heap = new SHeap[number];
   for (int i = 0; i < number; i++) {
      NODECOST(i) = -1;
      NODEPRED(i) = -1;
      HEAPIDX(i) = -1; 
   } 
}

/// ----------------------------------------------------------------------------
void CDijkstra::addToHeap(int n) {
   register int parent;
   register int index;
   register int t;
   HEAPNODE(heapNumber) = n;
   HEAPCOST(heapNumber) = NODECOST(n);
   HEAPIDX(n) = heapNumber;
   index = heapNumber; 
   parent = GETPARENT(index);
   while (index >= 1 && (GETCOST(parent) > GETCOST(index))) {
      HEAP_SWAP(parent, index);
      index  = parent;
      parent = GETPARENT(index);
   }
   heapNumber++;
}

/// ----------------------------------------------------------------------------
int CDijkstra::firstHeap(void) {
   int ret;
   if (heapNumber == 0) {
      ret = -1;
   } else {
      ret = HEAPNODE(0);
      heapNumber--;
      HEAPNODE(0) = HEAPNODE(heapNumber);
      HEAPCOST(0) = HEAPCOST(heapNumber);
      HEAPIDX(HEAPNODE(0)) = 0;
      down();
   }
   return ret;
}

/// ----------------------------------------------------------------------------
void CDijkstra::down(void) {
   register int index;
   register int hl, hr;
   register int t;
   int best ;
   index = 0;
   hl = GETLEFT(index);
   if (hl >= heapNumber) {
   } else {
      while(hl < heapNumber) {
	 hr = hl+1;
	 if (GETCOST(index) > GETCOST(hl)) {
	    best = hl;
	 } else {
	    best = index;
	 }
	 if (hr < heapNumber && GETCOST(best) > GETCOST(hr)) {
	    best = hr;
	 }
	 if (best != index) { // lower value found
	    HEAP_SWAP(index, best);
	    index = best;
	    hl = GETLEFT(index);
	 } else {
	    break;
	 }
      }
   }
   //check_heap(0, heap, nodes);
}

/// ----------------------------------------------------------------------------
int CDijkstra::relaxation(int node_idx) {
   int ret = 0;
   register int i;
   register int n;
   int cost = NODECOST(node_idx);
   int startEdge = START_EDGE(node_idx);
   int endEdge = END_EDGE(node_idx);

   if (startEdge >= 0) {  //only if edges from node exist
      for (i = startEdge; i <= endEdge; i++) {
	 const SEdge * e  = &(edges[i]);
	 n = e->to;
	 if (HEAPIDX(n) == -1) { //not in open list
	    if (NODECOST(n) == -1) { //not  in close list
	       NODECOST(n) = cost + e->cost;
	       NODEPRED(n) = e->from;
	       addToHeap(n);
	    }
	 } else {
	    int c = cost + e->cost;
	    if (NODECOST(n)  > (c)) {
	       NODECOST(n) = c;
	       NODEPRED(n) = e->from;
	       updateHeap(n);
	    }
	 }
      } //end for loop
   }
   return ret;
}

/// ----------------------------------------------------------------------------
void CDijkstra::updateHeap(int n) {
   register int index;
   register int parent;
   register int t;
   index = HEAPIDX(n);
   HEAPCOST(index) = NODECOST(n);
   if (index < heapNumber) {
      parent = GETPARENT(index);
      while (index >= 1 && GETCOST(index) < GETCOST(parent)) {  //swap with parent
	 HEAP_SWAP(index, parent);
	 index = parent;
	 parent = GETPARENT(index);
      }
   } 
   // check_heap(0, heap, nodes);
}

/// ----------------------------------------------------------------------------
void CDijkstra::cleanGraph(void) {
   if (graphNodes) {
      free(graphNodes);
      graphNodes = 0; 
   }
   if (edges) {
      free(edges);
      edges = 0;
   }
   DELETE_ARRAY(nodes);
   DELETE_ARRAY(nodeCost);
   DELETE_ARRAY(nodePred);
}

/// ----------------------------------------------------------------------------
void CDijkstra::checkHeap(int n) {
   int l = 2*n+1;
   int r = 2*n+2;
   if (l < heapNumber) {
      if (HEAPCOST(l) < HEAPCOST(n)) {
	 //TODO assert
/*	 fprintf(stderr, "HEAP PROPERTY VIOLENCE l:%d n:%d cost %d %d\n", l, n,
	    HEAPCOST(l), HEAPCOST(n));
   */
      } else {
	 checkHeap(l);
      }
   }
   if (r < heapNumber) {
      if (HEAPCOST(r) < HEAPCOST(n)) {
	 //TODO assert
	// fprintf(stderr, "HEAP PROPERTY VIOLENCE r:%d n:%d\n", r, n);
      }
      checkHeap(r);
   }
}

/* end of dijkstra.cc */
