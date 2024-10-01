/*
 * File name: dijkstra_heap.h
 * Date:      2008/07/30 11:52
 * Author:    Jan Faigl
 */

#ifndef __DIJKSTRA_HEAP_H__
#define __DIJKSTRA_HEAP_H__

namespace imr {
   namespace dijkstra {

#define HEAPNODE(i) heap[i].node
#define HEAPIDX(i) heap[i].IDX

      template<class Nodes, class Weight>
	 class CHeap {

	    struct SHeap {
	       int IDX;
	       int node;
	       Weight cost;
	    };

	    inline int get_parent(int i) {
	       return (i-1) >> 1;
	    }

	    inline int get_left(int i) {
	       return (i<<1) + 1;
	    }

#define HEAPCOST(i) heap[i].cost
	    //#define HEAPCOST(i) nodes[HEAPNODE(i)].cost
	    inline void swap(int i, int j) {
	       //#define HEAP_SWAP(i,j) HEAPIDX(HEAPNODE(i)) = j; HEAPIDX(HEAPNODE(j)) = i; t = HEAPNODE(i); HEAPNODE(i) = HEAPNODE(j); HEAPNODE(j) = t; t = HEAPCOST(i); HEAPCOST(i) = HEAPCOST(j); HEAPCOST(j) = t;
	       //#define HEAP_SWAP(i,j) HEAPIDX(HEAPNODE(i)) = j; HEAPIDX(HEAPNODE(j)) = i; t = HEAPNODE(i); HEAPNODE(i) = HEAPNODE(j); HEAPNODE(j) = t;
	       int t;
	       HEAPIDX(HEAPNODE(i)) = j; 
	       HEAPIDX(HEAPNODE(j)) = i; 
	       t = HEAPNODE(i); 
	       HEAPNODE(i) = HEAPNODE(j); 
	       HEAPNODE(j) = t;
	       Weight w = HEAPCOST(i); 
	       HEAPCOST(i) = HEAPCOST(j);
	       HEAPCOST(j) = w;
	    }

	    public:
	    CHeap(int size, Nodes & nodes) : size(size), nodes(nodes) {
	       heap = 0;
	       heapNumber = 0;
	       heap = new SHeap[size];
	       for (int i = 0; i < size; i++) {
		  HEAPIDX(i) = -1; 
	       } 
	    }

	    ~CHeap(void) {
	       delete[] heap;
	    }

	    inline void add(int n, Weight cost) {
	       register int parent;
	       register int index;
	       HEAPNODE(heapNumber) = n;
	       HEAPCOST(heapNumber) = cost;
	       HEAPIDX(n) = heapNumber;
	       index = heapNumber; 
	       parent = get_parent(index);
	       while (index >= 1 && (HEAPCOST(parent) > HEAPCOST(index))) {
		  swap(parent, index);
		  index  = parent;
		  parent = get_parent(index);
	       }
	       heapNumber++;
	    }

	    inline int getFirst(void) {
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

	    inline void down(void) {
	       register int index;
	       register int hl, hr;
	       int best ;
	       index = 0;
	       hl = get_left(index);
	       if (hl >= heapNumber) {
	       } else {
		  while(hl < heapNumber) {
		     hr = hl+1;
		     if (HEAPCOST(index) > HEAPCOST(hl)) {
			best = hl;
		     } else {
			best = index;
		     }
		     if (hr < heapNumber && HEAPCOST(best) > HEAPCOST(hr)) {
			best = hr;
		     }
		     if (best != index) { // lower value found
			swap(index, best);
			index = best;
			hl = get_left(index);
		     } else {
			break;
		     }
		  }
	       }
	       //check_heap(0, heap, nodes);
	    }

	    void inline update(int n, Weight cost) {
	       register int index;
	       register int parent;
	       index = HEAPIDX(n);
	       HEAPCOST(index) = cost;
	       if (index < heapNumber) {
		  parent = get_parent(index);
		  while (index >= 1 && HEAPCOST(index) < HEAPCOST(parent)) {  //swap with parent
		     swap(index, parent);
		     index = parent;
		     parent = get_parent(index);
		  }
	       } 
	       // check_heap(0, heap, nodes);
	    } 

	    void inline is_valid(int n) {
	       int l = 2*n+1;
	       int r = 2*n+2;
	       if (l < heapNumber) {
		  if (HEAPCOST(l) < HEAPCOST(n)) {
		     //TODO assert
		     /*	 fprintf(stderr, "HEAP PROPERTY VIOLENCE l:%d n:%d cost %d %d\n", l, n,
			 HEAPCOST(l), HEAPCOST(n));
			 */
		  } else {
		     is_valid(l);
		  }
	       }
	       if (r < heapNumber) {
		  if (HEAPCOST(r) < HEAPCOST(n)) {
		     //TODO assert
		     // fprintf(stderr, "HEAP PROPERTY VIOLENCE r:%d n:%d\n", r, n);
		  }
		  is_valid(r);
	       }
	    }

	    int & getIDX(int n) {
	       return HEAPIDX(n);
	    }

	    private:
	    int size;
	    SHeap * heap;
	    int heapNumber;
	    Nodes & nodes;
	 };

   } //end namespace dijkstra
} //end namespace imr


#endif

/* end of heap.h */
