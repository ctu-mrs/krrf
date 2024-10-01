#ifndef _RRT_STATE_H__
#define _RRT_STATE_H__

#include <list>
#include <vector>

namespace rrtPlanning {

class TNode {
    public:
        TNode() {
            parent = -1;
        }

        TNode(const TNode &rhs):state(rhs.state), pstate(rhs.pstate), input(rhs.input), others(rhs.others), next(rhs.next), parent(rhs.parent) {
        }

        ~TNode() {

        }

    typedef std::vector<double> State;

    State state, pstate, input, others;
    std::vector<int> next;
    int parent;


    /* ========================================================== 
       for gometric rrt in 2D
     */
    /** RRT-2D. others[] is:  */
    void RRT2D_setDefault() { 
    }

    /* ========================================================== */
    /** RRT-2D. others[] is: time iteration */
    double &RRT2DT_time() { return others[0]; }
    double RRT2DT_timeGet() const { return others[0]; }

    double &RRT2DT_iteration() { return others[1]; }
    double RRT2DT_iterationGet() const { return others[1]; }
    void RRT2DT_setDefault() { 
        others = State(2,0); 
    }
    
    /* ========================================================== */
    /* KRRF algorithms control vars (TSP version)
    */
    void KRRF_setDefault() {
        others = State(3,0);  // size 1, zero values
        others.reserve(3);
        others[0] = 0; // default heuristic value (h for reverse nodes, g for forward)
        others[1] = 0;
        others[2] = 1;
    }

    double &KRRF_h() { return others[0]; }
    double KRRF_hGet() const { return others[0]; }
    double &KRRF_time() { return others[1]; }
    double KRRF_timeGet() const { return others[1]; }
    double &KRRF_edgeSamples() { return others[2]; }
    double KRRF_edgeSamplesGet() const { return others[2]; }


    /* ========================================================== */
    /* LazyTSP algorithms control vars (TSP version)
    */
    void LazyTSP_setDefault() {
        others = State(3,0);  // size 1, zero values
        others.reserve(3);
        others[0] = 0; // default heuristic value (h for reverse nodes, g for forward)
        others[1] = 0;
        others[2] = 1;
    }

    double &LazyTSP_h() { return others[0]; }
    double LazyTSP_hGet() const { return others[0]; }
    double &LazyTSP_time() { return others[1]; }
    double LazyTSP_timeGet() const { return others[1]; }
    double &LazyTSP_edgeSamples() { return others[2]; }
    double LazyTSP_edgeSamplesGet() const { return others[2]; }


    /* ========================================================== 
       for control RRT in 2D.
       */
    /** RRT-2D. others[] is: time */
    double &RRT2DControl_time() { return others[0]; }
    double RRT2DControl_timeGet() const { return others[0]; }

    void RRT2DControl_setDefault() { 
        others = State(1,0); 
    }
    /* ========================================================== */



    /* ========================================================== */


    /** RRT-DD: others[] is: time, radius */
    void RRTDD_setDefault() { 
        others = State(2,0); 
        others.reserve(2);
        others[1] = -1; // radius = -1;
    }
    double &RRTDD_time() { return others[0]; }
    double RRTDD_timeGet() const { return others[0]; }
    double &RRTDD_radius() { return others[1]; }
    double RRTDD_radiusGet() const { return others[1]; }



    /* ========================================================== */
    /** RRT-3D with cumulative length. others[] is: time cumulativeLengthj */
    double &RRT3DCL_time() { return others[0]; }
    double RRT3DCL_timeGet() const { return others[0]; }
    
    double &RRT3DCL_length() { return others[1]; }
    double RRT3DCL_lengthGet() const { return others[1]; }

    void RRT3DCL_setDefault() { 
        others = State(2,0); 
        others.reserve(2);
    }



    /* ========================================================== */

    /** RRT-2D-star: (optimal RRT) others[] is: time, cost */
    void RRT2DStar_setDefaults() {
        others = State(2,0); //time is 0, cost is 0
        others.reserve(2);
    }
    double &RRT2DStar_time() { return others[0]; }
    double RRT2DStar_getTime() const { return others[0]; }
    double &RRT2DStar_cost() { return others[1]; }
    double RRT2DStar_getCost() const { return others[1]; }

    void RRT2DStar_removeNextIdx(const int idx) {
        std::vector<int> tmp;
        tmp.reserve(next.size());
        for(int i=0;i<(int)next.size();i++) {
            if (next[i] != idx) {
                tmp.push_back(next[i]);
            }
        }
        next = tmp;
    }






    /* ========================================================== */
    /** RRT-2D-star: (optimal RRT) others[] is: time, cost */
    void RRT2DControlStar_setDefaults() {
        others = State(2,0); //time is 0, cost is 0
        others.reserve(2);
    }
    double &RRT2DControlStar_time() { return others[0]; }
    double RRT2DControlStar_getTime() const { return others[0]; }
    double &RRT2DControlStar_cost() { return others[1]; }
    double RRT2DControlStar_getCost() const { return others[1]; }

    void RRT2DControlStar_removeNextIdx(const int idx) {
        std::vector<int> tmp;
        tmp.reserve(next.size());
        for(int i=0;i<(int)next.size();i++) {
            if (next[i] != idx) {
                tmp.push_back(next[i]);
            }
        }
        next = tmp;
    }












    /* ========================================================== */

    /** RRT-3D. others[] */
    void RRT3D_setDefault() { 
        others.clear();
    }

    /* ========================================================== */

    /** RRT-3D - with measuing time (=iteration) of reaching each node. 
      others[] is: time, iteration-of-reach 
     */
    double &RRT3DT_time() { return others[0]; }
    double RRT3DT_timeGet() const { return others[0]; }
    
    double &RRT3DT_iteration() { return others[1]; }
    double RRT3DT_iterationGet() const { return others[1]; }

    void RRT3DT_expandOthers() {
        while(others.size() < 2) {
            others.push_back(0);
        }
    }


    void RRT3DT_setDefault() { 
        others = State(2,0); 
        others.reserve(2);
    }


    /* ========================================================== */



 /** RRT-3D-star: (optimal RRT) others[] is: time, cost */
    void RRT3DStar_setDefaults() {
        others = State(2,0); //time is 0, cost is 0
        others.reserve(2);
    }
    double &RRT3DStar_time() { return others[0]; }
    double RRT3DStar_getTime() const { return others[0]; }
    double &RRT3DStar_cost() { return others[1]; }
    double RRT3DStar_getCost() const { return others[1]; }

    void RRT3DStar_removeNextIdx(const int idx) {
        vector<int> tmp;
        tmp.reserve(next.size());
        for(int i=0;i<(int)next.size();i++) {
            if (next[i] != idx) {
                tmp.push_back(next[i]);
            }
        }
        next = tmp;
    }


    /* ========================================================== */


    /** RRT-DD for 3D robot: others[] is: radius */
    void RRT3DD_setDefault() { 
        others = State(1,0); 
        others.reserve(1);
        others[0] = -1; // radius = -1;
    }
    double &RRT3DD_radius() { return others[1]; }
    double RRT3DD_radiusGet() const { return others[1]; }


/* ========================================================== */

    /** ART (random growing tree without voronoi bias) others[] is: length treeidx */
    void ART_setDefault() { 
        others = State(2,0); 
        others.reserve(2);
        others[1] = -1; // treeidx
    }
    double &ART_length() { return others[0]; }
    double ART_lengthGet() const { return others[0]; }
    double &ART_idx() { return others[1]; }
    double ART_idxGet() const { return others[1]; }


/* ========================================================== */

    /** ART (random growing tree without voronoi bias) others[] is: lenght treeidx */
    void ART3D_setDefault() { 
        others = State(2,0); 
        others.reserve(2);
        others[1] = -1; // treeidx
    }
    double &ART3D_length() { return others[0]; }
    double ART3D_lengthGet() const { return others[0]; }
    double &ART3D_idx() { return others[1]; }
    double ART3D_idxGet() const { return others[1]; }

    /* ========================================================== */




};



template<typename ST, typename IN>
class RRTState {
	public:

		typedef ST State;
		typedef IN Input;
		typedef RRTState<ST,IN> StateType;
        typedef unsigned char InfoType;
		typedef typename std::list<StateType>::iterator State_iterator;
		typedef typename std::list<StateType>::const_iterator Const_state_iterator;



		RRTState(const State &s = State(), const Input &i=Input(),
				const double t = 0, const int idx = 0, const State &ps = State()):
			state(s),
			input(i),
			next(),
			time(t),
			index(idx),
			prevState(ps),
			isPrevValid(false),
			indexInTrajectory(0),
			cumulativeLength(0),
            cumulativeTime(0),
			nextSize(0),
            info(0)
		{}

		~RRTState() { 
			state.clear();
			input.clear();
			next.clear();
			prevState.clear();
		}

		RRTState(const RRTState<ST,IN> &rhs):
			state(rhs.state),
			input(rhs.input),
			prev(rhs.prev),
			next(rhs.next),
			time(rhs.time),
			index(rhs.index),
			prevState(rhs.prevState),
			isPrevValid(rhs.isPrevValid),
			indexInTrajectory(rhs.indexInTrajectory),
			cumulativeLength(rhs.cumulativeLength),
            cumulativeTime(rhs.cumulativeTime),
			nextSize(rhs.nextSize),
            info(rhs.info)
		{
		}

		void clear() {
			prev = NULL;
			next.clear();
			state.clear();
			input.clear();
			prevState.clear();
		}

        std::string getString() const;

		//template<typename STT, typename INN>
		//friend std::ostream &operator<<(std::ostream &os, const RRTState<STT,INN> &s);

        // very important for copying tree!
		RRTState<ST,IN> &operator=(const RRTState<ST,IN> &rhs) {
			if (&rhs != this) {
				state = rhs.state;
				input = rhs.input;
				prev = rhs.prev;
				next = rhs.next;
				time = rhs.time;
				index = rhs.index;
				prevState = rhs.prevState;
                isPrevValid = rhs.isPrevValid;
				indexInTrajectory = rhs.indexInTrajectory;
				cumulativeLength = rhs.cumulativeLength;
                cumulativeTime = rhs.cumulativeTime;
				nextSize = rhs.nextSize;
                info = rhs.info;
			}

			return *this;
		}

		State state;
		Input input;
		State_iterator prev;
        std::list < State_iterator > next;
		double time;
		int index;
		State prevState;
		bool isPrevValid;
		int indexInTrajectory; 
		double cumulativeLength;
        double cumulativeTime;
		int nextSize;
        InfoType info;
};



} // namespace 

#endif

