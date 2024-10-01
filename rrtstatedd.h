#ifndef _RRT_STATEDD_H__
#define _RRT_STATEDD_H__

#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

#include "ut.h"
#include "map.h"

namespace rrtPlanning {

template<typename ST, typename IN>
class RRTStateDD {
	public:

		typedef ST State;
		typedef IN Input;
		typedef RRTStateDD<ST,IN> StateType;
		typedef typename std::list<StateType>::iterator State_iterator;
		typedef typename std::list<StateType>::const_iterator Const_state_iterator;



		RRTStateDD(const State &s = State(), const Input &i=Input(),
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
			nextSize(0),
            radius(-1),
            scale(1)
		{}

		~RRTStateDD() { 
			state.clear();
			input.clear();
			next.clear();
			prevState.clear();
		}

		RRTStateDD(const RRTStateDD<ST,IN> &rhs):
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
			nextSize(rhs.nextSize),
            radius(rhs.radius),
            scale(rhs.scale)
		{
		}

		void clear() {
			prev = NULL;
			next.clear();
			state.clear();
			input.clear();
			prevState.clear();
		}

		template<typename STT, typename INN>
		friend std::ostream &operator<<(std::ostream &os, const RRTStateDD<STT,INN> &s);

		RRTStateDD<ST,IN> &operator=(const RRTStateDD<ST,IN> &rhs) {
			if (&rhs != this) {
				state = rhs.state;
				input = rhs.input;
				prev = rhs.prev;
				next = rhs.next;
				time = rhs.time;
				index = rhs.index;
				prevState = rhs.prevState;
				indexInTrajectory = rhs.indexInTrajectory;
				cumulativeLength = rhs.cumulativeLength;
				nextSize = rhs.nextSize;
                radius = rhs.radius;
                scale = rhs.scale;
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
		int nextSize;
        double radius; // action radius, see rrtdd.h
        double scale; // scale of the robot - see RRT3_DD_I
};

} // namespace 

#endif

