#include "random.h"
#include <time.h>

#include <numeric>
#include "WLog.h"

namespace rrtPlanning {


static gsl_rng *gslrng = NULL; //this cariable is set by calling initRandomNumberGenerator()

double getRandom(const double from, const double to) {
	return (double)(1.0*rand() / (1.0*RAND_MAX))*(to-from) + from;
}

double getRandom(const double from, const double to, gsl_rng *r) {
    return gsl_rng_uniform(r)*(to-from) + from;
}


long setRandom(){
	time_t t;
	time(&t);
	srand(t);
    return t;
}

/** returns random number according to given distribution.
  * The distribution is defined as p0, p1, p2 ... p_n, where 0<=p_i<=1 and sum p_i = 1.
  */
int getRandomNumberFromDistribution(const std::vector<double> &dist) {
	int idx = 0;
	const double r = getRandom(0.0,1.0);
	double s = 0;
	while(s <= r && idx < (int)dist.size()) {
		s += dist[idx++];
	}
	return idx-1;
}

/** return index of data according their probability. This vector does not need to be sum to one */
int getRandomNumberAccordingToDataDistribution(const std::vector<double> &data) {
	double sum = std::accumulate(data.begin(), data.end(),0.0);
	if (sum == 0 || data.size() == 0) {
		WDEBUG("Cannot compute distribution of data. Sum is 0");
		return 0;
	}
	std::vector<double> c(data);
	for(int i=0;i<(int)c.size();i++) {
		c[i]/= sum;
	}
	int r = getRandomNumberFromDistribution(c);
	c.clear();
	return r;
}

/** return position of cell according to distribution (i,j) */
void getRandomNumberFromDistribution(const std::vector< std::vector<double> > &dist, int &i, int &j) {
	i = 0;
	j = 0;
	const double r = getRandom(0.0,1.0);
	double s = 0;
	int li=0;
    int lj=0;
	while(s <= r) {
		s+= dist[i][j];
		li = i;
		lj = j;
		if (++i >= (int)dist.size()) {
			i = 0;
			if (++j >= (int)dist[i].size()) {
				break;
			}
		}
	}
	i = li;
	j = lj;
}

/** this function should be called at start of the program, because it initializes random number generator algorithm
  * used by following function in this module */
void initRandomNumberGenerator() {
    gslrng = gsl_rng_alloc(gsl_rng_rand); // this is unix random number generator

	time_t t;
	time(&t);
	//std::cerr << "seed for gsl-rng is " << t << "\n";
	gsl_rng_set(gslrng,t);
}



void freeRandomNumberGenerator() {
    gsl_rng_free(gslrng);
}

//return random variable from gaussian distribution centered at 'center' with given sigma
double getRandomGauss(const double center, const double sigma) {
    return center + gsl_ran_gaussian(gslrng,sigma);
}


void setRandomSeed(const int randomSeed, gsl_rng *r) {
    WDEBUG("srand=" << randomSeed);
    if (randomSeed >= 0) {
        WDEBUG("Setting random seed to " << randomSeed);
        gsl_rng_set(r, randomSeed);
    } else {
        time_t t;
        time(&t);
        gsl_rng_set(r, t);
        long tt = t;
    }
}



void setRandomSeed(const int randomSeed) {
    WDEBUG("srand=" << randomSeed);
    if (randomSeed >= 0) {
        WDEBUG("Setting random seed to " << randomSeed);
        srand(randomSeed);
    } else {
        time_t t;
        time(&t);
        srand(t);
        long tt = t;
        WDEBUG("Random seed is time " << tt);
    }
}



}
