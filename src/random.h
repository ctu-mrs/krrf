
#ifndef _R_RANDOM_H_
#define _R_RANDOM_H_

#include <stdlib.h>
#include <vector>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_statistics.h>

#include <math.h>


namespace rrtPlanning {

int getRandomNumberFromDistribution(const std::vector<double> &dist);

// !! just for vector with sum to one!!!!!!!!!!!
void getRandomNumberFromDistribution(const std::vector< std::vector< double> > &dist, int &i, int &j);

// ! for arbitrary vectors !!!!!!
int getRandomNumberAccordingToDataDistribution(const std::vector<double> &data);

void setRandomSeed(const int randomSeed);
void setRandomSeed(const int randomSeed, gsl_rng *r);
long setRandom();

void initRandomNumberGenerator();
void freeRandomNumberGenerator();
double getRandomGauss(const double center, const double sigma);

double getRandom(const double from, const double to);
double getRandom(const double from, const double to, gsl_rng *r);


/* return random vector in n-ball pointing to a (n-1)sphere. algorithm based on 
 * Marsaglia, G. (1972), "Choosing a Point from the Surface of a Sphere", 
 *   Ann. Math. Stat. 43: 645-646, doi:10.1214/aoms/1177692644
 *
 * dimension of the ball is determined by size of the vector center. This vector is also used to
 * determine the center of the ball
 */
template<typename ST>
ST getRandomVectorOnBall(const ST &center, const double radius) {
    ST tmp(center.size(),0);
    double r = 0;
    for(int i=0;i<(int)tmp.size();i++) {
        //tmp[i] = getRandomGauss(center[i],1);
        tmp[i] = getRandomGauss(0,1);
        r += tmp[i]*tmp[i];
    }
    r = sqrt(r);
    for(int i=0;i<(int)tmp.size();i++) {
        tmp[i] /= r;
        tmp[i] *= radius;
        tmp[i] += center[i];
    }
    return tmp;
}



} // namespace 
#endif


