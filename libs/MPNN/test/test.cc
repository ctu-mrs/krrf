#include <stdlib.h>
#include <stdio.h>			// C I/O
#include <stdint.h>
#include <fstream>			// file I/O
#include <iostream>			// file I/O
#include <string>			// string manipulation
#include <math.h>			// math routines
#include <list>
#include <queue>
#include <time.h>

#include <DNN/ANN.h>			// ANN declarations
#include <DNN/multiann.h>

#ifndef INFINITY
#define INFINITY 1.0e40
#endif

#ifndef PI
#define PI 3.1415926535897932385
#endif

using namespace std;
using namespace MPNN;


int		MaxNeighbors    = 16;		// number of nearest neighbors
int		dim		= 2;		// dimension
int		m_pts		= 20000;       	// maximum number of data points
int		numIt		= 100;       	// maximum number of nearest neighbor calls

istream		*dim_in		= NULL;		// input for dimension
istream		*topology_in	= NULL;		// input for topology

void randomDistPoint(int dim, int *tt, ANNpoint &p)
{
    for (int kk = 0; kk < dim; kk++){
      double tmp = rand()/ (2.14783e+09);
      if (tt[kk] == 2) {
        p[kk] = (2*PI*tmp - PI); // region -PI PI
      }
      else if (tt[kk] == 3) {
	double tmp1 = rand()/(2.14783e+09);
	double tmp2 = rand()/(2.14783e+09);
	
	double Theta1 = 2*PI*tmp1;
	double Theta2 = PI*tmp2 - PI/2;
	double r1 = sqrt(1 - tmp);
	double r2 = sqrt(tmp);
	
	p[kk] =  sin(Theta1)*r1;
	p[kk + 1] = cos(Theta1)*r1;
	p[kk + 2] = sin(Theta2)*r2;
	p[kk + 3] = cos(Theta2)*r2;
	
	kk = kk + 3;		// random quaternions
      }
      else 
	p[kk] = 200*tmp - 100; 	//region -100 to 100
    }

}

void randomDist(int dim, int *tt, ANNpointArray &data_pts) 
{
  int n_pts = 0;
  while (n_pts < m_pts) {
    randomDistPoint(dim, tt, data_pts[n_pts]);
    n_pts++;
  }
}

// Default is to use the standard Euclidean metric
double Metric(ANNpoint x1, ANNpoint x2, int dim, int *topology, ANNpoint scale) {

  double rho = 0, fd, dtheta;
  
  for (int i = 0; i < dim; i++) {
    if (topology[i] == 1) {
      rho += ANN_POW(scale[i]*(x1[i] - x2[i]));
    }
    else if (topology[i] == 2) {
      fd = fabs(x1[i] - x2[i]);
      dtheta = ANN_MIN(fd, 2.0*PI - fd);
      rho += ANN_POW(scale[i]*dtheta);
    }
    else if (topology[i] == 3) {
      fd = x1[i]*x2[i] + x1[i+1]*x2[i+1] + x1[i+2]*x2[i+2] + x1[i+3]*x2[i+3];
      if (fd > 1) {
	double norm1 = x1[i]*x1[i] + x1[i+1]*x1[i+1] + x1[i+2]*x1[i+2] + x1[i+3]*x1[i+3];
	double norm2 = x2[i]*x2[i] + x2[i+1]*x2[i+1] + x2[i+2]*x2[i+2] + x2[i+3]*x2[i+3];
	fd = fd/(sqrt(norm1*norm2));
      }
      dtheta = ANN_MIN(acos(fd), acos(-fd));
      rho += ANN_POW(scale[i]*dtheta);
      i = i + 3;
    }
  }
  //cout << "dist = " << sqrt(rho) << endl <<endl;

  return sqrt(rho);
}

ANNbool readPt(istream &in, int *p)		// read point (false on EOF)
{
    for (int i = 0; i < dim; i++) {
	if(!(in >> p[i])) return ANNfalse;
    }
    return ANNtrue;
}

void printPt(ostream &out, ANNpoint p)		// print point
{
    out << "(" << p[0];
    for (int i = 1; i < dim; i++) {
	out << ", " << p[i];
    }
    out << ")\n";
}

int main(int argc, char **argv)
{
  double d_ann, d_brute;
  int idx_ann, idx_brute;

    static ifstream dimStream;			// dimension file stream
    static ifstream topologyStream;		// topology file stream

    ANNpointArray	data_pts;		// data points
    ANNpoint		query_pt;		// query point
    int 		*topology;		// topology of the space
    ANNpoint		scale;			// scaling of the coordinates
    ANNpoint		result_pt;     		// scaling of the coordinates
    MultiANN<ANNcoord *>		*MAG;			// search structure


    srand(time(NULL)); 
    dimStream.open("dim", ios::in);		// open query file
    if (!topologyStream) {
      cerr << "Cannot open dim file\n";
      exit(1);
    }
    dim_in = &dimStream;			// make this query stream
     (*dim_in) >> dim;				// read dimension


    query_pt = annAllocPt(dim);			// allocate query point
    data_pts = annAllocPts(m_pts, dim);		// allocate data points
    topology = new int[dim];			// allocate topology array
    scale = new double[dim];			// allocate scaling array


    topologyStream.open("topology", ios::in);	// open query file
    if (!topologyStream) {
      cerr << "Cannot open topology file\n";
      exit(1);
    }
    topology_in = &topologyStream;	      	// make this query stream
    readPt(*topology_in, topology);		// read topology
    
    for (int i = 0; i < dim; i++) {
      if (topology[i] == 1) scale[i] = 1.0;
      else if (topology[i] == 2) scale[i] = 50/PI;
      else if (topology[i] == 3) scale[i] = 50.0;
    }
    cout << "Topology and Scale: \n";			//print topology
    for (int i = 0; i < dim; i++) {
      cout << " topology[" << i << "] = " << topology[i] << "\t";
      cout << " scale[" << i << "] = " << scale[i] << endl;
    }
    cout << "\n";

  
    randomDist(dim, topology, data_pts);		// generate random points
    randomDistPoint(dim, topology, query_pt);		// generate query point
    
    cout << "MaxNeighbors=" << MaxNeighbors << "\t";
    cout << "m_pts=" << m_pts << "\t";
    cout << "dim=" << dim << "\t";
    cout << "numIt=" << numIt << "\n";

  for (int ii = 0; ii < 1000; ii++) {

    cout << "\n\t test " << ii << ":" << endl;

    randomDist(dim, topology, data_pts);		// generate random points
    randomDistPoint(dim, topology, query_pt);		// generate query point

    //***********************Construction phase**********************/
    
    clock_t tv1, tv2;
    double time;    
    tv1 = clock();

    MAG = new MultiANN<ANNcoord *>(dim, MaxNeighbors, topology, scale);	// create a data structure	
    for (int j = 0; j < m_pts; j++) {
      MAG->AddPoint(data_pts[j], data_pts[j]);		// add new data point
    }

    //ANN *a = new ANN(data_pts, 0, m_pts-1, m_pts, dim, topology, scale, 1);


    tv2 = clock();
    time = (tv2 - tv1)/(CLOCKS_PER_SEC / (double) 1000.0);
    cout << "Construction MPNN time:"<< time/100 << "sec\n";
    tv1 = clock();


    //*************************Query phase*******************************/

    
    double d;   
    for (int j = 0; j < numIt; j++) {
      //randomDistPoint(dim, topology, query_pt);				// generate query point
      d_ann = INFINITY;
      result_pt = (ANNpoint)MAG->NearestNeighbor(query_pt, idx_ann, d_ann);	// compute nearest neighbor using library
      //idx_ann = a->NearestNeighbor(topology, scale, query_pt, d_ann);		// compute nearest neighbor using library
    }
       

    tv2 = clock();
    time = (tv2 - tv1)/(CLOCKS_PER_SEC / (double) 1000.0);
    cout << "MPNN query time:"<< time/100 << "sec\n\n";
    tv1 = clock();

    for (int j = 0; j < numIt; j++) {
      // randomDistPoint(dim, topology, query_pt);				// generate query point
      d_brute = INFINITY;							// compare the obtained result with the brute force result
      for (int i = 0; i < m_pts; i++) {
	d = Metric(query_pt, data_pts[i], dim, topology, scale);
	if (d_brute > d) {
	  d_brute = d;
	  idx_brute = i;
	}
      }
    }

    tv2 = clock();
    time = (tv2 - tv1)/(CLOCKS_PER_SEC / (double) 1000.0);
    cout << "brute force time:"<< time/100 << "sec\n\n";
    tv1 = clock();


    //*************************k-Query phase*******************************/
     int MaxNeighbors = 16;
    double **n_best_list = new double *[MaxNeighbors];
    double *d_best_list = new double[MaxNeighbors];
    int *idx_best_list = new int[MaxNeighbors];
    list<double*> best_list;
    list<double*>::iterator ni;

     for (int j = 0; j < numIt; j++) {
      //randomDistPoint(dim, topology, query_pt);				// generate query point
      d_ann = INFINITY;
      MAG->NearestNeighbor(query_pt, (ANNcoord *)d_best_list, idx_best_list, (void **)n_best_list);	// compute nearest neighbor using library
    }
       

    tv2 = clock();
    time = (tv2 - tv1)/(CLOCKS_PER_SEC / (double) 1000.0);
    cout << "MPNN query time:"<< time/100 << "sec\n\n";
    tv1 = clock();

    for (int j = 0; j < numIt; j++) {
      // randomDistPoint(dim, topology, query_pt);				// generate query point
      d_brute = INFINITY;							// compare the obtained result with the brute force result
      best_list.clear();
      int k = 0;
      for (int i = 0; i < m_pts; i++) {
	d = Metric(query_pt, data_pts[i], dim, topology, scale);
	if ((d < 30) && (k < MaxNeighbors)) {
	  best_list.push_back(data_pts[i]);
	  d_brute = d;
	  idx_brute = i;
	  k++;
	}
      }
    }

    tv2 = clock();
    time = (tv2 - tv1)/(CLOCKS_PER_SEC / (double) 1000.0);
    cout << "brute force time:"<< time/100 << "sec\n\n";


    cout << "ANN distance = " << d_ann << endl;
    cout << "nearest neighbor";
    for (int i = 0; i < MaxNeighbors; i++) {
      if (d_best_list[i] < 30) 
	printPt(cout, (double *)n_best_list[i]);
    }
    
    cout << "Brute distance = " << d_brute << endl;
    cout << "nearest neighbor";
    for (ni = best_list.begin(); ni != best_list.end(); ni++)
      printPt(cout, (*ni));
  


    if (MAG){
      MAG->ResetMultiANN();
      MAG = NULL;
    }

    /************report problems if any*************************************/
    
    if (idx_ann != idx_brute) 			// report error if different nearest neighbors
      cout << "PROBLEM!!!!!!!"; 
      
    cout << "ANN distance = " << d_ann << endl;
    cout << "nearest neighbor";
    printPt(cout, result_pt);
    
    cout << "Brute distance = " << d_brute << endl;
    cout << "nearest neighbor";
    printPt(cout, data_pts[idx_brute]);
  }  
}






