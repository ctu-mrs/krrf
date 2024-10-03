
#include "dijkstra/dijkstra.h"
#include "dijkstra/dijkstra_lite.h"
#include "WLog.h"

#include <vector>
#include <algorithm>
#include <list>
#include <iostream>
#include "dijkstra.h"

namespace rrtPlanning {

using std::vector;
using std::reverse;
using std::list;
using std::cerr;


/* same as dijkstraLiteSearch, but this uses vector of edges instead of distance matrix */
vector<int> dijkstraLiteSearch2(const std::vector<SEdge> &edges, const int from, const int to) {
	imr::dijkstra::CDijkstraLite<double> dij;

    for(int i=0;i<(int)edges.size();i++) {
        dij.addEdge(edges[i].from, edges[i].to, edges[i].cost);
	}

	vector<int> pred;
	vector<double> dist;
	
	dij.solve(from,pred,dist);

	vector<int> res;

	const int predSize = pred.size();
	if (predSize > 0) {
		int l = to;
		while(l != from) {
			if (l == -1) {
				// unreachable vertex
				res.clear();
				return res;
			}
			res.push_back(l);
			if (l >= 0 && l<predSize) {
				l = pred[l];
			} else {
				res.clear();
				return res;
			}
		}
		res.push_back(l);
	}
	reverse(res.begin(),res.end());
	pred.clear();
	dist.clear();
	return res;
}



vector<int> dijkstraLiteSearch(double **m, const int size, const int from, const int to) {
	imr::dijkstra::CDijkstraLite<double> dij;

	for(int i=0;i<size;i++) {
		for(int j=0;j<size;j++)
			if (m[i][j] > 0) {
				dij.addEdge(i,j,m[i][j]);
			}
	}

	vector<int> pred;
	vector<double> dist;
	
	dij.solve(from,pred,dist);

	vector<int> res;

	const int predSize = pred.size();
	if (predSize > 0) {
		int l = to;
		while(l != from) {
			if (l == -1) {
				// unreachable vertex
				res.clear();
				return res;
			}
			res.push_back(l);
			if (l >= 0 && l<predSize) {
				l = pred[l];
			} else {
				res.clear();
				return res;
			}
		}
		res.push_back(l);
	}
	reverse(res.begin(),res.end());
	pred.clear();
	dist.clear();
	return res;
}

vector<int> dijkstraSearch(double **m, const int size, const int from, const int to) {

	for(int i=0;i<size;i++) {
		for(int j=0;j<size;j++) {
			if (m[i][j] == 0) {
				m[i][j] = -1;
			}
		}
	}


	// predchudci
	vector<int> p(size,-1);

	// vzdalenost d[i] od povatku
	vector<double> d(size,-1);

	vector<bool> u(size,false);

	d[from] = 0;

	while(1) {
		// nalezt nejlepsiho v d[]
		double minD = -1;
		int minI = -1;
		for(int i=0;i<size;i++)
			if ((!u[i]) && ( (d[i] < minD) && (d[i] != -1) || (minD == -1))) {
				minD = d[i];
				minI = i;
			}
		if (minI == -1 || minD == -1)
			break;

		if (minI == to)
			break;

		u[minI] = true;


		for(int j=0;j<size;j++) {
			if (m[minI][j] > 0) {
				if ((d[minI]+m[minI][j] < d[j]) || (d[j] == -1)) {
					d[j] = d[minI] + m[minI][j];
					p[j] = minI;
				}
			}
		}
	}

	list<int> result;

	int tmp = to;
	while(p[tmp] != -1) {
		result.push_front(tmp);
		tmp = p[tmp];
	}
	result.push_front(from);
	
	vector<int> res;
	res.reserve(result.size());
	for(list<int>::const_iterator i = result.begin(); i != result.end(); ++i)
		res.push_back(*i);
	
	p.clear();
	d.clear();
	u.clear();
	result.clear();
	return res;	
}


} // namespace rrtPlanning


