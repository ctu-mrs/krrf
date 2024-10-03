#ifndef DIJKSTRA__L_H__
#define DIJKSTRA__L_H__

#include <vector>
#include "types.h"

namespace rrtPlanning {

std::vector<int> dijkstraLiteSearch2(const std::vector<SEdge> &edges, const int from, const int to);
std::vector<int> dijkstraLiteSearch(double **m, const int size, const int from, const int to);
std::vector<int> dijkstraSearch(double **m, const int size, const int from, const int to);

}

#endif


