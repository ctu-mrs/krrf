/*
 * File name: dijkstra_lite.h
 * Date:      2008/07/30 10:38
 * Author:    Jan Faigl
 */

#ifndef __DIJKSTRA_LITE_H__
#define __DIJKSTRA_LITE_H__

#include <iostream> //TODO debug

#include <vector>
#include <string>
#include <fstream>

#include "dijkstra_heap.h"
namespace imr {
   namespace dijkstra {

      template<class T>
	 class CDijkstraLite {
         public:
             typedef T Weight;
             CDijkstraLite() {}
             ~CDijkstraLite() {}

             void clear() {
                nodes.clear();
             }

             void reserve(const int maxNodeSize) {
                nodes.reserve(maxNodeSize);
             }

             void addEdge(int from, int to, Weight weight) {
                 int max;
                 max = from > to ? from : to;
                 if (max >= (int)nodes.size()) {
                     nodes.resize(max + 1); 
                 }
                 nodes[from].edges.push_back(SEdge(to, weight));
             }

             void removeEdge(int from, int to) {

                 std::vector<SEdge> tmp;
                 const Edges &old( nodes[from].edges );
                 tmp.reserve(nodes[from].edges.size());
                 for(int i=0;i<(int)old.size();i++) {
                    if (old[i].to != to) {
                        tmp.push_back(old[i]);
                    }
                 }
                 nodes[from].edges = tmp;
                 tmp.clear();
             }

             /// ----------------------------------------------------------------------------
             /// @brief is_valid 
             /// 
             /// @return true if no duplicities of edges has been found
             /// ----------------------------------------------------------------------------
             bool is_valid(void) {
                 bool ret = true;
                 int v = 0;
                 while (v < nodes.size() && ret) {
                     Edges & edges(nodes[v].edges);
                     const int s = edges.size();
                     int i = 0;
                     while(i < s && ret) {
                         int j = i + 1;
                         while (j < s && ret) {
                             ret = edges[i].to != edges[j].to;
                             if (!ret) {
                                 std::cout << "Duplicate edge from " << v << "->"  << edges[i].to  << "i:" << i << " j:" << j << std::endl;
                             }
                             j++;
                         }
                         i++;
                     } //end first to
                     v++;
                 }
                 return ret;
             }


             void load(std::string filename){
                 int from;
                 int to;
                 Weight weight;

                 std::ifstream in(filename.c_str());
                 while(in >> from >> to >> weight) {
                     addEdge(from, to, weight);
                 }
             }
             void save(std::string filename){
                 std::fstream out;
                 out.open(filename.c_str(), std::ios::out);

                 for (int i = 0; i < nodes.size(); i++) {
                     out << i << " " << nodes[i].weight << " " << nodes[i].pred << std::endl;
                 }
             }
             void solve(){
                 solve(0);
             }

             void solve(unsigned int start, std::vector<int> & pred, std::vector<Weight> & weight) {
                 solve(start);
                 pred.clear();
                 weight.clear();
                 pred.reserve(nodes.size());
                 weight.reserve(nodes.size());
                 for (int i = 0; i < (int)nodes.size(); i++) {
                     pred.push_back(nodes[i].pred);
                     weight.push_back(nodes[i].weight);
                 }
             }

             void solve(unsigned int start){
                 int node_idx;
                 CHeap<Nodes, Weight> heap(nodes.size(), nodes);
                 for (int i = 0; i < (int)nodes.size(); i++) {
                     nodes[i].weight  = -1;
                     nodes[i].pred = -1;
                 }
                 /* TODO this does not work !!! ???
                    BOOST_FOREACH(SNode node, nodes) {
                    node.weight = -1;
                    node.pred = -1;
                    } */
                 nodes[start].weight = 0;
                 heap.add(start, 0);
                 while((node_idx = heap.getFirst()) != -1) {
                     SNode & node = nodes[node_idx];

                     for(int i=0;i<(int)node.edges.size();i++) {
                         SEdge &e = node.edges[i];
                         //		     BOOST_FOREACH(SEdge e, node.edges) {
                         SNode & to = nodes[e.to];
                         if (heap.getIDX(e.to) == -1) { 
                             if (to.weight  == -1) { //not  in close list
                                 to.weight = node.weight + e.weight;
                                 to.pred = node_idx;
                                 heap.add(e.to, to.weight);
                             }
                         } else {
                             Weight c = node.weight + e.weight;
                             if (to.weight > c) {
                                 to.weight = c;
                                 to.pred = node_idx;
                                 heap.update(e.to, to.weight);
                             }
                         }
                     } //end for loop
                     heap.getIDX(node_idx) = -1; 
                     }
                 }
                 private:

                 struct SEdge {
                     int to;
                     Weight weight;
                     SEdge(int to, Weight weight) : to(to), weight(weight) {}
                 };

                 typedef std::vector<SEdge> Edges;

                 struct SNode {
                     Weight weight;
                     int pred;
                     Edges edges;
                     SNode() : weight(0), pred(-1) {}
                 };

                 typedef std::vector<SNode> Nodes;
                 Nodes nodes;
             };

   } //end namespace dijkstra
} //end namespace imr


#endif

/* end of dijkstra_lite.h */
