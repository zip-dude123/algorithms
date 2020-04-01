#ifndef WPGRAPH_H
#define WPGRAPH_H

#include <unordered_map>
#include <vector>

namespace WPAlgos
{/*
  template<class Edge, class Node>
  class Graph
  {
    public:
      Graph();
      Graph(std::vector<Node> node, std::vector<Edge> edges);

      void addNode();
      void removeNode();
      void addEdge();
      void removeEdge();
      //const BFStruct BellmanFord();

      const int numNodes() { return m_nodes.size(); }
      const int numEdges() { return m_edges.size(); }


    private:
      std::vector<Node> m_nodes;
      std::vector<Edge> m_edges;

      PrimGraph m_PrimGraph;

  };*/

  struct BFRetStruct
  {
    std::vector<uint32_t> path;
    int length;
  };

  class PrimGraph
  {
    public:
      PrimGraph();

      void addNode(uint32_t id);
      void removeNode(uint32_t id);

      void addEdge(uint64_t edge, double weight);
      void addEdge(uint32_t u, uint32_t v, double weight);
      void removeEdge(uint64_t edge);
      void removeEdge(uint32_t u, uint32_t v);

      BFRetStruct BF(uint32_t u, uint32_t v);
      BFRetStruct BF(uint32_t u);




    private:
      std::unordered_map<uint32_t,bool> m_nodes;
      std::unordered_map<uint64_t,bool> m_edges;
      std::unordered_map<uint64_t,double> m_weights;

      uint32_t m_numNodes;

  };


}


#endif
