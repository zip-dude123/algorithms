#ifndef WPGRAPH_H
#define WPGRAPH_H

#include <unordered_map>

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
      void addNode(uint32_t id);
      void removeNode(uint32_t id);

      void addEdge(uint64_t edge);
      void addEdge(uint32_t u, uint32_t v);
      void removeEdge(uint64_t edge);
      void removeEdge(uint32_t u, uint32_t v);




    private:
      std::unordered_map<uint32_t> m_nodes;
      std::unordered_map<uint64_t,std::pair<bool, double>> m_edges;

      uint32_t numNodes;

  };


}


#endif
