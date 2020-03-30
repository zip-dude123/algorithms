//graph.cpp

#include <limits>
#include <vector>

#include "Graph.h"

#define MASK64_32  0x00000000FFFFFFFF;

using namespace WPAlgos;

void PrimGraph::addNode(uint32_t id)
{
  m_nodes[id] = true;
  m_numNodes++;
}

void PrimGraph::removeNode(uint32_t id)
{
  m_nodes[id] = false;
  numNodes--;
}

void PrimGraph::addEdge(uint64_t edge)
{
  m_edges[id] = true;
}

void PrimGraph::addEdge(uint64_t u, uint64_t v)
{
  m_edges[(u << 32) + (v & MASK64_32)] = true;
}

void PrimGraph::removeEdge(uint64_t edge)
{
  m_edges[id] = false;
}

void PrimGraph::removeEdge(uint64_t u, uint64_t v)
{
  m_edges[(u << 32) + (v & MASK64_32)] = false;
}

BFRetStruct PrimGraph::BF(uint32_t u, uint32_t v)
{
  BFRetStruct ret;

  //set distances to +inf
  std::unordered_map<uint32_t, double> distances;
  std::unordered_map<uint32_t, std::vector<uint32_t>> paths;
  for (uint32_t i : m_nodes)
  {
    if (m_nodes[i]) distances[i] = std::numeric_limits<double>::max;
  }

  for (uint32_t i : numNodes)
  {
    for (uint64_t edge : m_edges)
    {
      uint32_t u = (edge >> 32) & MASK64_32;
      uint32_t v = edge & MASK64_32;

      if (distances[u] + m_edges[edge].second < distances[v])
      {
        distances[v] = distances[u] + m_edges[edge].second;
        paths[v] = paths[u];
        paths[v].push_back(edge);
      }
    }
  }

  ret.path = paths[v];
  ret.length = distances[v];
  return ret;
}

BFRetStruct PrimGraph::BF(uint32_t u)
{
  return PrimGraph::BF(u, u);
}
