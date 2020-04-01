//graph.cpp

#include <limits>
#include <vector>
#include <iostream>

#include "Graph.h"

#define MASK64_32  0x00000000FFFFFFFF

using namespace WPAlgos;

PrimGraph::PrimGraph(): m_numNodes{0} {};

void PrimGraph::addNode(uint32_t id)
{
  m_nodes[id] = true;
  m_numNodes++;
}

void PrimGraph::removeNode(uint32_t id)
{
  m_nodes[id] = false;
  m_numNodes--;
}

void PrimGraph::addEdge(uint64_t edge, double weight)
{
  m_edges[edge] = true;
  m_weights[edge] = weight;
}

void PrimGraph::addEdge(uint32_t u, uint32_t v, double weight)
{
  uint64_t u64 = (uint64_t) u;
  m_edges[(u64 << 32) + (v & MASK64_32)] = true;
  m_weights[(u64 << 32) + (v & MASK64_32)] = weight;
}

void PrimGraph::removeEdge(uint64_t edge)
{
  m_edges[edge] = false;
}

void PrimGraph::removeEdge(uint32_t u, uint32_t v)
{
  uint64_t u64 = (uint64_t) u;
  m_edges[(u64 << 32) + (v & MASK64_32)] = false;
}

BFRetStruct PrimGraph::BF(uint32_t u, uint32_t v)
{
  BFRetStruct ret;

  //set distances to +inf
  std::unordered_map<uint32_t, double> distances;
  std::unordered_map<uint32_t, std::vector<uint32_t>> paths;
  std::cout << m_numNodes << std::endl;
  for (uint32_t i = 0; i < m_numNodes; i++)
  {
    if (m_nodes[i]) distances[i] = std::numeric_limits<double>::max();
  }
  distances[u] = 0;

  for (uint32_t i = 0; i < m_numNodes; i++)
  {
    for (auto it : m_edges)
    {
      uint64_t edge = it.first;
      uint32_t u = (edge >> 32) & MASK64_32;
      uint32_t v = edge & MASK64_32;

      if (distances[u] + m_weights[edge] < distances[v])
      {
        distances[v] = distances[u] + m_weights[edge];
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
