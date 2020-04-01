#include "Graph.h"
#include <iostream>

int main()
{
  WPAlgos::PrimGraph graph;
  for (uint32_t i = 0; i < 5; i++)
  {
    graph.addNode(i);
  }

  graph.addEdge(1,2,1);
  graph.addEdge(1,3,1);
  graph.addEdge(1,4,2);
  graph.addEdge(2,3,1);
  graph.addEdge(4,5,-3);
  graph.addEdge(5,1,4);
  graph.addEdge(5,3,1);

  WPAlgos::BFRetStruct ret = graph.BF(1,3);

  std::cout << ret.length << std::endl;

  return 0;
}
