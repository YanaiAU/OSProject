#include "graph.h"

Graph::Graph(int V) : V(V) {
    adjList.resize(V);
}

void Graph::addEdge(int src, int dest, int weight) {
    Edge edge = {src, dest, weight};
    adjList[src].push_back(edge);
}
