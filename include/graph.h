#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>

class Graph {
public:
    struct Edge {
        int src, dest, weight;
    };

    int V;
    std::vector<std::list<Edge>> adjList;

    Graph(int V);
    void addEdge(int src, int dest, int weight);
    std::vector<std::tuple<int, int, int, int>> getEdges();
    int getNumVertices();

};

#endif // GRAPH_H
