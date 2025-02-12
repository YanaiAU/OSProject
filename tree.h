#ifndef TREE_H
#define TREE_H

#include "graph.h"

class Tree : public Graph {
public:
    Tree(int V);

    // Additional methods for tree operations
    void addTreeEdge(int src, int dest, int weight);
};

#endif // TREE_H
