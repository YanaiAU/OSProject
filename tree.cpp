#include "tree.h"

Tree::Tree(int V) : Graph(V) {}

void Tree::addTreeEdge(int src, int dest, int weight) {
    addEdge(src, dest, weight);
}
