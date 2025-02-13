/*#include "mst_algorithm.h"
#include <queue>
#include <algorithm>
#include <limits>
#include <functional>

MSTAlgorithm* MSTAlgorithmFactory::createAlgorithm(AlgorithmType type) {
    switch (type) {
        case PRIM:
            return new PrimAlgorithm();
        case KRUSKAL:
            return new KruskalAlgorithm();
        default:
            return nullptr;
    }
}

void PrimAlgorithm::findMST(Graph &graph, Tree &mst) {
    std::vector<bool> visited(graph.V, false);
    visited[0] = true;
    int count = 1;

    while (count < graph.V) {
        int minWeight = std::numeric_limits<int>::max();
        int minSrc = -1, minDest = -1;

        for (int i = 0; i < graph.V; i++) {
            if (visited[i]) {
                for (const auto &edge : graph.adjList[i]) {
                    if (!visited[edge.dest] && edge.weight < minWeight) {
                        minWeight = edge.weight;
                        minSrc = i;
                        minDest = edge.dest;
                    }
                }
            }
        }

        if (minSrc != -1) {
            mst.addTreeEdge(minSrc, minDest, minWeight);
            visited[minDest] = true;
            count++;
        }
    }
}

void KruskalAlgorithm::findMST(Graph &graph, Tree &mst) {
    std::vector<Graph::Edge> edges;
    for (int u = 0; u < graph.V; u++) {
        for (const auto &edge : graph.adjList[u]) {
            edges.push_back(edge);
        }
    }

    std::sort(edges.begin(), edges.end(),
        [](const Graph::Edge &a, const Graph::Edge &b) { return a.weight < b.weight; });

    std::vector<int> parent(graph.V);
    for (int i = 0; i < graph.V; i++) parent[i] = i;

    std::function<int(int)> find = [&](int u) {
        if (parent[u] != u) parent[u] = find(parent[u]);
        return parent[u];
    };

    int edgeCount = 0;
    for (const auto &edge : edges) {
        if (edgeCount >= graph.V - 1) break;

        int set1 = find(edge.src);
        int set2 = find(edge.dest);

        if (set1 != set2) {
            mst.addTreeEdge(edge.src, edge.dest, edge.weight);
            parent[set2] = set1;
            edgeCount++;
        }
    }
}
*/