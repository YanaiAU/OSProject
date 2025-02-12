#include "kruskal_algorithm.h"

std::vector<std::tuple<int, int, int, int>> KruskalAlgorithm::findMST(
    const std::vector<std::tuple<int, int, int, int>>& graph_edges, int n) {

    UnionFind graph(n);
    std::vector<std::tuple<int, int, int, int>> edges = graph_edges;
    std::vector<std::tuple<int, int, int, int>> spanning_tree;

    std::sort(edges.begin(), edges.end(),
              [](const auto& a, const auto& b) { return std::get<2>(a) < std::get<2>(b); });

    for (const auto& edge : edges) {
        int from, to, cost, id;
        std::tie(from, to, cost, id) = edge;
        if (graph.unite(from, to)) {
            spanning_tree.emplace_back(from, to, cost, id);
        }
    }

    return spanning_tree;
}
