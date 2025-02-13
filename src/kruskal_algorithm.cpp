#include "../include/kruskal_algorithm.h"
#include <iostream>

std::vector<std::tuple<int, int, int, int>> KruskalAlgorithm::findMST(
    const std::vector<std::tuple<int, int, int, int>>& graph_edges, int n) {

    UnionFind graph(n);
    std::vector<std::tuple<int, int, int, int>> edges = graph_edges;
    std::vector<std::tuple<int, int, int, int>> spanning_tree;

    std::cout << "\n[DEBUG] Input edges to Kruskal:" << std::endl;
    for (const auto& e : edges) {
        std::cout << "  " << std::get<0>(e) << " -> " << std::get<1>(e)
                 << " (weight: " << std::get<2>(e) << ")" << std::endl;
    }

    // Sort edges by weight
    std::sort(edges.begin(), edges.end(),
              [](const auto& a, const auto& b) { return std::get<2>(a) < std::get<2>(b); });

    std::cout << "\n[DEBUG] Sorted edges:" << std::endl;
    for (const auto& e : edges) {
        std::cout << "  " << std::get<0>(e) << " -> " << std::get<1>(e)
                 << " (weight: " << std::get<2>(e) << ")" << std::endl;
    }

    // Process edges in ascending weight order
    for (const auto& edge : edges) {
        int from, to, cost, id;
        std::tie(from, to, cost, id) = edge;

        if (graph.find_parent(from) != graph.find_parent(to)) {
            graph.unite(from, to);
            spanning_tree.emplace_back(from, to, cost, id);
            std::cout << "[DEBUG] Added MST edge: " << from << " -> " << to
                     << " (weight: " << cost << ")" << std::endl;
        }
    }

    std::cout << "\n[DEBUG] Final MST edges:" << std::endl;
    for (const auto& e : spanning_tree) {
        std::cout << "  " << std::get<0>(e) << " -> " << std::get<1>(e)
                 << " (weight: " << std::get<2>(e) << ")" << std::endl;
    }

    return spanning_tree;
}