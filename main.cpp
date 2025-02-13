#include "include/mst_algorithm_factory.h"
#include <iostream>
#include <vector>

int main() {
    // Example graph: (from, to, weight, id)
    std::vector<std::tuple<int, int, int, int>> edges = {
        {0, 1, 10, 1}, {0, 2, 6, 2}, {0, 3, 5, 3}, {1, 3, 15, 4}, {2, 3, 4, 5}
    };
    int n = 4;

    // Create Prim's Algorithm instance
    MSTAlgorithm* primMST = MSTAlgorithmFactory::createAlgorithm(PRIM);
    auto primResult = primMST->findMST(edges, n);

    std::cout << "Prim's MST:\n";
    for (auto edge : primResult) {
        std::cout << std::get<0>(edge) << " - " << std::get<1>(edge)
                  << " : " << std::get<2>(edge) << "\n";
    }
    delete primMST;

    // Create Kruskal's Algorithm instance
    MSTAlgorithm* kruskalMST = MSTAlgorithmFactory::createAlgorithm(KRUSKAL);
    auto kruskalResult = kruskalMST->findMST(edges, n);

    std::cout << "\nKruskal's MST:\n";
    for (auto edge : kruskalResult) {
        std::cout << std::get<0>(edge) << " - " << std::get<1>(edge)
                  << " : " << std::get<2>(edge) << "\n";
    }
    delete kruskalMST;

    return 0;
}
