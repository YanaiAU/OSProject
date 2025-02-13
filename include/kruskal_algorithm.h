#ifndef KRUSKAL_ALGORITHM_H
#define KRUSKAL_ALGORITHM_H

#include "mst_algorithm.h"
#include "union_find.h"
#include <vector>
#include <tuple>
#include <algorithm>

class KruskalAlgorithm : public MSTAlgorithm {
public:
    std::vector<std::tuple<int, int, int, int>> findMST(
        const std::vector<std::tuple<int, int, int, int>>& edges, int n) override;
};

#endif // KRUSKAL_ALGORITHM_H
