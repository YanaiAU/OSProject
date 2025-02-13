#ifndef PRIM_ALGORITHM_H
#define PRIM_ALGORITHM_H

#include "mst_algorithm.h"
#include <vector>
#include <tuple>
#include <set>

class PrimAlgorithm : public MSTAlgorithm {
public:
    std::vector<std::tuple<int, int, int, int>> findMST(
        const std::vector<std::tuple<int, int, int, int>>& edges, int n) override;
};

#endif // PRIM_ALGORITHM_H
