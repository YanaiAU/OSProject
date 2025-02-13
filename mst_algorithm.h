#ifndef MST_ALGORITHM_H
#define MST_ALGORITHM_H

#include <vector>
#include <tuple>

class MSTAlgorithm {
public:
    virtual ~MSTAlgorithm() = default;
    virtual std::vector<std::tuple<int, int, int, int>> findMST(
        const std::vector<std::tuple<int, int, int, int>>& edges, int n) = 0;
};

#endif // MST_ALGORITHM_H