#ifndef UNION_FIND_H
#define UNION_FIND_H

#include <vector>

class UnionFind {
public:
    UnionFind(int _n);
    int find_parent(int node);
    bool unite(int x, int y);

private:
    std::vector<int> parent, rank;
    int n, cc;
};

#endif // UNION_FIND_H
