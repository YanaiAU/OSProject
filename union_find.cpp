#include "include/union_find.h"
#include <numeric>

UnionFind::UnionFind(int _n) {
    n = _n;
    cc = n;
    parent.resize(n);
    rank.assign(n, 1);
    std::iota(parent.begin(), parent.end(), 0);
}

int UnionFind::find_parent(int node) {
    if (parent[node] != node)
        parent[node] = find_parent(parent[node]); // Path compression
    return parent[node];
}

bool UnionFind::unite(int x, int y) {
    x = find_parent(x);
    y = find_parent(y);
    if (x == y) return false; // Already in the same set
    if (rank[x] < rank[y])
        parent[x] = y;
    else {
        parent[y] = x;
        rank[x] += (rank[x] == rank[y]);
    }
    --cc;
    return true;
}
