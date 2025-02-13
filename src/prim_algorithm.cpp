#include "../include/prim_algorithm.h"
#include <climits>


struct Edge {
    int w, to, id;
    bool operator<(Edge const& other) const {
        return std::make_pair(w, to) < std::make_pair(other.w, other.to);
    }
    Edge() : w(INT_MAX), to(-1), id(-1) {}
    Edge(int _w, int _to, int _id) : w(_w), to(_to), id(_id) {}
};

std::vector<std::tuple<int, int, int, int>> PrimAlgorithm::findMST(
    const std::vector<std::tuple<int, int, int, int>>& edges, int n) {

    std::vector<std::vector<Edge>> adj(n);
    for (const auto& e : edges) {
        int a, b, c, id;
        std::tie(a, b, c, id) = e;
        adj[a].emplace_back(c, b, id);
        adj[b].emplace_back(c, a, id);
    }

    std::vector<std::tuple<int, int, int, int>> spanning_tree;
    std::vector<Edge> min_e(n);
    min_e[0].w = 0;
    std::set<Edge> q;
    q.insert({0, 0, -1});
    std::vector<bool> selected(n, false);

    for (int i = 0; i < n; ++i) {
        int v = q.begin()->to;
        selected[v] = true;
        q.erase(q.begin());

        if (min_e[v].to != -1) {
            spanning_tree.emplace_back(min_e[v].to, v, min_e[v].w, min_e[v].id);
        }

        for (Edge e : adj[v]) {
            if (!selected[e.to] && e.w < min_e[e.to].w) {
                q.erase({min_e[e.to].w, e.to, e.id});
                min_e[e.to] = {e.w, v, e.id};
                q.insert({e.w, e.to, e.id});
            }
        }
    }
    return spanning_tree;
}
