#ifndef METRICS_CALCULATOR_HPP
#define METRICS_CALCULATOR_HPP

#include <vector>
#include <tuple>
#include <limits>
#include <iostream>

struct MetricResult {
    enum Type { 
        TOTAL_WEIGHT,
        LONGEST_DISTANCE,
        SHORTEST_EDGE,
        AVERAGE_DISTANCE 
    };
    Type type;
    union {
        int intValue;
        double doubleValue;
    };
};

class MetricsCalculator {
public:
    static MetricResult calculateTotalWeight(const std::vector<std::tuple<int, int, int, int>>& mstEdges);
    static MetricResult calculateLongestDistance(const std::vector<std::tuple<int, int, int, int>>& mstEdges, int numVertices);
    static MetricResult calculateShortestEdge(const std::vector<std::tuple<int, int, int, int>>& mstEdges);
    static MetricResult calculateAverageDistance(const std::vector<std::tuple<int, int, int, int>>& mstEdges, int numVertices);

private:
    static std::vector<std::vector<int>> buildDistanceMatrix(
        const std::vector<std::tuple<int, int, int, int>>& mstEdges, 
        int numVertices);
};

#endif // METRICS_CALCULATOR_HPP