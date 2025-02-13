#include "MetricsCalculator.hpp"

MetricResult MetricsCalculator::calculateTotalWeight(
    const std::vector<std::tuple<int, int, int, int>>& mstEdges) {
    
    MetricResult result;
    result.type = MetricResult::TOTAL_WEIGHT;
    
    int total = 0;
    for (const auto& edge : mstEdges) {
        total += std::get<2>(edge);
    }
    
    result.intValue = total;
    std::cout << "[METRIC] Calculated Total Weight: " << total << std::endl;
    return result;
}

std::vector<std::vector<int>> MetricsCalculator::buildDistanceMatrix(
    const std::vector<std::tuple<int, int, int, int>>& mstEdges, 
    int numVertices) {
    
    std::vector<std::vector<int>> dist(numVertices, 
        std::vector<int>(numVertices, std::numeric_limits<int>::max()));
    
    // Initialize with direct edges from MST
    for (int i = 0; i < numVertices; i++) {
        dist[i][i] = 0;
    }
    
    // Add MST edges (directed!)
    for (const auto& edge : mstEdges) {
        int u = std::get<0>(edge);
        int v = std::get<1>(edge);
        int w = std::get<2>(edge);
        dist[u][v] = w;
    }

    // Floyd-Warshall
    for (int k = 0; k < numVertices; k++) {
        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < numVertices; j++) {
                if (dist[i][k] != std::numeric_limits<int>::max() && 
                    dist[k][j] != std::numeric_limits<int>::max()) {
                    int newDist = dist[i][k] + dist[k][j];
                    if (newDist < dist[i][j]) {
                        dist[i][j] = newDist;
                    }
                }
            }
        }
    }
    
    return dist;
}

MetricResult MetricsCalculator::calculateLongestDistance(
    const std::vector<std::tuple<int, int, int, int>>& mstEdges, 
    int numVertices) {
    
    MetricResult result;
    result.type = MetricResult::LONGEST_DISTANCE;
    
    auto dist = buildDistanceMatrix(mstEdges, numVertices);

    // Find maximum reachable distance
    int maxDist = 0;
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            if (i != j && dist[i][j] != std::numeric_limits<int>::max()) {
                maxDist = std::max(maxDist, dist[i][j]);
            }
        }
    }

    result.intValue = maxDist;
    std::cout << "[METRIC] Calculated Longest Distance: " << maxDist << std::endl;
    return result;
}

MetricResult MetricsCalculator::calculateShortestEdge(
    const std::vector<std::tuple<int, int, int, int>>& mstEdges) {
    
    MetricResult result;
    result.type = MetricResult::SHORTEST_EDGE;
    
    int minDist = std::numeric_limits<int>::max();
    for (const auto& edge : mstEdges) {
        minDist = std::min(minDist, std::get<2>(edge));
    }
    
    result.intValue = minDist;
    std::cout << "[METRIC] Calculated Shortest Edge: " << minDist << std::endl;
    return result;
}

MetricResult MetricsCalculator::calculateAverageDistance(
    const std::vector<std::tuple<int, int, int, int>>& mstEdges, 
    int numVertices) {
    
    MetricResult result;
    result.type = MetricResult::AVERAGE_DISTANCE;
    
    auto dist = buildDistanceMatrix(mstEdges, numVertices);

    // Calculate average for reachable pairs where j≥i
    long long sum = 0;
    int count = 0;
    for (int i = 0; i < numVertices; i++) {
        for (int j = i; j < numVertices; j++) {
            if (i != j && dist[i][j] != std::numeric_limits<int>::max()) {
                sum += dist[i][j];
                count++;
            }
        }
    }
    
    result.doubleValue = count > 0 ? static_cast<double>(sum) / count : 0.0;
    std::cout << "[METRIC] Calculated Average Distance (sum=" << sum 
              << ", count=" << count << "): " << result.doubleValue << std::endl;
    return result;
}