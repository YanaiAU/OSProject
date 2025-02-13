#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <memory>
#include <unistd.h>
#include <arpa/inet.h>
#include "include/graph.h"
#include "include/mst_algorithm.h"
#include "include/mst_algorithm_factory.h"

#define SERVER_PORT 8080
#define MAX_CLIENT_HANDLERS 3
#define MAX_METRIC_WORKERS 4

#include <queue>

// Add this function to normalize edge directions in MST
std::vector<std::tuple<int, int, int, int>> normalizeEdgeDirections(
    const std::vector<std::tuple<int, int, int, int>>& mstEdges,
    int numVertices,
    int root = 0) {

    // Create adjacency list representation
    std::vector<std::vector<std::pair<int, std::tuple<int, int, int, int>>>> adj(numVertices);
    for (const auto& edge : mstEdges) {
        int u = std::get<0>(edge);
        int v = std::get<1>(edge);
        adj[u].push_back({v, edge});
        adj[v].push_back({u, edge});
    }

    // BFS to orient edges away from root
    std::vector<bool> visited(numVertices, false);
    std::queue<int> q;
    std::vector<std::tuple<int, int, int, int>> normalizedEdges;

    q.push(root);
    visited[root] = true;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (const auto& [v, edge] : adj[u]) {
            if (!visited[v]) {
                visited[v] = true;
                q.push(v);

                // Orient edge from u to v
                int src = u;
                int dst = v;
                int weight = std::get<2>(edge);
                int id = std::get<3>(edge);
                normalizedEdges.emplace_back(src, dst, weight, id);
            }
        }
    }

    return normalizedEdges;
}


// Thread-safe queue template
template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable cond;
    std::atomic<bool> stop{false};

public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(value);
        cond.notify_one();
    }

    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock, [this] { return !queue.empty() || stop; });
        if (stop && queue.empty()) return false;
        value = queue.front();
        queue.pop();
        return true;
    }

    void stopQueue() {
        std::lock_guard<std::mutex> lock(mutex);
        stop = true;
        cond.notify_all();
    }
};

// Metric calculation structures
struct MetricTask {
    enum Type {
        TOTAL_WEIGHT,
        LONGEST_DISTANCE,
        SHORTEST_EDGE,         // Changed back from SHORTEST_MST_DISTANCE
        AVERAGE_DISTANCE
    };
    Type type;
    std::vector<std::tuple<int, int, int, int>> mstEdges;
    int numVertices;
};

struct MetricResult {
    MetricTask::Type type;
    union {
        int intValue;
        double doubleValue;
    };
};

struct ClientResponse {
    int totalWeight;
    int longestDistance;
    double averageDistance;
    int shortestMSTEdge;
};

// Metric Worker Pool (Inner level leader-follower)
class MetricWorkerPool {
private:
    ThreadSafeQueue<MetricTask> taskQueue;
    ThreadSafeQueue<MetricResult> resultQueue;
    std::vector<std::thread> workers;
    std::atomic<bool> running{true};

void calculateMetric(const MetricTask& task, MetricResult& result) {
    result.type = task.type;

    switch (task.type) {
        case MetricTask::TOTAL_WEIGHT: {
            int total = 0;
            for (const auto& edge : task.mstEdges) {
                total += std::get<2>(edge);
            }
            result.intValue = total;
            std::cout << "[METRIC WORKER] Calculated Total Weight: " << total << std::endl;
            break;
        }
        case MetricTask::LONGEST_DISTANCE: {
            // Initialize distance matrix with infinity
            std::vector<std::vector<int>> dist(task.numVertices,
                std::vector<int>(task.numVertices, std::numeric_limits<int>::max()));

            // Initialize distances with direct edges from MST
            for (int i = 0; i < task.numVertices; i++) {
                dist[i][i] = 0;
            }

            // Add ONLY the MST edges (directed!)
            for (const auto& edge : task.mstEdges) {
                int u = std::get<0>(edge);
                int v = std::get<1>(edge);
                int w = std::get<2>(edge);
                dist[u][v] = w;  // Only in direction from u to v!
            }

            // Floyd-Warshall for directed graph
            for (int k = 0; k < task.numVertices; k++) {
                for (int i = 0; i < task.numVertices; i++) {
                    for (int j = 0; j < task.numVertices; j++) {
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

            // Debug output to show all valid paths
            std::cout << "\n[DEBUG] Distance matrix for MST (inf = no path):" << std::endl;
            for (int i = 0; i < task.numVertices; i++) {
                for (int j = 0; j < task.numVertices; j++) {
                    if (dist[i][j] == std::numeric_limits<int>::max()) {
                        std::cout << "inf ";
                    } else {
                        std::cout << dist[i][j] << " ";
                    }
                }
                std::cout << std::endl;
            }

            // Find maximum reachable distance
            int maxDist = 0;
            for (int i = 0; i < task.numVertices; i++) {
                for (int j = 0; j < task.numVertices; j++) {
                    if (i != j && dist[i][j] != std::numeric_limits<int>::max()) {
                        maxDist = std::max(maxDist, dist[i][j]);
                    }
                }
            }

            result.intValue = maxDist;
            std::cout << "[METRIC WORKER] Calculated Longest Distance: " << maxDist << std::endl;
            break;
        }
        case MetricTask::SHORTEST_EDGE: {
            int minDist = std::numeric_limits<int>::max();
            for (const auto& edge : task.mstEdges) {
                minDist = std::min(minDist, std::get<2>(edge));
            }
            result.intValue = minDist;
            std::cout << "[METRIC WORKER] Calculated Shortest Edge: " << minDist << std::endl;
            break;
        }
        case MetricTask::AVERAGE_DISTANCE: {
            // Same initialization as longest distance
            std::vector<std::vector<int>> dist(task.numVertices,
                std::vector<int>(task.numVertices, std::numeric_limits<int>::max()));

            for (int i = 0; i < task.numVertices; i++) {
                dist[i][i] = 0;
            }

            for (const auto& edge : task.mstEdges) {
                int u = std::get<0>(edge);
                int v = std::get<1>(edge);
                int w = std::get<2>(edge);
                dist[u][v] = w;  // Only directed edges!
            }

            // Floyd-Warshall
            for (int k = 0; k < task.numVertices; k++) {
                for (int i = 0; i < task.numVertices; i++) {
                    for (int j = 0; j < task.numVertices; j++) {
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

            // Calculate average for reachable pairs where j≥i
            long long sum = 0;
            int count = 0;
            for (int i = 0; i < task.numVertices; i++) {
                for (int j = i; j < task.numVertices; j++) {
                    if (i != j && dist[i][j] != std::numeric_limits<int>::max()) {
                        sum += dist[i][j];
                        count++;
                        std::cout << "[DEBUG] Valid path " << i << "->" << j
                                 << " = " << dist[i][j] << std::endl;
                    }
                }
            }

            result.doubleValue = count > 0 ? static_cast<double>(sum) / count : 0.0;
            std::cout << "[METRIC WORKER] Calculated Average Distance (sum=" << sum
                      << ", count=" << count << "): " << result.doubleValue << std::endl;
            break;
        }
    }
}

    void workerThread() {
        while (running) {
            MetricTask task;
            if (!taskQueue.pop(task)) continue;

            MetricResult result;
            calculateMetric(task, result);
            resultQueue.push(result);
        }
    }

public:
    MetricWorkerPool() {
        for (int i = 0; i < MAX_METRIC_WORKERS; i++) {
            workers.emplace_back(&MetricWorkerPool::workerThread, this);
        }
    }

    ClientResponse processMetrics(const std::vector<std::tuple<int, int, int, int>>& mstEdges, int numVertices) {
        std::vector<MetricTask::Type> types = {
            MetricTask::TOTAL_WEIGHT,
            MetricTask::LONGEST_DISTANCE,
            MetricTask::SHORTEST_EDGE,
            MetricTask::AVERAGE_DISTANCE
        };

        // Distribute tasks to workers
        for (auto type : types) {
            taskQueue.push({type, mstEdges, numVertices});
        }

        // Collect results
        ClientResponse response{0, 0, 0.0, 0};
        for (size_t i = 0; i < types.size(); i++) {
            MetricResult result;
            resultQueue.pop(result);

            switch (result.type) {
                case MetricTask::TOTAL_WEIGHT:
                    response.totalWeight = result.intValue;
                    break;
                case MetricTask::LONGEST_DISTANCE:
                    response.longestDistance = result.intValue;
                    break;
                case MetricTask::SHORTEST_EDGE:
                    response.shortestMSTEdge = result.intValue;
                    break;
                case MetricTask::AVERAGE_DISTANCE:
                    response.averageDistance = result.doubleValue;
                    break;
            }
        }

        return response;
    }

    ~MetricWorkerPool() {
        running = false;
        taskQueue.stopQueue();
        for (auto& worker : workers) {
            if (worker.joinable()) worker.join();
        }
    }
};

// Main Server (Outer level leader-follower)
class LeaderFollowerServer {
private:
    ThreadSafeQueue<int> clientQueue;
    std::vector<std::thread> clientHandlers;
    std::vector<std::unique_ptr<MetricWorkerPool>> metricPools;
    std::atomic<bool> running{true};
    int serverSocket;

    void handleClient(int clientSocket, MetricWorkerPool& metricPool) {
        try {
            // Receive number of vertices
            int n;
            if (recv(clientSocket, &n, sizeof(n), 0) <= 0) {
                throw std::runtime_error("Error receiving vertex count");
            }
            std::cout << "[HANDLER] Received vertices: " << n << std::endl;

            // Create graph and receive edges
            Graph graph(n);
            const int EXPECTED_EDGES = 5;
            Graph::Edge edge;

            for (int i = 0; i < EXPECTED_EDGES; i++) {
                if (recv(clientSocket, &edge, sizeof(edge), 0) <= 0) {
                    throw std::runtime_error("Error receiving edge data");
                }
                std::cout << "[HANDLER] Received edge: " << edge.src << " -> " << edge.dest
                         << " (weight: " << edge.weight << ")" << std::endl;
                graph.addEdge(edge.src, edge.dest, edge.weight);
            }

            // Receive algorithm choice
            int algorithmChoice;
            if (recv(clientSocket, &algorithmChoice, sizeof(algorithmChoice), 0) <= 0) {
                throw std::runtime_error("Error receiving algorithm choice");
            }

            // Create MST algorithm
            MSTAlgorithm* algorithm = nullptr;
            if (algorithmChoice == 0) {
                algorithm = MSTAlgorithmFactory::createAlgorithm(PRIM);
            } else if (algorithmChoice == 1) {
                algorithm = MSTAlgorithmFactory::createAlgorithm(KRUSKAL);
            } else {
                throw std::runtime_error("Invalid algorithm choice");
            }

            if (!algorithm) {
                throw std::runtime_error("Failed to create algorithm");
            }

            // Convert graph to edge list
            std::vector<std::tuple<int, int, int, int>> edges;
            int edgeId = 0;
            for (int i = 0; i < n; ++i) {
                for (const auto& e : graph.adjList[i]) {
                    if (e.src < e.dest) {
                        edges.emplace_back(e.src, e.dest, e.weight, edgeId++);
                    }
                }
            }

            // Compute MST
            auto mstEdges = algorithm->findMST(edges, n);
            mstEdges = normalizeEdgeDirections(mstEdges, n, 0);
            delete algorithm;



            // Use metric pool to calculate metrics in parallel
            ClientResponse response = metricPool.processMetrics(mstEdges, n);
            std::cout << "\n[HANDLER] MST Analysis Results:" << std::endl;
            std::cout << "├── Total Weight: " << response.totalWeight << std::endl;
            std::cout << "├── Longest Distance: " << response.longestDistance << std::endl;
            std::cout << "├── Average Distance: " << response.averageDistance << std::endl;
            std::cout << "└── Shortest MST Edge: " << response.shortestMSTEdge << std::endl;

            // Send response to client
            send(clientSocket, &response, sizeof(response), 0);

        } catch (const std::exception& e) {
            std::cerr << "[HANDLER] Error: " << e.what() << std::endl;
            ClientResponse errorResponse{-1, -1, -1.0, -1};
            send(clientSocket, &errorResponse, sizeof(errorResponse), 0);
        }

        close(clientSocket);
    }

    void clientHandlerThread(int handlerId) {
        while (running) {
            int clientSocket;
            if (!clientQueue.pop(clientSocket)) continue;

            std::cout << "[HANDLER " << handlerId << "] Processing client" << std::endl;
            handleClient(clientSocket, *metricPools[handlerId]);
        }
    }

public:
    LeaderFollowerServer() : serverSocket(-1) {
        // Create client handlers and their metric pools
        for (int i = 0; i < MAX_CLIENT_HANDLERS; i++) {
            metricPools.push_back(std::make_unique<MetricWorkerPool>());
            clientHandlers.emplace_back(&LeaderFollowerServer::clientHandlerThread, this, i);
        }
    }

    bool start() {
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket < 0) {
            std::cerr << "[SERVER] Socket creation failed" << std::endl;
            return false;
        }

        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(SERVER_PORT);

        int opt = 1;
        setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "[SERVER] Bind failed" << std::endl;
            return false;
        }

        if (listen(serverSocket, SOMAXCONN) < 0) {
            std::cerr << "[SERVER] Listen failed" << std::endl;
            return false;
        }

        std::cout << "[SERVER] Started on port " << SERVER_PORT << std::endl;
        return true;
    }

    void run() {
        while (running) {
            sockaddr_in clientAddr{};
            socklen_t clientAddrLen = sizeof(clientAddr);

            int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientAddrLen);
            if (clientSocket < 0) {
                if (running) {
                    std::cerr << "[LEADER] Accept failed" << std::endl;
                }
                continue;
            }

            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);
            std::cout << "[LEADER] New connection from " << clientIP << std::endl;

            clientQueue.push(clientSocket);
        }
    }

    void stop() {
        running = false;
        clientQueue.stopQueue();

        if (serverSocket != -1) {
            close(serverSocket);
        }

        for (auto& handler : clientHandlers) {
            if (handler.joinable()) {
                handler.join();
            }
        }
    }

    ~LeaderFollowerServer() {
        stop();
    }
};

int main() {
    LeaderFollowerServer server;

    if (!server.start()) {
        return 1;
    }

    std::thread([&server]() {
        char input;
        std::cin >> input;
        server.stop();
    }).detach();

    server.run();
    return 0;
}