#include "LeaderFollowerServer.hpp"
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "graph.h"
#include "mst_algorithm.h"
#include "mst_algorithm_factory.h"

#define SERVER_PORT 8080

LeaderFollowerServer::LeaderFollowerServer(int handlers) : numHandlers(handlers), serverSocket(-1) {
    for (int i = 0; i < numHandlers; i++) {
        metricPools.push_back(std::make_unique<WorkerPool>());
        clientHandlers.emplace_back(&LeaderFollowerServer::clientHandlerThread, this, i);
    }
}

LeaderFollowerServer::~LeaderFollowerServer() {
    stop();
}

void LeaderFollowerServer::handleClient(int clientSocket, WorkerPool& metricPool) {
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

        // Convert graph to edge list and compute MST
        std::vector<std::tuple<int, int, int, int>> edges;
        int edgeId = 0;
        for (int i = 0; i < n; ++i) {
            for (const auto& e : graph.adjList[i]) {
                if (e.src < e.dest) {
                    edges.emplace_back(e.src, e.dest, e.weight, edgeId++);
                }
            }
        }

        auto mstEdges = algorithm->findMST(edges, n);
        delete algorithm;

        // Create tasks for different metrics
        Task tasks[] = {
            {MetricResult::TOTAL_WEIGHT, mstEdges, n},
            {MetricResult::LONGEST_DISTANCE, mstEdges, n},
            {MetricResult::SHORTEST_EDGE, mstEdges, n},
            {MetricResult::AVERAGE_DISTANCE, mstEdges, n}
        };

        // Submit tasks
        for (const auto& task : tasks) {
            metricPool.submitTask(task);
        }

        // Prepare response
        ClientResponse response{0, 0, 0.0, 0, 0};

        // Collect metric results
        for (int i = 0; i < 4; i++) {
            MetricResult result;
            metricPool.getResult(result);

            switch (result.type) {
                case MetricResult::TOTAL_WEIGHT:
                    response.totalWeight = result.intValue;
                    break;
                case MetricResult::LONGEST_DISTANCE:
                    response.longestDistance = result.intValue;
                    break;
                case MetricResult::SHORTEST_EDGE:
                    response.shortestMSTEdge = result.intValue;
                    break;
                case MetricResult::AVERAGE_DISTANCE:
                    response.averageDistance = result.doubleValue;
                    break;
            }
        }

        // Add MST edges to response
        response.numMSTEdges = 0;
        for (const auto& edge : mstEdges) {
            Graph::Edge mstEdge;
            mstEdge.src = std::get<0>(edge);
            mstEdge.dest = std::get<1>(edge);
            mstEdge.weight = std::get<2>(edge);
            response.mstEdges[response.numMSTEdges++] = mstEdge;
        }

        // Send response
        send(clientSocket, &response, sizeof(response), 0);

        // Print results
        std::cout << "[HANDLER] MST Analysis Results:" << std::endl
                  << "├── Total Weight: " << response.totalWeight << std::endl
                  << "├── Longest Distance: " << response.longestDistance << std::endl
                  << "├── Average Distance: " << response.averageDistance << std::endl
                  << "└── Shortest MST Edge: " << response.shortestMSTEdge << std::endl;

        std::cout << "[HANDLER] MST Edges:" << std::endl;
        for (int i = 0; i < response.numMSTEdges; i++) {
            std::cout << "  " << response.mstEdges[i].src << " -> "
                     << response.mstEdges[i].dest << " (weight: "
                     << response.mstEdges[i].weight << ")" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[HANDLER] Error: " << e.what() << std::endl;
        ClientResponse errorResponse{-1, -1, -1.0, -1, 0};
        send(clientSocket, &errorResponse, sizeof(errorResponse), 0);
    }

    close(clientSocket);
}

void LeaderFollowerServer::clientHandlerThread(int handlerId) {
    while (running) {
        int clientSocket;
        if (!clientQueue.pop(clientSocket)) continue;
        
        std::cout << "[HANDLER " << handlerId << "] Processing client" << std::endl;
        handleClient(clientSocket, *metricPools[handlerId]);
    }
}

bool LeaderFollowerServer::start() {
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

void LeaderFollowerServer::run() {
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

void LeaderFollowerServer::stop() {
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