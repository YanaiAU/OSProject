#include <iostream>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <unistd.h>
#include <arpa/inet.h>
#include "graph.h"
#include "mst_algorithm.h"
#include "mst_algorithm_factory.h"
#include "tree.h"

#define SERVER_PORT 8080
#define MAX_THREADS 4

class ThreadSafeQueue {
private:
    std::queue<int> queue;
    std::mutex mutex;
    std::condition_variable cond;
    std::atomic<bool> stop{false};

public:
    void push(int value) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(value);
        cond.notify_one();
    }

    bool pop(int& value) {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock, [this] { return !queue.empty() || stop; });

        if (stop && queue.empty()) {
            return false;
        }

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

class LeaderFollowerServer {
private:
    ThreadSafeQueue clientQueue;
    std::vector<std::thread> followers;
    std::atomic<bool> running{true};
    int serverSocket;

    void handleClient(int clientSocket) {
        std::cout << "[WORKER] Handling new client..." << std::endl;

        try {
            // Receive number of vertices
            int n;
            if (recv(clientSocket, &n, sizeof(n), 0) <= 0) {
                throw std::runtime_error("Error receiving vertex count");
            }
            std::cout << "[WORKER] Received vertices: " << n << std::endl;

            // Create graph
            Graph graph(n);
            Graph::Edge edge;
            for (int i = 0; i < n; ++i) {
                if (recv(clientSocket, &edge, sizeof(edge), 0) <= 0) {
                    throw std::runtime_error("Error receiving edge data");
                }
                graph.addEdge(edge.src, edge.dest, edge.weight);
            }

            // Receive algorithm choice
            int algorithmChoice;
            if (recv(clientSocket, &algorithmChoice, sizeof(algorithmChoice), 0) <= 0) {
                throw std::runtime_error("Error receiving algorithm choice");
            }

            // Create appropriate algorithm
            MSTAlgorithm* algorithm = MSTAlgorithmFactory::createAlgorithm(
                static_cast<AlgorithmType>(algorithmChoice));

            if (!algorithm) {
                throw std::runtime_error("Invalid algorithm choice");
            }

            // Convert graph to edge list format
            std::vector<std::tuple<int, int, int, int>> edges;
            int edgeId = 0;
            for (int i = 0; i < n; ++i) {
                for (const auto& e : graph.adjList[i]) {
                    edges.emplace_back(e.src, e.dest, e.weight, edgeId++);
                }
            }

            // Compute MST
            auto mstEdges = algorithm->findMST(edges, n);

            // Calculate total weight
            int totalWeight = 0;
            for (const auto& edge : mstEdges) {
                totalWeight += std::get<2>(edge);
            }

            // Send result back to client
            send(clientSocket, &totalWeight, sizeof(totalWeight), 0);

            delete algorithm;

        } catch (const std::exception& e) {
            std::cerr << "[WORKER] Error: " << e.what() << std::endl;
        }

        close(clientSocket);
    }

    void followerThread() {
        while (running) {
            int clientSocket;
            if (clientQueue.pop(clientSocket)) {
                handleClient(clientSocket);
            }
        }
    }

public:
    LeaderFollowerServer() : serverSocket(-1) {}

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

        // Create follower threads
        for (int i = 0; i < MAX_THREADS; ++i) {
            followers.emplace_back(&LeaderFollowerServer::followerThread, this);
        }

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

        // Close server socket to unblock accept()
        if (serverSocket != -1) {
            close(serverSocket);
        }

        // Wait for all followers to finish
        for (auto& thread : followers) {
            if (thread.joinable()) {
                thread.join();
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

    // Handle graceful shutdown
    std::thread([&server]() {
        char input;
        std::cin >> input;
        server.stop();
    }).detach();

    server.run();
    return 0;
}