#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include "include/graph.h"
#include "include/mst_algorithm.h"

// Define the server port
#define SERVER_PORT 8080

void computeMST(Graph &graph, Tree &mst, MSTAlgorithm *algorithm) {
    std::cout << "[SERVER] Computing MST..." << std::endl;
    algorithm->findMST(graph, mst);
    std::cout << "[SERVER] MST Computation done." << std::endl;
}

int main() {
    int serverSocket, clientSocket;
    struct sockaddr_in serverAddr, clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    // Create socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        std::cerr << "[SERVER] Socket creation failed" << std::endl;
        return -1;
    }
    std::cout << "[SERVER] Starting server..." << std::endl;

    // Set up server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    serverAddr.sin_port = htons(SERVER_PORT);

    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind the socket to the port
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "[SERVER] Bind failed" << std::endl;
        return -1;
    }
    std::cout << "[SERVER] Bind successful." << std::endl;

    // Listen for incoming connections
    if (listen(serverSocket, 1) < 0) {
        std::cerr << "[SERVER] Listen failed" << std::endl;
        return -1;
    }
    std::cout << "[SERVER] Listening for clients on port " << SERVER_PORT << "..." << std::endl;

    // Accept incoming client connection
    clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientAddrLen);
    if (clientSocket < 0) {
        std::cerr << "[SERVER] Client connection failed" << std::endl;
        return -1;
    }
    std::cout << "[SERVER] New client connected!" << std::endl;

    // Handle client communication
    while (true) {
        std::cout << "[SERVER] Handling new client..." << std::endl;

        // Receive number of vertices
        int n;
        int bytesReceived = recv(clientSocket, &n, sizeof(n), 0);
        if (bytesReceived <= 0) {
            std::cerr << "[SERVER] Connection closed or error receiving data" << std::endl;
            break;
        }
        std::cout << "[SERVER] Received number of vertices: " << n << std::endl;

        // Create a graph and add edges
        Graph graph(n);
        for (int i = 0; i <= n; ++i) {
            Graph::Edge edge;
            bytesReceived = recv(clientSocket, &edge, sizeof(edge), 0);
            if (bytesReceived <= 0) {
                std::cerr << "[SERVER] Error receiving edge data" << std::endl;
                break;
            }
            graph.addEdge(edge.src, edge.dest, edge.weight);
            std::cout << "[SERVER] Received edge (" << edge.src << ", " << edge.dest << ") with weight " << edge.weight << std::endl;
        }

        // Receive algorithm choice
        int algorithmChoice;
        bytesReceived = recv(clientSocket, &algorithmChoice, sizeof(algorithmChoice), 0);
        if (bytesReceived <= 0) {
            std::cerr << "[SERVER] Error receiving algorithm choice" << std::endl;
            break;
        }
        std::cout << "[SERVER] Received algorithm choice: " << (algorithmChoice == 0 ? "Prim" : "Kruskal") << std::endl;

        // Print exact received algorithm choice (this is what you asked for)
        std::cout << "[SERVER] Exact received algorithm choice (int): " << algorithmChoice << std::endl;

        // Create MST algorithm based on choice
        MSTAlgorithm* algorithm = nullptr;
        if (algorithmChoice == 0) {
            algorithm = MSTAlgorithmFactory::createAlgorithm(MSTAlgorithmFactory::PRIM);
        } else if (algorithmChoice == 1) {
            algorithm = MSTAlgorithmFactory::createAlgorithm(MSTAlgorithmFactory::KRUSKAL);
        }

        if (!algorithm) {
            std::cerr << "[SERVER] Invalid algorithm choice" << std::endl;
            break;
        }

        // Create the MST tree
        Tree mst(n);

        // Compute the MST
        computeMST(graph, mst, algorithm);

        // Calculate MST weight (sum of edge weights)
        int mstWeight = 0;
        for (const auto& adjList : mst.adjList) {
            for (const auto& edge : adjList) {
                mstWeight += edge.weight;
            }
        }

        std::cout << "[SERVER] MST Total Weight: " << mstWeight << std::endl;

        // Send the MST weight back to the client
        send(clientSocket, &mstWeight, sizeof(mstWeight), 0);
        std::cout << "[SERVER] Sent MST weight to client." << std::endl;

        // Disconnect client
        std::cout << "[SERVER] Client disconnected." << std::endl;
        break;  // End communication with this client (remove if handling multiple clients)
    }

    // Close the sockets
    close(clientSocket);
    close(serverSocket);
    return 0;
}
