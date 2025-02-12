#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "graph.h"

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8080

void sendGraph(int sock, const Graph& graph) {
    int V = graph.V;
    send(sock, &V, sizeof(V), 0);
    std::cout << "[CLIENT] Sent number of vertices: " << V << std::endl;

    for (int i = 0; i < V; ++i) {
        for (const auto& edge : graph.adjList[i]) {
            send(sock, &edge, sizeof(Graph::Edge), 0);
            std::cout << "[CLIENT] Sent edge (" << edge.src << ", " << edge.dest << ") with weight " << edge.weight << std::endl;
        }
    }
}

int main() {
    std::cout << "[CLIENT] Starting client..." << std::endl;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "[CLIENT] Socket creation failed\n";
        return 1;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr);

    std::cout << "[CLIENT] Connecting to server..." << std::endl;
    if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "[CLIENT] Connection failed\n";
        return 1;
    }
    std::cout << "[CLIENT] Connected to server!" << std::endl;

    // Create a sample graph
    Graph graph(4);
    graph.addEdge(0, 1, 10);
    graph.addEdge(0, 2, 6);
    graph.addEdge(0, 3, 5);
    graph.addEdge(1, 3, 15);
    graph.addEdge(2, 3, 4);

    // Send graph to server
    sendGraph(sock, graph);

	sleep(2);

    // Choose algorithm: 0 = Prim, 1 = Kruskal
    int algorithmChoice = 0;
    send(sock, &algorithmChoice, sizeof(algorithmChoice), 0);
    std::cout << "[CLIENT] Sent algorithm choice: " << (algorithmChoice == 0 ? "Prim" : "Kruskal") << std::endl;

    // Receive MST total weight from the server
    int mstWeight;
    recv(sock, &mstWeight, sizeof(mstWeight), 0);
    std::cout << "[CLIENT] Received MST Total Weight: " << mstWeight << std::endl;

    close(sock);
    std::cout << "[CLIENT] Disconnected from server.\n";

    return 0;
}
