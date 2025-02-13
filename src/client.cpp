#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fstream>
#include <string>
#include <cstdlib>
#include "../include/graph.h"

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8080

struct ClientResponse {
    int totalWeight;
    int longestDistance;
    double averageDistance;
    int shortestMSTEdge;
    int numMSTEdges;
    Graph::Edge mstEdges[100];
};

class GraphVisualizer {
public:
    static void visualizeGraph(const Graph& graph, const std::string& filename) {
        std::ofstream out(filename + ".dot");
        out << "digraph G {\n";
        out << "    node [shape=circle, style=filled, fillcolor=lightblue];\n";
        out << "    edge [color=black];\n";

        for (int i = 0; i < graph.V; i++) {
            for (const auto& edge : graph.adjList[i]) {
                out << "    " << edge.src << " -> " << edge.dest
                    << " [label=\"" << edge.weight << "\"];\n";
            }
        }
        out << "}\n";
        out.close();

        std::string cmd = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
        if (system(cmd.c_str()) == 0) {
            std::cout << "\nGraph visualization saved to " << filename << ".png" << std::endl;
        } else {
            std::cerr << "Error: Failed to generate PNG. Is graphviz installed?" << std::endl;
        }
    }

    static void visualizeMST(const ClientResponse& response, const std::string& filename) {
        std::ofstream out(filename + ".dot");
        out << "digraph MST {\n";
        out << "    node [shape=circle, style=filled, fillcolor=lightgreen];\n";
        out << "    edge [color=blue, penwidth=2];\n";

        for (int i = 0; i < response.numMSTEdges; i++) {
            const auto& edge = response.mstEdges[i];
            out << "    " << edge.src << " -> " << edge.dest
                << " [label=\"" << edge.weight << "\"];\n";
        }

        out << "}\n";
        out.close();

        std::string cmd = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
        if (system(cmd.c_str()) == 0) {
            std::cout << "\nMST visualization saved to " << filename << ".png" << std::endl;
        } else {
            std::cerr << "Error: Failed to generate PNG. Is graphviz installed?" << std::endl;
        }
    }
};

void sendGraph(int sock, const Graph& graph) {
    int V = graph.V;
    send(sock, &V, sizeof(V), 0);
    std::cout << "[CLIENT] Sent number of vertices: " << V << std::endl;

    for (int i = 0; i < V; ++i) {
        for (const auto& edge : graph.adjList[i]) {
            send(sock, &edge, sizeof(Graph::Edge), 0);
            std::cout << "[CLIENT] Sent edge (" << edge.src << ", " << edge.dest
                     << ") with weight " << edge.weight << std::endl;
        }
    }
}

Graph createGraphInteractively() {
    int V;
    std::cout << "\nEnter number of vertices: ";
    std::cin >> V;

    Graph graph(V);

    int numEdges;
    std::cout << "Enter number of edges: ";
    std::cin >> numEdges;

    std::cout << "\nEnter edges in format: source destination weight" << std::endl;
    for (int i = 0; i < numEdges; i++) {
        int src, dest, weight;
        std::cout << "Edge " << i+1 << ": ";
        std::cin >> src >> dest >> weight;
        if (src >= 0 && src < V && dest >= 0 && dest < V) {
            graph.addEdge(src, dest, weight);
        } else {
            std::cout << "Invalid vertex numbers. Must be between 0 and " << V-1 << std::endl;
            i--; // Retry this edge
        }
    }

    return graph;
}

int main() {
    if (system("dot -V > /dev/null 2>&1") != 0) {
        std::cerr << "Warning: graphviz is not installed. Visualizations will not be generated.\n"
                  << "Please install graphviz using:\n"
                  << "    sudo apt-get install graphviz\n\n";
    }

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

    // Create graph interactively
    std::cout << "\n=== Graph Creation ===" << std::endl;
    Graph graph = createGraphInteractively();

    // Visualize original graph
    std::cout << "\nGenerating graph visualization..." << std::endl;
    GraphVisualizer::visualizeGraph(graph, "original_graph");

    // Send graph to server
    sendGraph(sock, graph);

    // Choose algorithm
    std::cout << "\n=== Algorithm Selection ===" << std::endl;
    std::cout << "Available algorithms:" << std::endl;
    std::cout << "0: Prim's Algorithm" << std::endl;
    std::cout << "1: Kruskal's Algorithm" << std::endl;

    int algorithmChoice;
    do {
        std::cout << "Choose algorithm (0 or 1): ";
        std::cin >> algorithmChoice;
    } while (algorithmChoice != 0 && algorithmChoice != 1);

    send(sock, &algorithmChoice, sizeof(algorithmChoice), 0);
    std::cout << "[CLIENT] Sent algorithm choice: "
              << (algorithmChoice == 0 ? "Prim" : "Kruskal") << std::endl;

    // Receive and display MST metrics
    ClientResponse response;
    recv(sock, &response, sizeof(response), 0);

    std::cout << "\n=== MST Analysis Results ===" << std::endl;
    std::cout << "Total MST Weight: " << response.totalWeight << std::endl;
    std::cout << "Longest Distance between vertices: " << response.longestDistance << std::endl;
    std::cout << "Average Distance between vertices: " << response.averageDistance << std::endl;
    std::cout << "Shortest MST Edge: " << response.shortestMSTEdge << std::endl;

    std::cout << "\nMST Edges:" << std::endl;
    for (int i = 0; i < response.numMSTEdges; i++) {
        std::cout << response.mstEdges[i].src << " -> "
                 << response.mstEdges[i].dest << " (weight: "
                 << response.mstEdges[i].weight << ")" << std::endl;
    }

    // Visualize MST
    std::cout << "\nGenerating MST visualization..." << std::endl;
    GraphVisualizer::visualizeMST(response, "mst_graph");

    close(sock);
    std::cout << "\n[CLIENT] Disconnected from server." << std::endl;

    return 0;
}