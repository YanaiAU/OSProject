#ifndef GRAPH_VISUALIZER_H
#define GRAPH_VISUALIZER_H

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include "graph.h"

class GraphVisualizer {
public:
    static void visualizeGraph(const Graph& graph, const std::string& filename) {
        // Create DOT file for original graph
        std::ofstream out(filename + ".dot");
        out << "digraph G {\n";
        
        // Add styling
        out << "    node [shape=circle, style=filled, fillcolor=lightblue];\n";
        out << "    edge [color=black];\n";
        
        // Add edges
        for (int i = 0; i < graph.V; i++) {
            for (const auto& edge : graph.adjList[i]) {
                out << "    " << edge.src << " -> " << edge.dest 
                    << " [label=\"" << edge.weight << "\"];\n";
            }
        }
        out << "}\n";
        out.close();

        // Convert to PNG using graphviz
        std::string cmd = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
        system(cmd.c_str());
        
        // Open the image (works on Unix-like systems)
        system(("xdg-open " + filename + ".png").c_str());
    }
    
    static void visualizeMST(const std::vector<std::tuple<int, int, int, int>>& mstEdges, 
                            int numVertices, const std::string& filename) {
        // Create DOT file for MST
        std::ofstream out(filename + ".dot");
        out << "digraph MST {\n";
        
        // Add styling
        out << "    node [shape=circle, style=filled, fillcolor=lightgreen];\n";
        out << "    edge [color=blue, penwidth=2];\n";
        
        // Add edges
        for (const auto& edge : mstEdges) {
            int src = std::get<0>(edge);
            int dest = std::get<1>(edge);
            int weight = std::get<2>(edge);
            out << "    " << src << " -> " << dest 
                << " [label=\"" << weight << "\"];\n";
        }
        out << "}\n";
        out.close();

        // Convert to PNG using graphviz
        std::string cmd = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
        system(cmd.c_str());
        
        // Open the image
        system(("xdg-open " + filename + ".png").c_str());
    }
};

#endif // GRAPH_VISUALIZER_H