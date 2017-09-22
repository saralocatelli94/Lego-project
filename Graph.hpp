//
//  Graph.hpp
//  Graphs
//
//  Created by Olliver Ordell on 29/11/2016.
//  Copyright © 2016 Olliver Ordell. All rights reserved.
//

#ifndef Graph_hpp
#define Graph_hpp

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stack>
#include "Vertex.hpp"

class Graph{
public:
    Graph();
    ~Graph();
    
    void addVertex();
    void addVertex(std::string& name);
    void addVertex(std::string& name, bool& dimond, bool& goal, bool& sokoban);
    void addEdge(unsigned int source, unsigned int target);
    void addEdge(unsigned int source, unsigned int target, unsigned int weight);
    void addEdge(std::string source, std::string target, unsigned int weight);
    void addEdge(std::string source, std::string target, unsigned int weight, char direction);
    void removeEdge(unsigned int source, unsigned int target);
    
    bool vertexExist(std::string name);
    
    // Acces a vertex:
    Vertex &getVertex(unsigned int i);
    Vertex &getVertex(std::string name);
    
    // print graph
    void printGraph();
    
protected:
    std::vector<Vertex> adjList;
    unsigned int numOfVertex;
    
private:
    void topSort(Vertex & tempV, std::vector<Vertex*> & stack, std::vector<Vertex*> & visited);
};

#endif /* Graph_hpp */
