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
#include <map>
#include "Vertex.hpp"

class Graph{
public:
    Graph();
    ~Graph();
    
    Graph(int noDiamonds);
    
    void addVertex();
    void addVertex(std::string& name);
    void addVertex(std::string& name, bool& dimond, bool& goal, bool& sokoban);
    void addVertex(std::string& name, bool& dimond, bool& goal, bool& sokoban, char sokobanDirection);
    void addEdge(unsigned int source, unsigned int target);
    void addEdge(unsigned int source, unsigned int target, unsigned int weight);
    void addEdge(std::string source, std::string target, unsigned int weight);
    void addEdge(std::string source, std::string target, unsigned int weight, char direction);
    void removeEdge(unsigned int source, unsigned int target);
    int** printAdjMatrix();
    std::pair<int,std::vector<char>> dijkstra(int**matrix,int src,bool flag);
    int findMinDist(int dist[]);
    void updateMap(int dest,int src);
    std::vector<int> getDiamondsID();
    int getSource();
    bool vertexExist(std::string name);
    
    // Acces a vertex:
    Vertex &getVertex(unsigned int i);
    Vertex &getVertex(std::string name);

    unsigned int getNumOfVertex();
    unsigned int getNumOfDiamonds();
    unsigned int getRobotPosition();
    
    // print graph
    void printGraph();
    std::map<int,Vertex> vertexMap; // map for the index of a vertex and the vertex
    
    // Operator overload:
    bool operator==(Graph rhw);
    std::string getGraphRepresentation();
    std::vector<int> getDiamondPosition();
    
    // Set information on all vertex's based on string input
    void setAllVertexInfo(std::string graphRepresentation);
    void setAllVertexInfo(std::vector<int> diamondPos, int robotPos);

protected:
    std::vector<Vertex> adjList;
    unsigned int numOfVertex;
    unsigned int numOfDiamonds;
    unsigned int RobotPosition;
    std::vector<int> diamondPosition;
    
    // String representing the Graph. Makes it easier to compare graphs
    std::string graphRepresentation;
    
private:
    void topSort(Vertex & tempV, std::vector<Vertex*> & stack, std::vector<Vertex*> & visited);
    void printSolution(int dist[]);

    int minDistance(int dist[], bool sptSet[]);
    
    void createGraphRepresentation();
    void updateDiamondPosition();
    void updateRobotPosition();
};

#endif /* Graph_hpp */
