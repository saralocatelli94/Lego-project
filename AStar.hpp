//
//  AStar.hpp
//  Sokoban Solver
//
//  Created by Olliver Ordell on 24/09/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef AStar_hpp
#define AStar_hpp

#include <stdio.h>
#include <cmath>
#include <vector>
#include <iomanip>
#include "Graph.hpp"
#include "VertexList.hpp"

class AStar {
    
public:
    AStar();
    
    AStar(Graph roadMap, Vertex vertexStart, Vertex vertexGoal, char directionGoal);
    
    void runAStar();
    std::vector<VertexList> getPath();
    
    void printVertexToGoal();
    
    ~AStar();
    
protected:
    Graph map;
    Vertex vStart, vGoal;
    int currentVertexIndex;
    char directionGoal;
    
    std::vector<VertexList> vertexOpenList;
    std::vector<VertexList> vertexClosedList;
    std::vector<VertexList> vertexToGoal;
    
private:
    bool validStartAndGoal();
    double heuristicDistance(Vertex current, Vertex goal);
    double calcWeight(VertexList closedListVertex, Edge newDirection);
    char calcOrientation(char currentOrientation, char edgeDirection);
    double const turningMultiplyer = 1.75;
};

#endif /* AStar_hpp */
