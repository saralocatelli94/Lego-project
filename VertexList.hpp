//
//  VertexList.hpp
//  Sokoban Solver
//
//  Created by Olliver Ordell on 24/09/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef VertexList_hpp
#define VertexList_hpp

#include <stdio.h>
#include "Graph.hpp"

struct VertexList {
    VertexList();
    ~VertexList();
    
    VertexList(Vertex *vTarget = nullptr, Vertex *vPrev = nullptr,
               double costTrav = 0, double costHeu = 0, char OOR = '0');
    
    Vertex *vertexTarget;
    Vertex *vertexPrevious;
    double costTravel;
    double costHeuristic;
    char orientationOfRobot;
    
    double cost_Total(){return costTravel+costHeuristic;};
};

#endif /* VertexList_hpp */
