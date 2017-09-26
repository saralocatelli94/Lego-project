//
//  VertexList.cpp
//  Sokoban Solver
//
//  Created by Olliver Ordell on 24/09/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "VertexList.hpp"

VertexList::VertexList(){
    
}

VertexList::VertexList(Vertex *vTarget, Vertex *vPrev, double costTrav, double costHeu, char OOR):
vertexTarget(vTarget), vertexPrevious(vPrev), costTravel(costTrav), costHeuristic(costHeu), orientationOfRobot(OOR){
    
}

VertexList::~VertexList(){
    
}
