//
//  SolverNode_v2.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 10/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "SolverNode_v2.hpp"

SolverNode_v2::SolverNode_v2(){
}

SolverNode_v2::~SolverNode_v2(){
}

SolverNode_v2::SolverNode_v2(int pos, std::vector<int> dPos, int id_current, int id_prev, int depth, int distTravl, int distHeuristic):
position(pos), diamondPositions(dPos), ID(id_current), prevID(id_prev), depthInTree(depth), distanceTraveled(distTravl), distanceHeuristic(distHeuristic){
    distanceTotal = distanceTraveled + distanceHeuristic;
}
