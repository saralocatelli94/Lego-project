//
//  SolverNode_v2.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 10/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef SolverNode_v2_hpp
#define SolverNode_v2_hpp

#include <stdio.h>
#include <iostream>
#include <vector>

struct SolverNode_v2{
    
    SolverNode_v2();
    ~SolverNode_v2();
    
    SolverNode_v2(int posB, int posA, std::vector<int> dPos, int id_current, int id_prev, int depth, int distTravl, int distHeuristic = 0);
    
    // Robot position:
    int positionBefore;         // Before moving diamond
    int positionAfter;          // After moving diamond
    
    // Diamond positions:
    std::vector<int> diamondPositions;
    
    // ID's
    int ID;
    int prevID;
    
    // Distance the robot has traveled to this move:
    int distanceTraveled;
    int distanceHeuristic;
    int distanceTotal;
    
    // Depth in the tree:
    int depthInTree;
};

#endif /* SolverNode_v2_hpp */
