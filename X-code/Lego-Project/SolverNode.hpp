//
//  SolverNode.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 01/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef SolverNode_hpp
#define SolverNode_hpp

#include <stdio.h>
#include <iostream>

struct SolverNode{

    SolverNode();
    ~SolverNode();
    
    SolverNode(std::string beforeMove, std::string afterMove);
    SolverNode(std::string beforeMove, std::string afterMove,
               long id, long p_id);
    SolverNode(std::string beforeMove, std::string afterMove,
               long id, long p_id, long dist, int depth);
    
    // Robot position:
    int position_BeforeDiamondMove;
    int position_AfterDiamondMove;
    
    std::string graphString_BeforeDiamondMove;
    std::string graphString_AfterDiamondMove;
    
    int ID;
    int prevID;
    
    // Distance the robot has traveled to this move:
    int distanceTraveled;
    
    // Depth in the tree:
    int depthInTree;
    
};

#endif /* SolverNode_hpp */
