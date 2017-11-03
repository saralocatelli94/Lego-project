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
    
    int position_BeforeDiamondMove;
    int position_AfterDiamondMove;
    
    std::string graphString_BeforeDiamondMove;
    std::string graphString_AfterDiamondMove;
    
    long ID;
    long prevID;
    
};

#endif /* SolverNode_hpp */
