//
//  SolverNode.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 01/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "SolverNode.hpp"

SolverNode::SolverNode(){
}

SolverNode::~SolverNode(){
}

SolverNode::SolverNode(std::string beforeMove, std::string afterMove):
graphString_BeforeDiamondMove(beforeMove), graphString_AfterDiamondMove(afterMove){
    for (int i = 0 ; i < beforeMove.length() ; i++) {
        if (beforeMove.at(i) == 'M' || beforeMove.at(i) == 'N') {
            position_BeforeDiamondMove = i;
            break;
        }
    }
    for (int i = 0 ; i < afterMove.length() ; i++) {
        if (afterMove.at(i) == 'M' || afterMove.at(i) == 'N') {
            position_AfterDiamondMove = i;
            break;
        }
    }
    ID = -1;
    prevID = -1;
}

SolverNode::SolverNode(std::string beforeMove, std::string afterMove, long id, long p_id):
graphString_BeforeDiamondMove(beforeMove), graphString_AfterDiamondMove(afterMove), ID(id), prevID(p_id){
    for (int i = 0 ; i < beforeMove.length() ; i++) {
        if (beforeMove.at(i) == 'M' || beforeMove.at(i) == 'N') {
            position_BeforeDiamondMove = i;
            break;
        }
    }
    for (int i = 0 ; i < afterMove.length() ; i++) {
        if (afterMove.at(i) == 'M' || afterMove.at(i) == 'M') {
            position_AfterDiamondMove = i;
            break;
        }
    }
}

