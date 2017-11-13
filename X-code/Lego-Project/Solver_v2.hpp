//
//  Solver_v2.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 10/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef Solver_v2_hpp
#define Solver_v2_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include "SolverNode_v2.hpp"
#include "Graph.hpp"
#include "AStar.hpp"
#include "PathDrawer.hpp"

class Solver_v2 {
public:
    Solver_v2();
    ~Solver_v2();
    
    Solver_v2(Graph & mStart, Graph & mGoal, int numOfDia, int width, int height);
    
    void startSolver();
    
    std::vector<std::vector<SolverNode_v2>> getSolution();
    
    unsigned int getNumOfNodes();
    
protected:
    Graph * mapStart, * mapGoal, * mapCurrent;
    std::vector<SolverNode_v2> solutionList_Open;               // All nodes
    std::vector<std::vector<SolverNode_v2>> solutionList_Closed;// All solutions
    
    unsigned int numOfNodes;
    unsigned int currentNode;
    unsigned int numOfSolutions;
    
    int map_width;
    int map_height;
    
private:
    int currentRobotPosition;
    int numOfDiamonds;
    std::vector<int> diamondPositionCurrent;
    
    // Find the direction of movment:
    char findDirection(int robotIndex, int diamondIndex);
    
    // Deadlock check:
    bool deadlockCheck(int vertex);
    
    // For Hash-tabel use:
    int static const tableSize = 1009;      // Prime number
    std::vector<std::vector<int>> hashTable;
    std::vector<int> lookUp_diamondValue;
    
    // Functions used for hashing:
    std::string creatHashKey();
    std::string creatHashKey(std::vector<int> diamondPos);
    int hash(std::string key);
    void insertHash(int hashVal, std::vector<int> diamondPos);
    bool lookUpHash_prevAdded(int hashVal);
    bool lookUpHash_prevAdded(int hashVal, std::vector<int> diamondPos);
};

#endif /* Solver_v2_hpp */
