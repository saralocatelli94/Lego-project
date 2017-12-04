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
#include "HashTableStruct.hpp"
#include "Quicksort.hpp"
#include <algorithm>

class Solver_v2 {
public:
    Solver_v2();
    ~Solver_v2();
    
    Solver_v2(Graph & mStart, Graph & mGoal, int numOfDia, int width, int height);
    
    void startSolver(bool allowRobotToReverse = true);
    void startSolver_2(bool allowRobotToReverse = true);
    
    std::vector<std::vector<SolverNode_v2>> getSolution();
    
    unsigned int getNumOfNodes();
    
protected:
    Graph * mapStart, * mapGoal, * mapCurrent;
    std::vector<SolverNode_v2> solutionList_Open;               // All nodes
    std::vector<SolverNode_v2> solutionList_AllExplored;
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
    std::vector<int> diamondPositionGoal;
    std::vector<int> deadlockZones;
    
    // Find the direction of movment:
    char findDirection(int robotIndex, int diamondIndex);
    
    // Deadlock check:
    bool deadlockCheck(int vertex);
    void findDeadlockZones();
    bool deadlockCornerCheck(int vertex);
    
    // Calculate heutistic distance to goal:
    int calculateHeuristicDist(std::vector<int> diamondPos);
    
    // Straight line distance between vertex's
    double pythagoras(int a, int b);
    
    // Does a vector contain a number?
    bool contain(std::vector<int> vec, int num);
    
    
    // For Hash-tabel use:
    int static const tableSize = 100003;      // Prime number (10009)
    std::vector<std::vector<HashTableStruct>> hashTable;
    std::vector<int> lookUp_diamondValue;
    
    // Functions used for hashing:
    std::string creatHashKey();
    std::string creatHashKey(std::vector<int> diamondPos);
    int hash(std::string key);
    //void insertHash(int hashVal, std::vector<int> diamondPos);
    bool lookUpHash_prevAdded(int hashVal);
    //bool lookUpHash_prevAdded(int hashVal, std::vector<int> diamondPos);
    
    // Functions for hash, with robot position:
    std::string creatHashKey_wRobot();
    std::string creatHashKey_wRobot(std::vector<int> diamondPos, int robPos);
    void insertHash_wRobot(int hashVal, std::vector<int> diamondPos, int robPos, int dist);
    bool lookUpHash_prevAdded_wRobot(int hashVal, std::vector<int> diamondPos, int robPos, int dist);
    
};

#endif /* Solver_v2_hpp */
