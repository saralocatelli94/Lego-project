//
//  Solver.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 31/10/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef Solver_hpp
#define Solver_hpp

#include <stdio.h>
#include <iostream>
#include "Graph.hpp"
#include "SolverNode.hpp"
#include "AStar.hpp"
#include "PathDrawer.hpp"
#include <vector>
#include <math.h>

class Solver{
public:
    Solver();
    ~Solver();
    
    Solver(Graph & mStart, Graph & mGoal);
    Solver(Graph & mStart, Graph & mGoal, int width, int height);
    
    void startSolver();
    
    std::vector<std::vector<SolverNode>> getSolution();
    
    unsigned long getNumOfNodes();
    
protected:
    Graph * mapStart, * mapGoal, * mapCurrent;
    std::vector<SolverNode> solutionList_Open;      // All nodes
    std::vector<std::vector<SolverNode>> solutionList_Closed;    // Nodes to solution
    
    unsigned long numOfNodes;
    unsigned long currentNode;
    
    int map_width;
    int map_height;
    
    int numOfSolutions;
    
private:
    // string representation of 'mapStart' and 'mapGoal'
    std::string startConfiguration;
    std::string goalConfiguration;
    // String of current configuration:
    std::string currentConfiguration;
    
    // compairs current config to goal config:
    bool compairCurrentConfigToGoalConfig();
    
    // Check if the new configuration is already in the openlist:
    bool configurationAllreadyChecked(std::string config);
    
    // Compair configurations:
    bool compairConfigs(std::string current, std::string next);
    
    // Find the direction of movment:
    char findDirection(int robotIndex, int diamondIndex);
    
    // Check for deadlock on mapCurrent:
    bool deadlockCheck();
    
    // Check for deadlock on specific diamond in mapCurrent
    bool deadlockCheck(std::string vertexName);
    
    // configurations when calling a constructor.
    void constructorSetup();
};

#endif /* Solver_hpp */
