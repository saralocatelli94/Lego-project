//
//  SolutionConverter.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 15/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef SolutionConverter_hpp
#define SolutionConverter_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include "AStar.hpp"
#include "Solver_v2.hpp"
#include <fstream>

class SolutionConverter {
public:
    SolutionConverter();
    ~SolutionConverter();
    
    SolutionConverter(Graph & map, std::vector<std::vector<SolverNode_v2>> S, char sDir, std::string fName = "solution");
    
    void startConverter(bool allowRobotToReverse = true);
    
protected:
    std::vector<std::vector<SolverNode_v2>> solution;
    Graph * mapSolution;
    char startDirection;
    std::string fileName;
    
    std::vector<std::vector<int>> convertion;
    
private:
    // Calculates the direction of going from one vertex to another
    char calculateDirection(int initIndex, int postIndex);
    
    // Calculate direction change:
    int calculateDiretionChange(char dirInit, char dirPost);
    
    // Write convertion to a .txt-file:
    void writeConvertionToFile();
    
    // Turning constants:
    static const int left = 2;
    static const int right = 3;
    static const int uTurn = 4;
};

#endif /* SolutionConverter_hpp */
