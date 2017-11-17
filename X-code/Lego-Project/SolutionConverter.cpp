//
//  SolutionConverter.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 15/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "SolutionConverter.hpp"

SolutionConverter::SolutionConverter(){
}

SolutionConverter::~SolutionConverter(){
}

SolutionConverter::SolutionConverter(Graph & map, std::vector<std::vector<SolverNode_v2>> S, char sDir, std::string fName):
mapSolution(&map), solution(S), startDirection(sDir), fileName(fName){
}

void SolutionConverter::startConverter(bool allowRobotToReverse){
    
    convertion.clear();
    
    // For all solutions:
    for (int i = 0 ; i < solution.size() ; i++) {
        
        char directionInit = startDirection;
        std::vector<int> tempConvertion;
        tempConvertion.clear();
        
        // For solution i: (start with j = 1 for easier accessing the solution)
        for (int j = 1 ; j < solution[i].size() ; j++) {
            
            // Set the map to the previous configuration:
            mapSolution->setAllVertexInfo(solution[i][j-1].diamondPositions, solution[i][j-1].positionAfter);
            
            // Run A Star on the path from prev. vertex to new vertex:
            char directionPost = calculateDirection(solution[i][j].positionBefore, solution[i][j].positionAfter);
            AStar path(*mapSolution, mapSolution->getVertex(solution[i][j-1].positionAfter), mapSolution->getVertex(solution[i][j].positionBefore), directionInit, directionPost);
            path.runAStar(allowRobotToReverse);
            std::vector<VertexList> vector_path = path.getPath();
            
            // Itterate through the path to generate the convertion:
            // The path is flipped, so itterate down:
            for (int k = (int) vector_path.size() - 2 ; k >= 0 ; k--) {
                
                char directionNext = vector_path[k].orientationOfRobot;
                
                // Add direction change to the list:
                if (directionInit != directionNext) {
                    tempConvertion.push_back(calculateDiretionChange(directionInit, directionNext));
                }
                
                tempConvertion.push_back(1);
                
                directionInit = directionNext;
            }
            if (directionInit != directionPost) {
                tempConvertion.push_back(calculateDiretionChange(directionInit, directionPost));
            }
            
            // Move the diamond one space:
            tempConvertion.push_back(1);
            
            directionInit = directionPost;
        }
        
        convertion.push_back(tempConvertion);
    }
    // Write to .txt-file
    writeConvertionToFile();
}

char SolutionConverter::calculateDirection(int initIndex, int postIndex){
    char c = '0';
    for (int i = 0 ; i < mapSolution->getVertex(initIndex).connections.size() ; i++) {
        if (mapSolution->getVertex(initIndex).connections[i].getTarget()->getIndex() == postIndex) {
            c = mapSolution->getVertex(initIndex).connections[i].getDirection();
            break;
        }
    }
    return c;
}

int SolutionConverter::calculateDiretionChange(char dirInit, char dirPost){
    int change;
    int left = 2, right = 3,uTurn = 4;
    
    /**
     1 = go straight;
     2 = turn left
     3 = turn right
     4 = turn 180 deg.
     */
    
    switch (dirInit) {
        case 'n':
            switch (dirPost) {
                case 'e':
                    change = right;
                    break;
                    
                case 's':
                    change = uTurn;
                    break;
                    
                case 'w':
                    change = left;
                    break;
                    
                default:
                    break;
            }
            break;
            
        case 'e':
            switch (dirPost) {
                case 'n':
                    change = left;
                    break;
                    
                case 's':
                    change = right;
                    break;
                    
                case 'w':
                    change = uTurn;
                    break;
                    
                default:
                    break;
            }
            break;
            
        case 's':
            switch (dirPost) {
                case 'n':
                    change = uTurn;
                    break;
                    
                case 'e':
                    change = left;
                    break;
                    
                case 'w':
                    change = right;
                    break;
                    
                default:
                    break;
            }
            break;
            
        case 'w':
            switch (dirPost) {
                case 'n':
                    change = right;
                    break;
                    
                case 's':
                    change = left;
                    break;
                    
                case 'e':
                    change = uTurn;
                    break;
                    
                default:
                    break;
            }
            break;
            
        default:
            break;
    }
    
    return change;
}

void SolutionConverter::writeConvertionToFile(){
    std::ofstream output;
    output.open (fileName + ".txt");
    
    for (int i = 0 ; i < convertion.size() ; i++) {
        for (int j = 0 ; j < convertion[i].size() ; j++) {
            output << convertion[i][j] << ",";
        }
        output << "\n";
    }
    output.close();
}




