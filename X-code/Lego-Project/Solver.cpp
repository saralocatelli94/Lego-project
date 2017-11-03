//
//  Solver.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 31/10/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "Solver.hpp"

Solver::Solver(){
}

Solver::~Solver(){
}

Solver::Solver(Graph mS, Graph mG):
mapStart(mS), mapGoal(mG), mapCurrent(mS){
    constructorSetup();
}

void Solver::startSolver(){
    
    /**
     Steps for solving:
     
     1) Find vertex's that does NOT contain diamonds, and has a neighbour that does contain a diamond.
     2) From thees find the ones that has a freespace on the other side of the diamond from the robots view, and see if it is a 'free-space'.
     3) From thees find the ones that the robot can actually move to, from it's current position
     4) Move the diamond one (1) space (vertex) in the direction.
     5) Check if the new configuration already is in the openlist. If so, discard it
     6) Update variables
     7) When solutions is found (end of while-loop) backtrack the openlist, to find the solution.
     */
    
    /* A list of all diamonds's index in the graph:
    std::vector<int> diamonds;
    for (int i = 0 ; i < mapCurrent.getNumOfVertex() ; i++) {
        if (mapCurrent.getVertex(i).getDiamond()) {
            diamonds.push_back(mapCurrent.getVertex(i).getIndex());
        }
    }*/
    // A list of all vertex's in the graph that does NOT have i diamond, and has an edge to a vertex containing a diamond.
    std::vector<std::vector<int>> vertex_NextToDiamonds;
    // A list of the dimonds that ro robot can actually move:
    std::vector<std::vector<int>> vertex_Movable;
    // A list of the dimonds the robot can move, AND get to (by using the A*):
    std::vector<std::vector<int>> vertex_MovableAndReachable;
    
    int ID_current = -1;
    
    // Start the solver:
    while (!compairCurrentConfigToGoalConfig()) {
        
        vertex_NextToDiamonds.clear();
        vertex_Movable.clear();
        vertex_MovableAndReachable.clear();
        
        std::cout << "Num of nodes:    " << numOfNodes << std::endl;
        std::cout << "Current config0: " << currentConfiguration << std::endl;
        /*
        for (int i = 0 ; i < diamonds.size() ; i++) {
            for (int j = 0 ; j < mapCurrent.getVertex(diamonds[i]).connections.size() ; j++) {
                if (!mapCurrent.getVertex(diamonds[i]).connections[j].getTarget()->getDiamond()) {
                    std::vector<int> temp;
                    temp.push_back(j);  // Free space
                    temp.push_back(i);  // Diamond
                    vertex_NextToDiamonds.push_back(temp);
                }
            }
        }
        */
        // Find vertex's with no diamonds, and that has vertex's with diamonds as a neighbour.
        for (int i = 0 ; i < mapCurrent.getNumOfVertex() ; i++) {
            for (int j = 0 ; j < mapCurrent.getVertex(i).connections.size() ; j++) {
                if (mapCurrent.getVertex(i).connections[j].getTarget()->getDiamond() &&
                    !mapCurrent.getVertex(i).getDiamond()) {
                    std::vector<int> temp;
                    temp.push_back(i);  // Free space
                    temp.push_back(mapCurrent.getVertex(i).connections[j].getTarget()->getIndex());  // Diamond
                    vertex_NextToDiamonds.push_back(temp);
                }
            }
        }
        /*
        for (int i = 0 ; i < vertex_NextToDiamonds.size() ; i++) {
            std::cout << "Freespace: " << vertex_NextToDiamonds[i][0] << std::endl;
            std::cout << "Diamond:   " << vertex_NextToDiamonds[i][1] << std::endl;
        }
        */
        // From the found vertex's above, find the ones that has a free-space on the other side of the diamond seen from the robot:
        for (int i = 0 ; i < vertex_NextToDiamonds.size() ; i++) {
            int robotIndex = vertex_NextToDiamonds[i][0];
            int diamondIndex = vertex_NextToDiamonds[i][1];
            
            char direction = findDirection(robotIndex, diamondIndex);
            std::string nextVertexName;
            
            if (direction == 'w') {
                nextVertexName = std::to_string(mapCurrent.getVertex(robotIndex).getYPosition()) +
                std::to_string(mapCurrent.getVertex(diamondIndex).getXPosition() - 1);
            }
            else if (direction == 'e') {
                nextVertexName = std::to_string(mapCurrent.getVertex(robotIndex).getYPosition()) +
                std::to_string(mapCurrent.getVertex(diamondIndex).getXPosition() + 1);

            }
            else if (direction == 'n') {
                nextVertexName = std::to_string(mapCurrent.getVertex(diamondIndex).getYPosition() - 1) +
                std::to_string(mapCurrent.getVertex(robotIndex).getXPosition());

            }
            else if (direction == 's') {
                nextVertexName = std::to_string(mapCurrent.getVertex(diamondIndex).getYPosition() + 1) +
                std::to_string(mapCurrent.getVertex(robotIndex).getXPosition());
            }
            
            // If vertex exist in graph and does not contain a diamond, the movement of the diamond is valid:
            if (mapCurrent.vertexExist(nextVertexName)) {
                if (!mapCurrent.getVertex(nextVertexName).getDiamond() &&
                    !mapCurrent.getVertex(nextVertexName).getSokoban()) {
                    vertex_Movable.push_back(vertex_NextToDiamonds[i]);
                }
            }
        }
        
        // Find the position of the sokoban:
        int sokobanIndexPosition = -1;
        for (int i = 0 ; i < mapCurrent.getNumOfVertex() ; i++) {
            if (mapCurrent.getVertex(i).getSokoban()){
                sokobanIndexPosition = i;
            }
        }
        
        // See if there is a valid path to the free-space vertex, from the robots current position, using A*:
        for (int i = 0 ; i < vertex_Movable.size() ; i++) {
            // Get end-direction:
            int robotIndex = vertex_Movable[i][0];
            int diamondIndex = vertex_Movable[i][1];
            char direction = findDirection(robotIndex, diamondIndex);
            // If start- and end-vertex are the same:
            if (mapCurrent.getVertex(sokobanIndexPosition).getIndex() == mapCurrent.getVertex(vertex_Movable[i][0]).getIndex()) {
                vertex_MovableAndReachable.push_back(vertex_Movable[i]);
            }
            else {
                AStar temp(mapCurrent,
                           mapCurrent.getVertex(sokobanIndexPosition),
                           mapCurrent.getVertex(vertex_Movable[i][0]),
                           direction);
                if (temp.runAStar()) {
                    vertex_MovableAndReachable.push_back(vertex_Movable[i]);
                }
            }
        }
        /*
        for (int i = 0 ; i < vertex_MovableAndReachable.size() ; i++) {
            std::cout << "Freespace: " << vertex_MovableAndReachable[i][0] << std::endl;
            std::cout << "Diamond:   " << vertex_MovableAndReachable[i][1] << std::endl;
        }
        */
        // Save the current map configs:
        std::string configBeforeMoving = mapCurrent.getGraphRepresentation();
        std::string configAfterMoving;
        
        // Move the diamond one space in the direction:
        for (int i = 0 ; i < vertex_MovableAndReachable.size() ; i++) {
            // Get the direction of movement:
            int robotIndex = vertex_MovableAndReachable[i][0];
            int diamondIndex = vertex_MovableAndReachable[i][1];
            char direction = findDirection(robotIndex, diamondIndex);
            
            // Move the robot to new position, and erase the diamond from same position:
            mapCurrent.getVertex(sokobanIndexPosition).setSokoban(false);
            mapCurrent.getVertex(vertex_MovableAndReachable[i][1]).setSokoban(true);
            mapCurrent.getVertex(vertex_MovableAndReachable[i][1]).setDiamond(false);
            
            // Find name of new vertex:
            std::string newVertexName;
            int xPos = mapCurrent.getVertex(vertex_MovableAndReachable[i][1]).getXPosition();
            int yPos = mapCurrent.getVertex(vertex_MovableAndReachable[i][1]).getYPosition();
            if (direction == 'w') {
                newVertexName = std::to_string(yPos) + std::to_string(xPos - 1);
            }
            else if (direction == 'e') {
                newVertexName = std::to_string(yPos) + std::to_string(xPos + 1);
            }
            else if (direction == 'n') {
                newVertexName = std::to_string(yPos - 1) + std::to_string(xPos);
            }
            else if (direction == 's') {
                newVertexName = std::to_string(yPos + 1) + std::to_string(xPos);
            }
            
            // Config the new vertex:
            mapCurrent.getVertex(newVertexName).setDiamond(true);
            
            // Save the new map configuration:
            configAfterMoving = mapCurrent.getGraphRepresentation();
            
            // See if the new config is allready in the openlist:
            if (!configurationAllreadyChecked(configAfterMoving)) {
                // Put the new map in the openlist:
                solutionList_Open.push_back(SolverNode(configBeforeMoving, configAfterMoving, numOfNodes, ID_current));
                numOfNodes++;
            }
            else {
                std::cout << "Already been checked\n";
            }
            
            // check to see if we have found the goal config:
            if (compairConfigs(configAfterMoving, goalConfiguration)) {
                currentConfiguration = configAfterMoving;
                break;
            }
        }
        
        // See if we have found the goal:
        if (!compairConfigs(currentConfiguration, goalConfiguration)) {
            // take the next config from openlist:
            currentConfiguration = solutionList_Open.at(currentNode).graphString_AfterDiamondMove;
            std::cout << "Current config1: " << currentConfiguration << std::endl;
        }
        // TODO:: Jeg ændre ikke index. Husk map config ordenligt.
        mapCurrent.setAllVertexInfo(currentConfiguration);
        ID_current++;
        currentNode++;
        /*
        for (int i = 0 ; i < solutionList_Open.size() ; i++) {
            std::string name = "node" + std::to_string(i) + ".ppm";
            Graph temp = mapCurrent;
            temp.setAllVertexInfo(solutionList_Open[i].graphString_BeforeDiamondMove);
            AStar aStarTest(temp, temp.getVertex(solutionList_Open[i].position_BeforeDiamondMove), temp.getVertex(solutionList_Open[i].position_AfterDiamondMove), 'n');
            
            std::vector<VertexList> path = aStarTest.getPath();
            
            PathDrawer Test(10, 9, temp, path);
            Test.drawPathAndSave(name);
            
            
        }
        */
    }
    
    // Backtrack to find the solution:
    std::vector<SolverNode> temp;
    temp.push_back(solutionList_Closed.at(solutionList_Closed.size()-1));
    solutionList_Closed.pop_back();
    for (int i = (int)solutionList_Closed.size()-1 ; i >= 0 ; i--) {
        if (temp.back().prevID == solutionList_Closed[i].ID) {
            temp.push_back(solutionList_Closed[i]);
        }
    }
    
    for (int i = (int)temp.size()-1 ; i >= 0 ; i--) {
        solutionList_Open.push_back(temp[i]);
    }
    
}

std::vector<SolverNode> Solver::getSolution(){
    if (!solutionList_Closed.empty())
        return solutionList_Closed;
    else {
        std::cerr << "Error: Solution not found.\n";
        return solutionList_Closed;
    }
}

unsigned long Solver::getNumOfNodes(){
    return numOfNodes;
}

bool Solver::compairCurrentConfigToGoalConfig(){
    return compairConfigs(currentConfiguration, goalConfiguration);
}

bool Solver::configurationAllreadyChecked(std::string config){
    for (int i = 0 ; i < solutionList_Open.size() ; i++) {
        if (compairConfigs(config, solutionList_Open[i].graphString_AfterDiamondMove)) {
            return true;
        }
    }
    return false;
}

bool Solver::compairConfigs(std::string current, std::string next){
    /**
     Free space:        "."
     Sokoban:           "M"
     Sokoban on goal:   "N"
     Goal:              "G"
     Diamond:           "J"
     Diamond on goal:   "Q"
     */
    // Ignoring the position of the robot:
    for (int i = 0 ; i < current.length() ; i++) {
        if ((current.at(i) != next.at(i)) &&
            !(current.at(i) == 'M' || current.at(i) == 'N' ||
              next.at(i) == 'M'    || next.at(i) == 'N')) {
                return false;
            }
    }
    return true;
}

char Solver::findDirection(int robotIndex, int diamondIndex){
    char direction = 'o';
    
    int xCoorDiff = mapCurrent.getVertex(robotIndex).getXPosition() - mapCurrent.getVertex(diamondIndex).getXPosition();
    int yCoorDiff = mapCurrent.getVertex(robotIndex).getYPosition() - mapCurrent.getVertex(diamondIndex).getYPosition();
    
    // find direction:
    if (abs(xCoorDiff) > abs(yCoorDiff)) {
        // X different
        if (xCoorDiff > 0) {
            // Direction left:
            direction = 'w';
        }
        else {
            // Diretion right:
            direction = 'e';
        }
    }
    else if (abs(xCoorDiff) < abs(yCoorDiff)) {
        // Y different
        if (yCoorDiff > 0) {
            // Direction Up
            direction = 'n';
        }
        else {
            // Direction Down
            direction = 's';
        }
    }
    
    return direction;
}

void Solver::constructorSetup(){
    startConfiguration = mapStart.getGraphRepresentation();
    currentConfiguration = mapCurrent.getGraphRepresentation();
    goalConfiguration = mapGoal.getGraphRepresentation();
    numOfNodes = 0;
    currentNode = 0;
}
