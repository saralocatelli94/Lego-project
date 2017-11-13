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

Solver::Solver(Graph & mS, Graph & mG):
mapStart(&mS), mapGoal(&mG), mapCurrent(&mS){
    constructorSetup();
}

Solver::Solver(Graph & mS, Graph & mG, int width, int height):
mapStart(&mS), mapGoal(&mG), mapCurrent(&mS), map_width(width), map_height(height){
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
    
    // A list of all diamonds's index in the graph:
    std::vector<int> vertex_diamonds;
    // A list of all vertex's in the graph that does NOT have i diamond, and has an edge to a vertex containing a diamond.
    std::vector<std::vector<int>> vertex_NextToDiamonds;
    // A list of the dimonds that ro robot can actually move:
    std::vector<std::vector<int>> vertex_Movable;
    // A list of the dimonds the robot can move, AND get to (by using the A*):
    std::vector<std::vector<int>> vertex_MovableAndReachable;
    // A list of ID's of SolverNodes, that is a solution:
    std::vector<long> solutionID;
    
    int ID_current = -1;
    int itterator = 0;
    int distMax = __INT_MAX__;
    int distCurrent = 0;
    bool deadlock;
    
    // Start the solver:
    while (/*distCurrent < distMax && numOfNodes >= currentNode*/ true) {
        /*
        // If we have found the goal:
        if (compairCurrentConfigToGoalConfig()) {
            break;
        }
        */
        vertex_diamonds.clear();
        vertex_NextToDiamonds.clear();
        vertex_Movable.clear();
        vertex_MovableAndReachable.clear();
        
        if (itterator > 500) {
            std::cout << "Num of nodes:    " << numOfNodes << std::endl;
            itterator = 0;
        }
        itterator++;
        
        // Update the index's of the diamonds position:
        for (int i = 0 ; i < mapCurrent->getNumOfVertex() ; i++) {
            if (mapCurrent->getVertex(i).getDiamond()) {
                vertex_diamonds.push_back(mapCurrent->getVertex(i).getIndex());
            }
        }
        
        // Find the connections from the diamonds position, that does not contain a diamond:
        for (int i = 0 ; i < vertex_diamonds.size() ; i++) {
            for (int j = 0 ; j < mapCurrent->getVertex(vertex_diamonds[i]).connections.size() ; j++) {
                if (!mapCurrent->getVertex(vertex_diamonds[i]).connections[j].getTarget()->getDiamond()) {
                    std::vector<int> temp;
                    temp.push_back(mapCurrent->getVertex(vertex_diamonds[i]).connections[j].getTarget()->getIndex());  // Free space
                    temp.push_back(vertex_diamonds[i]);  // Diamond
                    vertex_NextToDiamonds.push_back(temp);
                    
                }
            }
        }
        
        // From the found vertex's above, find the ones that has a free-space on the other side of the diamond seen from the robot:
        for (int i = 0 ; i < vertex_NextToDiamonds.size() ; i++) {
            int robotIndex = vertex_NextToDiamonds[i][0];
            int diamondIndex = vertex_NextToDiamonds[i][1];
            
            char direction = findDirection(robotIndex, diamondIndex);
            std::string nextVertexName;
            
            if (direction == 'w') {
                nextVertexName = std::to_string(mapCurrent->getVertex(robotIndex).getYPosition()) +
                std::to_string(mapCurrent->getVertex(diamondIndex).getXPosition() - 1);
            }
            else if (direction == 'e') {
                nextVertexName = std::to_string(mapCurrent->getVertex(robotIndex).getYPosition()) +
                std::to_string(mapCurrent->getVertex(diamondIndex).getXPosition() + 1);

            }
            else if (direction == 'n') {
                nextVertexName = std::to_string(mapCurrent->getVertex(diamondIndex).getYPosition() - 1) +
                std::to_string(mapCurrent->getVertex(robotIndex).getXPosition());

            }
            else if (direction == 's') {
                nextVertexName = std::to_string(mapCurrent->getVertex(diamondIndex).getYPosition() + 1) +
                std::to_string(mapCurrent->getVertex(robotIndex).getXPosition());
            }
            
            // If vertex exist in graph and does not contain a diamond, the movement of the diamond is valid:
            if (mapCurrent->vertexExist(nextVertexName)) {
                if (!mapCurrent->getVertex(nextVertexName).getDiamond()) {
                    vertex_Movable.push_back(vertex_NextToDiamonds[i]);
                }
            }
        }
        
        // Find the position of the sokoban:
        int sokobanIndexPosition = -1;
        for (int i = 0 ; i < mapCurrent->getNumOfVertex() ; i++) {
            if (mapCurrent->getVertex(i).getSokoban()){
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
            if (mapCurrent->getVertex(sokobanIndexPosition).getIndex() == mapCurrent->getVertex(vertex_Movable[i][0]).getIndex()) {
                std::vector<int> temp2;
                temp2.push_back(vertex_Movable[i][0]);          // Free space
                temp2.push_back(vertex_Movable[i][1]);          // Diamond
                temp2.push_back(0);                             // Length of robot-path
                vertex_MovableAndReachable.push_back(temp2);
            }
            else {
                AStar temp(*mapCurrent,
                           mapCurrent->getVertex(sokobanIndexPosition),
                           mapCurrent->getVertex(vertex_Movable[i][0]),
                           direction);
                if (temp.runAStar()) {
                    std::vector<int> temp2;
                    temp2.push_back(vertex_Movable[i][0]);          // Free space
                    temp2.push_back(vertex_Movable[i][1]);          // Diamond
                    temp2.push_back((int)temp.getPath().size()-1);    // Length of robot-path
                    
                    vertex_MovableAndReachable.push_back(temp2);
                }
            }
        }

        // Save the current map configs:
        std::string configBeforeMoving = mapCurrent->getGraphRepresentation();
        std::string configAfterMoving;
        
        // Move the diamond one space in the direction:
        for (int i = 0 ; i < vertex_MovableAndReachable.size() ; i++) {
            // Because we change the map on each itteration, reset the map to the current node in openlist
            mapCurrent->setAllVertexInfo(configBeforeMoving);
            
            // Get the direction of movement:
            int robotIndex = vertex_MovableAndReachable[i][0];
            int diamondIndex = vertex_MovableAndReachable[i][1];
            char direction = findDirection(robotIndex, diamondIndex);
            
            // Move the robot to new position, and erase the diamond from same position:
            mapCurrent->getVertex(sokobanIndexPosition).setSokoban(false);
            mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).setSokoban(true);
            mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).setDiamond(false);
            
            // Find name of new vertex:
            std::string newVertexName;
            int xPos = mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).getXPosition();
            int yPos = mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).getYPosition();
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
            mapCurrent->getVertex(newVertexName).setDiamond(true);
            
            // Reset deadlock check:
            deadlock = false;//deadlockCheck(newVertexName);
            // debug:
            // if (deadlock) {std::cout << "Deadlock at X = " << mapCurrent->getVertex(newVertexName).getXPosition() << ", Y = " << mapCurrent->getVertex(newVertexName).getYPosition() << ".\n";}
            
            // Save the new map configuration:
            configAfterMoving = mapCurrent->getGraphRepresentation();
            
            // Calculate the depth in the "tree" and the total distance of the robot traveled:
            int depth;
            long dist;
            if (ID_current == -1) {
                depth = 0;
                dist = vertex_MovableAndReachable[i][2];
            } else {
                depth = solutionList_Open[ID_current].depthInTree + 1;
                dist = solutionList_Open[ID_current].distanceTraveled + vertex_MovableAndReachable[i][2] + 1;
            }
            
            // See if the new config is allready in the openlist:
            if ((!configurationAllreadyChecked(configAfterMoving) || compairConfigs(configAfterMoving, goalConfiguration))
                && !deadlock) {
                // Put the new map in the openlist:
                solutionList_Open.push_back(SolverNode(configBeforeMoving, configAfterMoving,
                                                       numOfNodes, ID_current,
                                                       dist, depth));
                numOfNodes++;
                
                // check to see if we have found the goal config:
                if (compairConfigs(configAfterMoving, goalConfiguration)) {
                    //currentConfiguration = configAfterMoving;
                    numOfSolutions++;
                    distMax = solutionList_Open.at(solutionList_Open.size()-1).distanceTraveled;
                    solutionID.push_back(solutionList_Open.at(solutionList_Open.size()-1).ID);
                    break;
                }
            }
            else {
                //std::cout << "Already been checked\n";
            }
        }
        
        // See if we have found the goal:
        if (!compairConfigs(currentConfiguration, goalConfiguration)) {
            // take the next config from openlist:
            currentConfiguration = solutionList_Open.at(currentNode).graphString_AfterDiamondMove;
        }
        else {
            std::cout << "We hit the goal.\n";
            std::cout << "Num of nodes:    " << numOfNodes << std::endl;
            currentConfiguration = solutionList_Open.at(currentNode).graphString_AfterDiamondMove;
        }
        
        distCurrent = solutionList_Open.at(currentNode).distanceTraveled;
        mapCurrent->setAllVertexInfo(currentConfiguration);
        ID_current++;
        currentNode++;
        
        if (currentNode == numOfNodes) {
            break;
        }
    }
    
    // Backtrack to find the solutions:
    for (int i = 0 ; i < numOfSolutions ; i++) {
        std::vector<SolverNode> temp;
        temp.push_back(solutionList_Open.at(solutionID.at(i)));
        for (int j = (int)solutionList_Open.at(solutionID.at(i)).ID - 1 ; j >= 0 ; j--) {
            if (temp.back().prevID == solutionList_Open[j].ID) {
                temp.push_back(solutionList_Open[j]);
            }
        }
        
        // Flip the vector:
        std::vector<SolverNode> tempFlip;
        for (int j = (int)temp.size()-1 ; j >= 0 ; j--) {
            tempFlip.push_back(temp[j]);
        }
        
        solutionList_Closed.push_back(tempFlip);
        
    }
    
    // Print the solutions as images:
    for (int j = 0 ; j < numOfSolutions ; j++) {
        for (int i = 0 ; i < solutionList_Closed[j].size() ; i++) {
            Graph temp = *mapCurrent;
            temp.setAllVertexInfo(solutionList_Closed[j][i].graphString_AfterDiamondMove);
            AStar aStarTest(temp, temp.getVertex(solutionList_Closed[j][i].position_BeforeDiamondMove), temp.getVertex(solutionList_Closed[j][i].position_AfterDiamondMove), 'n');
            
            PathDrawer a(map_width, map_height, temp);
            a.drawMapAndSave("Images/solution" + std::to_string(j+1) + "_step" + std::to_string(i) + "_ID" + std::to_string(solutionList_Closed[j][i].ID) + ".ppm");
        }
    }
    
    
    std::cout << "Num of solutions: " << numOfSolutions << std::endl;
}

std::vector<std::vector<SolverNode>> Solver::getSolution(){
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
    
    int xCoorDiff = mapCurrent->getVertex(robotIndex).getXPosition() - mapCurrent->getVertex(diamondIndex).getXPosition();
    int yCoorDiff = mapCurrent->getVertex(robotIndex).getYPosition() - mapCurrent->getVertex(diamondIndex).getYPosition();
    
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

bool Solver::deadlockCheck(){
    // Simpel test, to see if a diamond is in a corner, and cant get retrackted
    
    for (int i = 0 ; i < mapCurrent->getNumOfVertex() ; i++) {
        if (mapCurrent->getVertex(i).getDiamond() && deadlockCheck(mapCurrent->getVertex(i).getName())) {
            return true;
        }
    }
    return false;
}

bool Solver::deadlockCheck(std::string vertexName){
    // Simpel test, to see if a diamond is in a corner, and cant get retrackted
    int size = (int) mapCurrent->getVertex(vertexName).connections.size();
    if (size < 3 && mapCurrent->getVertex(vertexName).getDiamond()) {
        
        switch (mapCurrent->getVertex(vertexName).connections[0].getDirection()) {
            case 'n':
                if (mapCurrent->getVertex(vertexName).connections[1].getDirection() != 's')
                    return true;
                break;
            case 'e':
                if (mapCurrent->getVertex(vertexName).connections[1].getDirection() != 'w')
                    return true;
                break;
            case 's':
                if (mapCurrent->getVertex(vertexName).connections[1].getDirection() != 'n')
                    return true;
                break;
            case 'w':
                if (mapCurrent->getVertex(vertexName).connections[1].getDirection() != 'e')
                    return true;
                break;
            default:
                break;
        }
    }
    return false;
}

void Solver::constructorSetup(){
    startConfiguration = mapStart->getGraphRepresentation();
    currentConfiguration = mapCurrent->getGraphRepresentation();
    goalConfiguration = mapGoal->getGraphRepresentation();
    numOfNodes = 0;
    currentNode = 0;
    numOfSolutions = 0;
}
