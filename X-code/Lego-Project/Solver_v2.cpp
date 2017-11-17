//
//  Solver_v2.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 10/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "Solver_v2.hpp"

Solver_v2::Solver_v2(){
}

Solver_v2::~Solver_v2(){
}

Solver_v2::Solver_v2(Graph & mS, Graph & mG, int numOfDia, int width, int height):
mapStart(&mS), mapGoal(&mG), mapCurrent(&mS), numOfDiamonds(numOfDia), map_width(width), map_height(height){
    numOfNodes = 0;
    currentNode = 0;
    numOfSolutions = 0;
    std::vector<int> t;
    for (int i = 0 ; i < tableSize ; i++ ) {
        hashTable.push_back(t);
    }
    for (int i = 0 ; i < mapStart->getNumOfVertex() ; i++) {
        int x = rand() % 1000 + 1;
        lookUp_diamondValue.push_back(x);
    }
    diamondPositionGoal = mapGoal->getDiamondPosition();
}

/********************** Solver functions ************************/

void Solver_v2::startSolver(bool allowRobotToReverse){
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
    
    /* A list of the free-space that the robot can actually move the diamond to:
     [0] = Initial robot index
     [1] = current diamond index / later robot index
     [2] = freespace / later dimond index */
    std::vector<std::vector<int>> vertex_MovableFreeSpace;
    /* A list of the dimonds the robot can move, AND get to (by using the A*):
     [0] = Initial robot index
     [1] = current diamond index / later robot index
     [2] = freespace / later dimond index
     [3] = distance travelsed by robot (using A-star) */
    std::vector<std::vector<int>> vertex_MovableAndReachable;
    // A list of ID's of SolverNodes, that is a solution:
    std::vector<int> solutionID;
    
    int itterator = 0;
    int distMax = __INT_MAX__;
    int distCurrent = 0;
    bool deadlock;
    
    diamondPositionCurrent = mapCurrent->getDiamondPosition();
    int hDist = calculateHeuristicDist(diamondPositionCurrent);
    solutionList_Open.push_back(SolverNode_v2(mapStart->getRobotPosition(), mapStart->getRobotPosition(), mapStart->getDiamondPosition(), currentNode, -1, 0, 0, hDist));
    numOfNodes++;
    
    
    while (numOfNodes > currentNode && distCurrent < distMax) {
        
        diamondPositionCurrent.clear();
        vertex_MovableFreeSpace.clear();
        vertex_MovableAndReachable.clear();
        
        if (itterator > 1000) {
            std::cout << "Num of nodes:    " << numOfNodes << std::endl;
            itterator = 0;
        }
        itterator++;
        
        // Find free-space around the diamonds:
        currentRobotPosition = mapCurrent->getRobotPosition();
        diamondPositionCurrent = mapCurrent->getDiamondPosition();
        for (int i = 0 ; i < numOfDiamonds ; i++) {
            for (int k = 0 ; k < mapCurrent->getVertex(diamondPositionCurrent[i]).connections.size() ; k++) {
                if (!mapCurrent->getVertex(diamondPositionCurrent[i]).connections[k].getTarget()->getDiamond()) {
                    // The neighbour vertex is a free-space:
                    char direction = findDirection(diamondPositionCurrent[i], mapCurrent->getVertex(diamondPositionCurrent[i]).connections[k].getTarget()->getIndex());
                    // Find a free-space in the opposite direction:
                    for (int j = 0 ; j < mapCurrent->getVertex(diamondPositionCurrent[i]).connections.size() ; j++) {
                        switch (direction) {
                            case 'n':
                                if (mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getDirection() == 's' &&
                                    !mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getDiamond()) {
                                    std::vector<int> f;
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[k].getTarget()->getIndex());
                                    f.push_back(diamondPositionCurrent[i]);
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getIndex());
                                    vertex_MovableFreeSpace.push_back(f);
                                }
                                break;
                                
                            case 'e':
                                if (mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getDirection() == 'w' &&
                                    !mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getDiamond()) {
                                    std::vector<int> f;
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[k].getTarget()->getIndex());
                                    f.push_back(diamondPositionCurrent[i]);
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getIndex());
                                    vertex_MovableFreeSpace.push_back(f);
                                }
                                break;
                                
                            case 's':
                                if (mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getDirection() == 'n' &&
                                    !mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getDiamond()) {
                                    std::vector<int> f;
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[k].getTarget()->getIndex());
                                    f.push_back(diamondPositionCurrent[i]);
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getIndex());
                                    vertex_MovableFreeSpace.push_back(f);
                                }
                                break;
                                
                            case 'w':
                                if (mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getDirection() == 'e' &&
                                    !mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getDiamond()) {
                                    std::vector<int> f;
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[k].getTarget()->getIndex());
                                    f.push_back(diamondPositionCurrent[i]);
                                    f.push_back(mapCurrent->getVertex(diamondPositionCurrent[i]).connections[j].getTarget()->getIndex());
                                    vertex_MovableFreeSpace.push_back(f);
                                }
                                break;
                                
                            default:
                                break;
                        }
                    }
                }
            }
        }
        
        // Find the once that the robot can actually move to from current position:
        for (int i = 0 ; i < vertex_MovableFreeSpace.size() ; i++ ) {
            char direction = findDirection(vertex_MovableFreeSpace[i][0], vertex_MovableFreeSpace[i][1]);
            // If start- and end-vertex are the same:
            if (currentRobotPosition == vertex_MovableFreeSpace[i][0]) {
                std::vector<int> temp;
                temp.push_back(vertex_MovableFreeSpace[i][0]);         //
                temp.push_back(vertex_MovableFreeSpace[i][1]);         //
                temp.push_back(vertex_MovableFreeSpace[i][2]);         //
                temp.push_back(0);                                     //
                vertex_MovableAndReachable.push_back(temp);
            }
            else {
                AStar path(*mapCurrent,
                           mapCurrent->getVertex(currentRobotPosition),
                           mapCurrent->getVertex(vertex_MovableFreeSpace[i][0]),
                           direction);
                if (path.runAStar(allowRobotToReverse)) {
                    std::vector<int> temp;
                    temp.push_back(vertex_MovableFreeSpace[i][0]);      //
                    temp.push_back(vertex_MovableFreeSpace[i][1]);      //
                    temp.push_back(vertex_MovableFreeSpace[i][2]);      //
                    temp.push_back((int)path.getPath().size()-1);       //
                    vertex_MovableAndReachable.push_back(temp);
                }
            }
        }
        
        // Find all new configurations from the current position, and save all new once (discard once we have tried before)
        for (int i = 0 ; i < vertex_MovableAndReachable.size() ; i++) {
            //TODO: Reconfigure the map, to current map.
            mapCurrent->setAllVertexInfo(diamondPositionCurrent, currentRobotPosition);
            
            //Set info in new vertex's:
            mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).setDiamond(false);    // Diamond
            mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).setSokoban(true);     // Diamond
            mapCurrent->getVertex(vertex_MovableAndReachable[i][2]).setDiamond(true);     // Free-space
            
            // Calculate the depth in the "tree" and the total distance of the robot traveled:
            int depth = solutionList_Open[currentNode].depthInTree + 1;
            int dist = solutionList_Open[currentNode].distanceTotal + vertex_MovableAndReachable[i][3] + 1;
            
            std::vector<int> diamondPosTemp = mapCurrent->getDiamondPosition();
            int robotPosTemp = vertex_MovableAndReachable[i][1];
            
            hDist = calculateHeuristicDist(diamondPosTemp);
            
            // See if we hit the goal.
            if (diamondPositionGoal == diamondPosTemp && dist <= distMax) {
                numOfSolutions++;
                solutionID.push_back(numOfNodes);
                distMax = dist;
                insertHash(hash(creatHashKey(diamondPosTemp)), diamondPosTemp);
                solutionList_Open.push_back(SolverNode_v2(vertex_MovableAndReachable[i][0], robotPosTemp, diamondPosTemp, numOfNodes, currentNode, depth, dist, hDist));
                numOfNodes++;
                break;
            }
            
            // Deadlock check:
            deadlock = deadlockCheck(vertex_MovableAndReachable[i][2]);
            
            // See if the new config is allready in the openlist, and there is no deadlock. If not, add it.
            if (!lookUpHash_prevAdded( hash(creatHashKey(diamondPosTemp)), diamondPosTemp) && !deadlock) {
                // Not added before:
                insertHash(hash(creatHashKey(diamondPosTemp)), diamondPosTemp);
                solutionList_Open.push_back(SolverNode_v2(vertex_MovableAndReachable[i][0], robotPosTemp, diamondPosTemp, numOfNodes, currentNode, depth, dist, hDist));
                numOfNodes++;
            }
            
        }
        
        // Take the next configuration in the open-list, that has a total distance LESS than 'distMax':
        for (int i = currentNode+1 ; i <= numOfNodes ; i++ ) {
            currentNode = i;
            if (i < numOfNodes && solutionList_Open.at(i).distanceTotal < distMax) {
                break;
            }
            // No new nodes have a smaller distance. Break the loop by setting currentNode higher than numOfNodes
            if (i == numOfNodes) {
                currentNode = numOfNodes + 1;
                break;
            }
        }
        
        // Reset to next configuration in open-list:
        if (currentNode <= numOfNodes) {
            mapCurrent->setAllVertexInfo(solutionList_Open.at(currentNode).diamondPositions, solutionList_Open.at(currentNode).positionAfter);
            distCurrent = solutionList_Open.at(currentNode).distanceTotal;
        }
        
        
    }
    
    // Backtrack:
    for (int i = 0 ; i < numOfSolutions ; i++) {
        std::vector<SolverNode_v2> solutionTemp;
        solutionTemp.push_back(solutionList_Open.at(solutionID.at(i)));
        
        int p_ID = solutionList_Open.at(solutionID.at(i)).prevID;
        while (p_ID > -1) {
            solutionTemp.push_back(solutionList_Open.at(p_ID));
            p_ID = solutionList_Open.at(p_ID).prevID;
        }
        
        // Flip the vector:
        std::vector<SolverNode_v2> solutionFlip;
        for (int j = (int)solutionTemp.size()-1 ; j >= 0 ; j--) {
            solutionFlip.push_back(solutionTemp.at(j));
        }
        
        solutionList_Closed.push_back(solutionFlip);
    }
    /*
    // Print the solutions as images:
    for (int j = 0 ; j < numOfSolutions ; j++) {
        for (int i = 1 ; i < solutionList_Closed[j].size() ; i++) {
            Graph temp = *mapCurrent;
            std::vector<int> dPosTemp = solutionList_Closed[j][i].diamondPositions;
            int robotPos = solutionList_Closed[j][i].positionAfter;
            temp.setAllVertexInfo(dPosTemp, robotPos);
            AStar aStarTest(temp, temp.getVertex(solutionList_Closed[j][i-1].positionAfter), temp.getVertex(solutionList_Closed[j][i].positionAfter), 'n');
            
            PathDrawer a(map_width, map_height, temp);
            a.drawMapAndSave("Images/solution" + std::to_string(j+1) + "_step" + std::to_string(i) + "_ID" + std::to_string(solutionList_Closed[j][i].ID) + ".ppm");
        }
    }
    */
    std::cout << "Num of nodes in openlist: " << numOfNodes << std::endl;
    std::cout << "Num of solutions:         " << numOfSolutions << std::endl;
}

std::vector<std::vector<SolverNode_v2>> Solver_v2::getSolution() {
    if (!solutionList_Closed.empty())
        return solutionList_Closed;
    else {
        std::cerr << "Error: Solution not found.\n";
        return solutionList_Closed;
    }
}

char Solver_v2::findDirection(int robotIndex, int diamondIndex){
    char direction = 'o';
    for (int i = 0 ; i < mapCurrent->getVertex(robotIndex).connections.size() ; i++) {
        if (mapCurrent->getVertex(robotIndex).connections[i].getTarget()->getIndex() == diamondIndex) {
            direction = mapCurrent->getVertex(robotIndex).connections[i].getDirection();
            break;
        }
    }
    return direction;
}

bool Solver_v2::deadlockCheck(int vertex){
    if (mapCurrent->getVertex(vertex).getGoal()) {
        return false;
    }
    // Simpel test, to see if a diamond is in a corner, and cant get retrackted
    int size = (int) mapCurrent->getVertex(vertex).connections.size();
    if (size < 3 && mapCurrent->getVertex(vertex).getDiamond()) {
        
        switch (mapCurrent->getVertex(vertex).connections[0].getDirection()) {
            case 'n':
                if (mapCurrent->getVertex(vertex).connections[1].getDirection() != 's')
                    return true;
                break;
            case 'e':
                if (mapCurrent->getVertex(vertex).connections[1].getDirection() != 'w')
                    return true;
                break;
            case 's':
                if (mapCurrent->getVertex(vertex).connections[1].getDirection() != 'n')
                    return true;
                break;
            case 'w':
                if (mapCurrent->getVertex(vertex).connections[1].getDirection() != 'e')
                    return true;
                break;
            default:
                break;
        }
    }
    return false;
}

int Solver_v2::calculateHeuristicDist(std::vector<int> diamondPos){
    int hDist = 0;
    
    // Which goal is the diamonds connected to:
    std::vector<int> diamondsConnectedToGoal;
    
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        // Find the smallest straight line distance to the goal.
        double straightLineDist[numOfDiamonds];
        int smallest = 0, minDist = __INT_MAX__;
        for (int j = 0 ; j < numOfDiamonds ; j++) {
            straightLineDist[j] = pythagoras(diamondPos[i], diamondPositionGoal[j]);
            // If distance is smaller that preveous, and j is not already assigned:
            if (straightLineDist[j] < minDist && !contain(diamondsConnectedToGoal, j)) {
                minDist = straightLineDist[j];
                smallest = j;
            }
        }
        diamondsConnectedToGoal.push_back(smallest);
    }
    
    // hDist = sum of all x and y differences.
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        int xDiff = abs(mapCurrent->getVertex(diamondPos[i]).getXPosition() - mapCurrent->getVertex(diamondPositionGoal.at(diamondsConnectedToGoal[i])).getXPosition());
        int yDiff = abs(mapCurrent->getVertex(diamondPos[i]).getYPosition() - mapCurrent->getVertex(diamondPositionGoal.at(diamondsConnectedToGoal[i])).getYPosition());
        hDist = hDist + xDiff + yDiff;
    }
    
    return hDist;
}

double Solver_v2::pythagoras(int vertex_a, int vertex_b){
    int x_a = mapCurrent->getVertex(vertex_a).getXPosition();
    int y_a = mapCurrent->getVertex(vertex_a).getYPosition();
    int x_b = mapCurrent->getVertex(vertex_b).getXPosition();
    int y_b = mapCurrent->getVertex(vertex_b).getYPosition();
    return sqrt((x_a - x_b) * (x_a - x_b) + (y_a - y_b) * (y_a - y_b));
}

bool Solver_v2::contain(std::vector<int> vec, int num){
    for (int i = 0 ; i < vec.size() ; i++) {
        if (vec.at(i) == num) {
            return true;
        }
    }
    return false;
}

/********************** Hash functions ************************/

std::string Solver_v2::creatHashKey(){
    int s = 0;
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        s = s + lookUp_diamondValue[diamondPositionCurrent[i]];
    }
    return std::to_string(s);
}

std::string Solver_v2::creatHashKey(std::vector<int> diamondPos){
    int s = 0;
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        s = s + lookUp_diamondValue[diamondPos[i]];
    }
    return std::to_string(s);
}

int Solver_v2::hash(std::string key){
    int hashValue = 0;
    for (char c : key) {
        hashValue = 37 * hashValue + c;
    }
    return hashValue % tableSize;
}

void Solver_v2::insertHash(int hashVal, std::vector<int> diamondPos){
    for (int i = 0 ; i < diamondPos.size() ; i++) {
        hashTable.at(hashVal).push_back(diamondPos[i]);
    }
}

bool Solver_v2::lookUpHash_prevAdded(int hashVal){
    return (hashTable.at(hashVal).empty() ? false : true);
}

bool Solver_v2::lookUpHash_prevAdded(int hashVal, std::vector<int> diamondPos){
    if (!lookUpHash_prevAdded(hashVal)) {
        return false;
    }
    int itterations = (int)hashTable.at(hashVal).size() / numOfDiamonds;
    for (int i = 0 ; i < itterations ; i++) {
        bool checked = true;
        for (int k = 0 ; k < numOfDiamonds ; k++) {
            if (hashTable.at(hashVal)[k + i*numOfDiamonds] != diamondPos[k]) {
                checked = false;
            }
        }
        if (checked) {
            return true;
        }
    }
    return false;
}
