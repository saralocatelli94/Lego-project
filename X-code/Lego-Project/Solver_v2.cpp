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
    std::vector<HashTableStruct> t;
    for (int i = 0 ; i < tableSize ; i++ ) {
        hashTable.push_back(t);
    }
    for (int i = 0 ; i < mapStart->getNumOfVertex() ; i++) {
        int x = rand() % 2000 + 1;
        lookUp_diamondValue.push_back(x);
    }
    diamondPositionGoal = mapGoal->getDiamondPosition();
    // Find deadlock zones:
    findDeadlockZones();
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
     [3] = distance travelsed by robot (using A-star) + 1 for moving the diamond */
    std::vector<std::vector<int>> vertex_MovableAndReachable;
    // A list of ID's of SolverNodes, that is a solution:
    std::vector<int> solutionID;
    
    int itterator = 0;
    int distMax = __INT_MAX__;
    int distCurrent = 0;
    bool deadlock;
    currentRobotPosition = mapStart->getRobotPosition();
    diamondPositionCurrent = mapStart->getDiamondPosition();
    
    diamondPositionCurrent = mapCurrent->getDiamondPosition();
    int hDist = 0;//calculateHeuristicDist(diamondPositionCurrent);
    solutionList_Open.push_back(SolverNode_v2(currentRobotPosition, currentRobotPosition, diamondPositionCurrent, currentNode, -1, distCurrent, 0, 0, hDist));
    numOfNodes++;
    distCurrent = hDist + 0;
    
    PathDrawer startMap(map_width, map_height, *mapStart, deadlockZones);
    startMap.drawMapAndSave("Images/map_start.ppm", true);
    
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
            
            std::vector<int> diamondPosTemp = mapCurrent->getDiamondPosition();
            int robotPosTemp = vertex_MovableAndReachable[i][1];
            
            // Calculate the depth in the "tree" and the total distance of the robot traveled:
            int depth = solutionList_Open[currentNode].depthInTree + 1;
            int dist = vertex_MovableAndReachable[i][3] + 1;
            hDist = 0;//calculateHeuristicDist(diamondPosTemp);
            int tempTotalDist = distCurrent + dist + hDist;
            
            // Create hash:
            int hashIndex = hash(creatHashKey(diamondPosTemp));
            
            // See if we hit the goal.
            if (diamondPositionGoal == diamondPosTemp && tempTotalDist <= distMax) {
                numOfSolutions++;
                solutionID.push_back(numOfNodes);
                distMax = tempTotalDist;
                insertHash_wRobot(hashIndex, diamondPosTemp, robotPosTemp, tempTotalDist);
                solutionList_Open.push_back(SolverNode_v2(vertex_MovableAndReachable[i][0], robotPosTemp, diamondPosTemp, numOfNodes, currentNode, depth, distCurrent, dist, hDist));
                numOfNodes++;
                break;
            }
            
            // Deadlock check:
            deadlock = contain(deadlockZones, vertex_MovableAndReachable[i][2]);
            
            // See if the new config is allready in the openlist, and there is no deadlock. If not, add it.
            if (!lookUpHash_prevAdded_wRobot(hashIndex, diamondPosTemp, robotPosTemp, tempTotalDist) && !deadlock) {
                // Not added before:
                insertHash_wRobot(hashIndex, diamondPosTemp, robotPosTemp, tempTotalDist);
                solutionList_Open.push_back(SolverNode_v2(vertex_MovableAndReachable[i][0], robotPosTemp, diamondPosTemp, numOfNodes, currentNode, depth, distCurrent, dist, hDist));
                numOfNodes++;
            }
            
        }
        
        // Take the next configuration in the open-list, that has a total distance LESS than 'distMax':
        for (int i = currentNode+1 ; i <= numOfNodes ; i++ ) {
            currentNode = i;
            if (currentNode < numOfNodes && solutionList_Open.at(currentNode).distanceTotal < distMax) {
                break;
            }
            // No new nodes have a smaller distance. Break the loop by setting currentNode higher than numOfNodes
            if (currentNode >= numOfNodes) {
                currentNode += 1;
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
    std::cout << "Num of nodes in openlist: " << numOfNodes << std::endl;
    std::cout << "Num of solutions:         " << numOfSolutions << std::endl;
}

void Solver_v2::startSolver_2(bool allowRobotToReverse){
    
    /* A list of the free-space that the robot can actually move the diamond to:
     [0] = Initial robot index
     [1] = current diamond index / later robot index
     [2] = freespace / later dimond index */
    std::vector<std::vector<int>> vertex_MovableFreeSpace;
    /* A list of the dimonds the robot can move, AND get to (by using the A*):
     [0] = Initial robot index
     [1] = current diamond index / later robot index
     [2] = freespace / later dimond index
     [3] = distance travelsed by robot (using A-star) + 1 for moving the diamond */
    std::vector<std::vector<int>> vertex_MovableAndReachable;
    // A list of ID's of SolverNodes, that is a solution:
    std::vector<int> solutionID;
    
    int itterator = 0;
    int distMax = __INT_MAX__ - 1;
    int distCurrent = 0;
    bool deadlock;
    currentRobotPosition = mapStart->getRobotPosition();
    diamondPositionCurrent = mapStart->getDiamondPosition();
    
    diamondPositionCurrent = mapCurrent->getDiamondPosition();
    int hDist = 0;//calculateHeuristicDist(diamondPositionCurrent);
    solutionList_Open.push_back(SolverNode_v2(currentRobotPosition, currentRobotPosition, diamondPositionCurrent, currentNode, -1, distCurrent, 0, 0, hDist));
    numOfNodes++;
    distCurrent = hDist + 0;
    
    PathDrawer startMap(map_width, map_height, *mapStart, deadlockZones);
    startMap.drawMapAndSave("Images/map_start.ppm", true);
    
    while (!solutionList_Open.empty() && distCurrent < distMax) {
        
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
            mapCurrent->setAllVertexInfo(diamondPositionCurrent, currentRobotPosition);
            
            //Set info in new vertex's:
            mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).setDiamond(false);    // Diamond
            mapCurrent->getVertex(vertex_MovableAndReachable[i][1]).setSokoban(true);     // Diamond
            mapCurrent->getVertex(vertex_MovableAndReachable[i][2]).setDiamond(true);     // Free-space
            
            std::vector<int> diamondPosTemp = mapCurrent->getDiamondPosition();
            int robotPosTemp = vertex_MovableAndReachable[i][1];
            
            // Calculate the depth in the "tree" and the total distance of the robot traveled:
            int depth = solutionList_Open.at(0).depthInTree + 1;
            int dist = vertex_MovableAndReachable[i][3] + 1;
            hDist = 0;//calculateHeuristicDist(diamondPosTemp);
            int tempTotalDist = distCurrent + dist + hDist;
            
            // Create hash:
            int hashIndex = hash(creatHashKey(diamondPosTemp));
            
            // See if we hit the goal.
            if (diamondPositionGoal == diamondPosTemp && tempTotalDist <= distMax) {
                numOfSolutions++;
                solutionID.push_back(numOfNodes);
                distMax = tempTotalDist;
                insertHash_wRobot(hashIndex, diamondPosTemp, robotPosTemp, tempTotalDist);
                solutionList_AllExplored.push_back(SolverNode_v2(vertex_MovableAndReachable[i][0], robotPosTemp, diamondPosTemp, numOfNodes, currentNode, depth, distCurrent, dist, hDist));
                numOfNodes++;
                break;
            }
            
            // Deadlock check:
            deadlock = contain(deadlockZones, vertex_MovableAndReachable[i][2]);
            
            // See if the new config is allready in the openlist, and there is no deadlock. If not, add it.
            if (!lookUpHash_prevAdded_wRobot(hashIndex, diamondPosTemp, robotPosTemp, tempTotalDist) && !deadlock) {
                // Not added before:
                insertHash_wRobot(hashIndex, diamondPosTemp, robotPosTemp, tempTotalDist);
                solutionList_Open.push_back(SolverNode_v2(vertex_MovableAndReachable[i][0], robotPosTemp, diamondPosTemp, numOfNodes, currentNode, depth, distCurrent, dist, hDist));
                numOfNodes++;
            }
            
        }
        
        solutionList_AllExplored.push_back(solutionList_Open.at(0));
        solutionList_Open.erase(solutionList_Open.begin());
        
        // Sort the list
        if (!solutionList_Open.empty()) {
            std::sort(solutionList_Open.begin(), solutionList_Open.end());
        }
        
        if (!solutionList_Open.empty() && solutionList_Open.at(0).distanceTotal < distMax) {
            distCurrent = solutionList_Open.at(0).distanceTotal;
            currentNode = solutionList_Open.at(0).ID;
            mapCurrent->setAllVertexInfo(solutionList_Open.at(0).diamondPositions, solutionList_Open.at(0).positionAfter);
        }
        else {
            distCurrent = distMax + 1;
            break;
        }
        
        
    }
    
    // Backtrack:
    for (int i = 0 ; i < numOfSolutions ; i++) {
        std::vector<SolverNode_v2> solutionTemp;
        
        int j = (int)solutionList_AllExplored.size()-1;
        while (solutionList_AllExplored.at(j).ID != solutionID.at(i)) {
            j--;
        }
        
        solutionTemp.push_back(solutionList_AllExplored.at(j));
        
        for (int k = (int)solutionList_AllExplored.size()-1 ; k >= 0 ; k--) {
            if (solutionList_AllExplored.at(k).ID == solutionTemp.back().prevID) {
                solutionTemp.push_back(solutionList_AllExplored.at(k));
            }
        }
        
        // Flip the vector:
        std::vector<SolverNode_v2> solutionFlip;
        for (int j = (int)solutionTemp.size()-1 ; j >= 0 ; j--) {
            solutionFlip.push_back(solutionTemp.at(j));
        }
        
        solutionList_Closed.push_back(solutionFlip);
    }
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
    return deadlockCornerCheck(vertex);
}

void Solver_v2::findDeadlockZones(){
    /* 1) Find 1'st corner.
     * 2) See if the corner is allready in the vector. If not, push it to vector
     * 3) Find the two directions it's edges has.
     * 4) For each direction, go 1 step. Check if vertex exist. Check if vertex fufilles the requirements.
     * 5) If we hit another corner push it to the vector.
     * 6) If all vertex's in between meets the requirements, push them to list aswell.
     */
    for (int i = 0 ; i < mapStart->getNumOfVertex() ; i++) {
        
        bool isACorner = deadlockCornerCheck(i);
        
        if (isACorner) {
            
            // Push corner to list:
            deadlockZones.push_back(i);
            
            // Find the two directions from the corner:
            std::vector<char> dir;
            dir.push_back(mapStart->getVertex(i).connections[0].getDirection());
            dir.push_back(mapStart->getVertex(i).connections[1].getDirection());
            
            // For each direction, the following vertex cant have the corrosponding direction, if it is a deadlock zone:
            std::vector<char> directionsNotAllowed;
            switch (dir[0]) {
                case 'n':
                    switch (dir[1]) {
                        case 'w':
                            directionsNotAllowed.push_back('e');
                            directionsNotAllowed.push_back('s');
                            break;
                        
                        case 'e':
                            directionsNotAllowed.push_back('w');
                            directionsNotAllowed.push_back('s');
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case 'e':
                    switch (dir[1]) {
                        case 'n':
                            directionsNotAllowed.push_back('s');
                            directionsNotAllowed.push_back('w');
                            break;
                            
                        case 's':
                            directionsNotAllowed.push_back('n');
                            directionsNotAllowed.push_back('w');
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case 's':
                    switch (dir[1]) {
                        case 'e':
                            directionsNotAllowed.push_back('w');
                            directionsNotAllowed.push_back('n');
                            break;
                            
                        case 'w':
                            directionsNotAllowed.push_back('e');
                            directionsNotAllowed.push_back('n');
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                case 'w':
                    switch (dir[1]) {
                        case 'n':
                            directionsNotAllowed.push_back('s');
                            directionsNotAllowed.push_back('e');
                            break;
                            
                        case 's':
                            directionsNotAllowed.push_back('n');
                            directionsNotAllowed.push_back('e');
                            break;
                            
                        default:
                            break;
                    }
                    break;
                    
                default:
                    break;
            }
            
            // Go in two directions from the corner:
            for (int j = 0 ; j < dir.size() ; j++) {
                
                /* Criterias:
                 * 1) Not a goal;
                 * 2) Does not have a edge in the direction of 'directionNotAllowed'
                 * 3) Is not in 'deadlockZone'
                 */
                std::vector<int> tempDeadlockZones;
                int nextVertexIndex = mapStart->getVertex(i).connections[j].getTarget()->getIndex();
                bool vertexIsInDeadlockZone = true;
                
                while (!deadlockCornerCheck(nextVertexIndex)) {
                    
                    // If vertex is a goal:
                    if (mapStart->getVertex(nextVertexIndex).getGoal()) {
                        vertexIsInDeadlockZone = false;
                    }
                    // If vertex has edge in wrong direction:
                    for (int k = 0 ; k < mapStart->getVertex(nextVertexIndex).connections.size() ; k++) {
                        if (mapStart->getVertex(nextVertexIndex).connections[k].getDirection() == directionsNotAllowed[j]) {
                            vertexIsInDeadlockZone = false;
                        }
                    }
                    // Is not already in 'deadlockZone'
                    if (contain(deadlockZones, nextVertexIndex)) {
                        vertexIsInDeadlockZone = false;
                    }
                    // If vertex doesn't meet the criteria, break:
                    if (!vertexIsInDeadlockZone) {
                        break;
                    }
                    // Else add it to temp list:
                    else {
                        tempDeadlockZones.push_back(nextVertexIndex);
                    }
                    // Find new vertex:
                    for (int k = 0 ; k < mapStart->getVertex(nextVertexIndex).connections.size() ; k++) {
                        if (mapStart->getVertex(nextVertexIndex).connections[k].getDirection() == dir[j]) {
                            nextVertexIndex = mapStart->getVertex(nextVertexIndex).connections[k].getTarget()->getIndex();
                        }
                    }
                }
                // If all vertex meet the criterias, put them all in list:
                if (vertexIsInDeadlockZone) {
                    for (int k = 0 ; k < tempDeadlockZones.size() ; k++) {
                        deadlockZones.push_back(tempDeadlockZones[k]);
                    }
                }
            }
        }
    }
}

bool Solver_v2::deadlockCornerCheck(int vertex){
    if (mapCurrent->getVertex(vertex).getGoal()) {
        return false;
    }
    // Simpel test, to see if a diamond is in a corner, and cant get retrackted
    int size = (int) mapCurrent->getVertex(vertex).connections.size();
    if (size < 2) {
        return true;
    }
    if (size == 2) {
        
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
    double hDist = 0;
    
    std::vector<int> diamondAssignment;
    std::vector<int> diamondOrder;
    std::vector<double> minDistanceToGoal;
    
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        
        double minDist = __INT_MAX__;
        int minIndex = -1;
        
        for (int j = 0 ; j < numOfDiamonds ; j++) {
            
            double tempDist = pythagoras(diamondPos[i], diamondPositionGoal[j]);
                                         
            if (minDist > tempDist) {
                minDist = tempDist;
                minIndex = j;
            }
        }
        minDistanceToGoal.push_back(minDist);
    }
    
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        
        double minDist = __INT_MAX__;
        int minIndex = -1;
        
        for (int j = 0 ; j < numOfDiamonds ; j++) {
            
            if (minDistanceToGoal[j] < minDist && !contain(diamondOrder, j)) {
                minDist = minDistanceToGoal[j];
                minIndex = j;
            }
        }
        diamondOrder.push_back(minIndex);
    }
    
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        
        double minDist = __INT_MAX__;
        int minIndex = -1;
        
        for (int j = 0 ; j < numOfDiamonds ; j++) {
            
            if (!contain(diamondAssignment, j)) {
                double tempDist = pythagoras(diamondPos[diamondOrder[i]],diamondPositionGoal[j]);
                
                if (minDist > tempDist) {
                    minDist = tempDist;
                    minIndex = j;
                }
            }
        }
        diamondAssignment.push_back(minIndex);
    }
    
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        hDist += pythagoras(diamondPos[diamondOrder[i]], diamondAssignment[diamondAssignment[i]]);
    }
    
    return (int)hDist;
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
/*
void Solver_v2::insertHash(int hashVal, std::vector<int> diamondPos){
    for (int i = 0 ; i < diamondPos.size() ; i++) {
        hashTable.at(hashVal).push_back(diamondPos[i]);
    }
}
*/
bool Solver_v2::lookUpHash_prevAdded(int hashVal){
    return (hashTable.at(hashVal).empty() ? false : true);
}
/*
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
*/
std::string Solver_v2::creatHashKey_wRobot(){
    int s = lookUp_diamondValue[currentRobotPosition];
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        s = s + lookUp_diamondValue[diamondPositionCurrent[i]];
    }
    return std::to_string(s);
}

std::string Solver_v2::creatHashKey_wRobot(std::vector<int> diamondPos, int robPos){
    int s = robPos;
    for (int i = 0 ; i < numOfDiamonds ; i++) {
        s = s + lookUp_diamondValue[diamondPos[i]];
    }
    return std::to_string(s);
}

void Solver_v2::insertHash_wRobot(int hashVal, std::vector<int> diamondPos, int robPos, int dist){
    hashTable.at(hashVal).push_back(HashTableStruct(diamondPos, robPos, dist));
}

bool Solver_v2::lookUpHash_prevAdded_wRobot(int hashVal, std::vector<int> diamondPos, int robPos, int dist){
    if (!lookUpHash_prevAdded(hashVal)) {
        return false;
    }
    HashTableStruct temp(diamondPos, robPos, dist);
    int tempDist = __INT_MAX__;
    bool samePosition = false;
    for (int i = 0 ; i < hashTable.at(hashVal).size() ; i++) {
        bool checked = true;
        // If diamond position is different:
        if (!hashTable.at(hashVal).at(i).dPosEqual(temp)) {
            checked = false;
        }
        // If diamond position is the same, but robot position is different:
        else if (hashTable.at(hashVal).at(i).dPosEqual(temp) && !hashTable.at(hashVal).at(i).rPosEqual(temp)) {
            checked = false;
        }
        // If diamond and robot position is equal, set 'samePosition' to later check if distance is lower then preveous found.
        else if (hashTable.at(hashVal).at(i).dPosEqual(temp) && hashTable.at(hashVal).at(i).rPosEqual(temp)) {
            samePosition = true;
            checked = false;
        }
        // Find the smallest distance in the table:
        if (hashTable.at(hashVal).at(i).distance < tempDist) {
            tempDist = hashTable.at(hashVal).at(i).distance;
        }
        
        if (checked) {
            return true;
        }
    }
    // If same position is found, and distance isn't lower then prev. added, return true;
    if (samePosition && temp.distance > tempDist) {
        return true;
    }
    return false;
}
