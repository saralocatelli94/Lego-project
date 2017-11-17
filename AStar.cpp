//
//  AStar.cpp
//  Sokoban Solver
//
//  Created by Olliver Ordell on 24/09/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "AStar.hpp"

AStar::AStar(){
    
}

AStar::AStar(Graph roadMap, Vertex vertexStart, Vertex vertexGoal, char dirGoal):
map(roadMap), vStart(vertexStart), vGoal(vertexGoal), directionGoal(dirGoal){
    directionStart = '0';
}

AStar::AStar(Graph roadMap, Vertex vertexStart, Vertex vertexGoal, char dirStart, char dirGoal):
map(roadMap), vStart(vertexStart), vGoal(vertexGoal), directionStart(dirStart), directionGoal(dirGoal){
    
}

bool AStar::runAStar(bool allowSokobanToReverse){
    if (!validStartAndGoal()) {
        //std::cerr << "Goal and/or start not valid.\n";
        //return false;
    }
    
    currentVertexIndex = vStart.getIndex();
    
    // Add startVertex to visited list:
    vertexClosedList.push_back(VertexList(&map.getVertex(currentVertexIndex), NULL, 0, heuristicDistance(vStart, vGoal), directionStart));
    
    int openListIndex = 0;
    int closedListIndex = 0;
    
    while (currentVertexIndex != vGoal.getIndex())
    {
        // Go through all connections from current vertex:
        for (int i = 0 ; i < map.getVertex(currentVertexIndex).connections.size() ; i++){
            
            // Check if the new vertex is allready in vertexOpenList
            bool prevAddedToOpenList = false;
            for (int j = 0 ; j < vertexOpenList.size() ; j++)
            {
                if (vertexOpenList[j].vertexTarget->getIndex() == map.getVertex(currentVertexIndex).connections[i].getTarget()->getIndex())
                {
                    prevAddedToOpenList = true;
                    openListIndex = j;
                }
            }
            
            // Check if connection is in vertexClosedList. If so, it's not supposed to be put in vertexClosedList
            bool prevAddedToClosedList = false;
            for (int j = 0 ; j < vertexClosedList.size() ; j++)
            {
                if (vertexClosedList[j].vertexTarget->getIndex() == map.getVertex(currentVertexIndex).connections[i].getTarget()->getIndex())
                {
                    prevAddedToClosedList = true;
                    
                }
            }
            
            // If connection is completly new, add it to vertexOpenList:
            if (!prevAddedToOpenList && !prevAddedToClosedList &&
                !map.getVertex(currentVertexIndex).connections[i].getTarget()->getDiamond())
            {
                // See if ro robot needs to turn 90 degree
                
                double weight = calcWeight(vertexClosedList[closedListIndex], map.getVertex(currentVertexIndex).connections[i], allowSokobanToReverse);
                
                // See of the robot should revers:
                char orientation;
                if (allowSokobanToReverse) {
                    orientation = calcOrientation(vertexClosedList[closedListIndex].orientationOfRobot,
                                                   map.getVertex(currentVertexIndex).connections[i].getDirection());
                }
                else {
                    orientation = map.getVertex(currentVertexIndex).connections[i].getDirection();
                }

                vertexOpenList.push_back(VertexList(map.getVertex(currentVertexIndex).connections[i].getTarget(),
                                                    &map.getVertex(currentVertexIndex),
                                                    weight,
                                                    heuristicDistance(vGoal, *map.getVertex(currentVertexIndex).connections[i].getTarget()),
                                                    orientation));
            }
            // Check if new route is cheaper than prev found route:
            else if (prevAddedToOpenList && !prevAddedToClosedList &&
                     !map.getVertex(currentVertexIndex).connections[i].getTarget()->getDiamond())
            {
                if (vertexOpenList[openListIndex].costTravel >
                    (map.getVertex(currentVertexIndex).connections[i].getWeight() + vertexClosedList[closedListIndex].costTravel))
                {
                    // See if ro robot needs to turn 90 degree
                    double weight = calcWeight(vertexClosedList[closedListIndex], map.getVertex(currentVertexIndex).connections[i], allowSokobanToReverse);
                    char orientation;
                    // See of the robot should revers:
                    if (allowSokobanToReverse) {
                        orientation = calcOrientation(vertexClosedList[closedListIndex].orientationOfRobot,
                                                      map.getVertex(currentVertexIndex).connections[i].getDirection());
                    }
                    else {
                        orientation = map.getVertex(currentVertexIndex).connections[i].getDirection();
                    }
                    
                    vertexOpenList[openListIndex].vertexPrevious = &map.getVertex(currentVertexIndex);
                    vertexOpenList[openListIndex].costTravel = weight;
                    vertexOpenList[openListIndex].orientationOfRobot = orientation;
                }
            }
        
        }
        
        // Safty check:
        if (vertexOpenList.empty())
        {
            //std::cerr << "ERROR: No valid path to goal.\n";
            return false;
        }
        
        // Now take the closest vertex to the goal. Add it to vertexClosedList, and remove it from vertexOpenList
        double tempDist = __DBL_MAX__;
        
        // Find the vertex with lowest total cost
        for (int i = 0 ; i < vertexOpenList.size() ; i++)
        {
            if (vertexOpenList[i].cost_Total() < tempDist)
            {
                openListIndex = i;
                tempDist = vertexOpenList[i].cost_Total();
            }
        }
        // Add lowest cost vertex to closed list:
        vertexClosedList.push_back(vertexOpenList[openListIndex]);
        // Delete from open list:
        vertexOpenList.erase(vertexOpenList.begin()+openListIndex);
        
        // update index-variables:
        closedListIndex = (int)vertexClosedList.size()-1;
        currentVertexIndex = vertexClosedList[closedListIndex].vertexTarget->getIndex();
        
    }
    
    // Backtrack the shortest found path:
    // Pushback last vertex
    vertexToGoal.push_back(vertexClosedList.at(vertexClosedList.size()-1));
    vertexClosedList.pop_back();
    
    for (int i = (int)vertexClosedList.size()-1 ; i >= 0 ; i--)
    {
        if (vertexToGoal.back().vertexPrevious->getIndex() == vertexClosedList[i].vertexTarget->getIndex())
        {
            vertexToGoal.push_back(vertexClosedList[i]);
        }
    }
    
    //printVertexToGoal();
    return true;
}

std::vector<VertexList> AStar::getPath(){
    return vertexToGoal;
}

void AStar::printVertexToGoal(){
    std::cout << "Printing the road to goal:\n";
    std::cout << std::setw(6) << "Index:";
    std::cout << std::setw(7) << "Name:";
    std::cout << std::setw(13) << "Orientation:" << std::endl;
    for (int i = (int)vertexToGoal.size()-1 ; i >= 0 ; i--)
    {
        std::cout << std::setw(6) << vertexToGoal[i].vertexTarget->getIndex();
        std::cout << std::setw(7) << vertexToGoal[i].vertexTarget->getName();
        std::cout << std::setw(13) << vertexToGoal[i].orientationOfRobot;
        std::cout << std::endl;
    }
    std::cout << "Number of vertex's: " << vertexToGoal.size() << std::endl;
}

bool AStar::validStartAndGoal(){
    if (!vStart.getSokoban()) {
        //std::cerr << "ERROR: Sokoban not on start-Vertex.\n";
        //return false;
    }
    else if (vStart.getIndex() == vGoal.getIndex()) {
        //std::cerr << "ERROR: Start- and Goal-Vertex are the same\n";
        //return false;
    }
    return true;
}

double AStar::heuristicDistance(Vertex current, Vertex goal){
    return 5*sqrt((goal.getXPosition()-current.getXPosition())*(goal.getXPosition()-current.getXPosition()) + (goal.getYPosition()-current.getYPosition())*(goal.getYPosition()-current.getYPosition()));
}

double AStar::calcWeight(VertexList closedListVertex, Edge newDirection, bool reverse){
    double weight = 0;
    
    switch (closedListVertex.orientationOfRobot) {
        case 'n':
            switch (newDirection.getDirection()) {
                case 'n':
                    weight = closedListVertex.costTravel + newDirection.getWeight();
                    break;
                    
                case 'e':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                case 's':
                    if (reverse) {
                        weight = closedListVertex.costTravel + newDirection.getWeight();
                    } else {
                        weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer*1.25;
                    }
                    break;
                    
                case 'w':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                default:
                    break;
            }
            break;
            
        case 's':
            switch (newDirection.getDirection()) {
                case 'n':
                    if (reverse) {
                        weight = closedListVertex.costTravel + newDirection.getWeight();
                    } else {
                        weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer*1.25;
                    }
                    break;
                    
                case 'e':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                case 's':
                    weight = closedListVertex.costTravel + newDirection.getWeight();
                    break;
                    
                case 'w':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                default:
                    break;
            }
            break;
            
        case 'e':
            switch (newDirection.getDirection()) {
                case 'n':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                case 'e':
                    weight = closedListVertex.costTravel + newDirection.getWeight();
                    break;
                    
                case 's':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                case 'w':
                    if (reverse) {
                        weight = closedListVertex.costTravel + newDirection.getWeight();
                    } else {
                        weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer*1.25;
                    }
                    break;
                    
                default:
                    break;
            }
            break;
            
        case 'w':
            switch (newDirection.getDirection()) {
                case 'n':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                case 'e':
                    if (reverse) {
                        weight = closedListVertex.costTravel + newDirection.getWeight();
                    } else {
                        weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer*1.25;
                    }
                    break;
                    
                case 's':
                    weight = closedListVertex.costTravel + newDirection.getWeight()*turningMultiplyer;
                    break;
                    
                case 'w':
                    weight = closedListVertex.costTravel + newDirection.getWeight();
                    break;
                    
                default:
                    break;
            }
            break;
            
        default:
            break;
    }
    
    return weight;
}

char AStar::calcOrientation(char currentOrientation, char edgeDirection){
    char orientation;
    if (currentOrientation == 'n' && edgeDirection == 's')
    {
        orientation = 'n';
    }
    else if (currentOrientation == 'e' && edgeDirection == 'w')
    {
        orientation = 'e';
    }
    else if (currentOrientation == 's' && edgeDirection == 'n')
    {
        orientation = 's';
    }
    else if (currentOrientation == 'w' && edgeDirection == 'e')
    {
        orientation = 'w';
    }
    else
    {
        orientation = edgeDirection;
    }
    return orientation;
}

AStar::~AStar(){
    
}
