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
    //orientationOfRobot = vStart.getSokobanDirection();
}

void AStar::runAStar(){
    if (!validStartAndGoal()) {
        return;
    }
    
    currentVertexIndex = vStart.getIndex();
    
    // Add startVertex to visited list:
    vertexClosedList.push_back(VertexList(&map.getVertex(currentVertexIndex), NULL,
                                          0, pythagoras(vStart, vGoal),
                                          map.getVertex(currentVertexIndex).getSokobanDirection()));
    
    int openListIndex = 0;
    int closedListIndex = 0;
    
    while (currentVertexIndex != vGoal.getIndex())
    {
        // Go through all connections from current vertex:
        for (int i = 0 ; i < map.getVertex(currentVertexIndex).connections.size() ; i++){
            
            /*
             TODO:
             Impliment check for each vertex:
             - Check if the connected vertex contains a dimond. If so, you can't move there.
             - Compair the direction of the robot, and the edge. Update the direction of the robot corrosponsingly
             */
            
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
            if (!prevAddedToOpenList && !prevAddedToClosedList)
            {
                // See if ro robot needs to turn 90 degree
                double weight = 0;
                if (vertexClosedList[closedListIndex].orientationOfRobot == map.getVertex(currentVertexIndex).connections[i].getDirection())
                {
                    weight = vertexClosedList[closedListIndex].costTravel + map.getVertex(currentVertexIndex).connections[i].getWeight();
                } else {
                    weight = vertexClosedList[closedListIndex].costTravel + map.getVertex(currentVertexIndex).connections[i].getWeight() * turningMultiplyer;
                }
                
                // See of the robot should revers:
                char orientation;
                if (vertexClosedList[closedListIndex].orientationOfRobot == 'n' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 's')
                {
                    orientation = 'n';
                }
                else if (vertexClosedList[closedListIndex].orientationOfRobot == 'e' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 'w')
                {
                    orientation = 'e';
                }
                else if (vertexClosedList[closedListIndex].orientationOfRobot == 's' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 'n')
                {
                    orientation = 's';
                }
                else if (vertexClosedList[closedListIndex].orientationOfRobot == 'w' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 'e')
                {
                    orientation = 'w';
                }
                else
                {
                    orientation = map.getVertex(currentVertexIndex).connections[i].getDirection();
                }
                
                vertexOpenList.push_back(VertexList(map.getVertex(currentVertexIndex).connections[i].getTarget(),
                                                    &map.getVertex(currentVertexIndex),
                                                    weight,
                                                    2*pythagoras(vGoal, *map.getVertex(currentVertexIndex).connections[i].getTarget()),
                                                    orientation));
            }
            // Check if new route is cheaper than prev found route:
            else if (prevAddedToOpenList && !prevAddedToClosedList)
            {
                if (vertexOpenList[openListIndex].costTravel >
                    (map.getVertex(currentVertexIndex).connections[i].getWeight() + vertexClosedList[closedListIndex].costTravel))
                {
                    // See if ro robot needs to turn 90 degree
                    double weight = 0;
                    if (vertexClosedList[closedListIndex].orientationOfRobot == map.getVertex(currentVertexIndex).connections[i].getDirection())
                    {
                        weight = vertexClosedList[closedListIndex].costTravel + map.getVertex(currentVertexIndex).connections[i].getWeight();
                    } else {
                        weight = vertexClosedList[closedListIndex].costTravel + map.getVertex(currentVertexIndex).connections[i].getWeight() * turningMultiplyer;
                    }
                    
                    // See of the robot should revers:
                    char orientation;
                    if (vertexClosedList[closedListIndex].orientationOfRobot == 'n' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 's')
                    {
                        orientation = 'n';
                    }
                    else if (vertexClosedList[closedListIndex].orientationOfRobot == 'e' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 'w')
                    {
                        orientation = 'e';
                    }
                    else if (vertexClosedList[closedListIndex].orientationOfRobot == 's' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 'n')
                    {
                        orientation = 's';
                    }
                    else if (vertexClosedList[closedListIndex].orientationOfRobot == 'w' && map.getVertex(currentVertexIndex).connections[i].getDirection() == 'e')
                    {
                        orientation = 'w';
                    }
                    else
                    {
                        orientation = map.getVertex(currentVertexIndex).connections[i].getDirection();
                    }
                    
                    vertexOpenList[openListIndex].vertexPrevious = map.getVertex(currentVertexIndex).connections[i].getTarget();
                    vertexOpenList[openListIndex].costTravel = weight;
                    vertexOpenList[openListIndex].orientationOfRobot = orientation;
                }
            }
        
        }
        
        // Safty check:
        if (vertexOpenList.empty())
        {
            std::cerr << "ERROR: No valid path to goal.\n";
            return;
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
    
    printVertexToGoal();
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
        std::cerr << "ERROR: Sokoban not on start-Vertex.\n";
        return false;
    }
    else if (vStart.getIndex() == vGoal.getIndex()) {
        std::cerr << "ERROR: Start- and Goal-Vertex are the same\n";
        return false;
    }
    else {
        return true;
    }
}

double AStar::pythagoras(Vertex current, Vertex goal){
    return sqrt((goal.getXPosition()-current.getXPosition())*(goal.getXPosition()-current.getXPosition()) + (goal.getYPosition()-current.getYPosition())*(goal.getYPosition()-current.getYPosition()));
}

AStar::~AStar(){
    
}
