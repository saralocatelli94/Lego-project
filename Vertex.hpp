//
//  Vertex.hpp
//  Graphs
//
//  Created by Olliver Ordell on 04/12/2016.
//  Copyright © 2016 Olliver Ordell. All rights reserved.
//

#ifndef Vertex_hpp
#define Vertex_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

class Edge;

class Vertex{
public:
    Vertex();
    ~Vertex();
    
    Vertex(int index, std::string name);
    Vertex(int index, std::string name, bool diamond, bool goal, bool sokoban);
    Vertex(int index, std::string name, bool diamond, bool goal, bool sokoban, char sDirection);
    
    std::vector<Edge> connections;
    
    int getIndex();
    std::string getName();
    bool getDiamond();
    bool getGoal();
    bool getSokoban();
    bool getgoalReached();
    char getSokobanDirection();
    int getXPosition();
    int getYPosition();
    
    void setGoal(bool g);
    void setDiamond(bool d);
    void setSokoban(bool s);
    void setgoalReached(bool gr);
    void setSokobanDirection(char sDirection);
    
protected:
    int index, xPosition, yPosition;
    std::string name;
    
    bool diamond, goal, sokoban, goalReached;
    char sokobanDirection;
};

////////////////////////////////////////////////////////////////

class Edge{
public:
    Edge();
    ~Edge();
    
    Edge(Vertex & target);
    Edge(Vertex & target, unsigned int weight);
    Edge(Vertex & target, unsigned int weight, char direction);
    
    Vertex *getTarget();
    int getWeight();
    char getDirection();
    
protected:
    Vertex *target;
    int weight;
    char direction;
};


#endif /* Vertex_hpp */
