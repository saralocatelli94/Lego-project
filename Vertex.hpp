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
    Vertex(int index, std::string name, bool dimond, bool goal, bool sokoban);
    
    std::vector<Edge> connections;
    
    int getIndex();
    std::string getName();
    bool getDimond();
    bool getGoal();
    bool getSokoban();
    
    void setDimond(bool d);
    void setSokoban(bool s);
    
protected:
    int index;
    std::string name;
    
    bool dimond, goal, sokoban;
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
