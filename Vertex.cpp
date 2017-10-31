//
//  Vertex.cpp
//  Graphs
//
//  Created by Olliver Ordell on 04/12/2016.
//  Copyright © 2016 Olliver Ordell. All rights reserved.
//

#include "Vertex.hpp"

Vertex::Vertex(){
    
}

Vertex::~Vertex(){
    
}

Vertex::Vertex(int i, std::string n):
index(i), name(n){
    xPosition = stoi(n) % 10;
    yPosition = stoi(n) / 10;
}

Vertex::Vertex(int i, std::string n, bool d, bool g, bool s):
index(i), name(n), diamond(d), goal(g), sokoban(s), goalReached(false){
    sokobanDirection = '0';
    xPosition = stoi(n) % 10;
    yPosition = stoi(n) / 10;
}

Vertex::Vertex(int i, std::string n, bool d, bool g, bool s, char sD):
index(i), name(n), diamond(d), goal(g), sokoban(s){
    xPosition = stoi(n) % 10;
    yPosition = stoi(n) / 10;
    if (s)
        sokobanDirection = sD;
    else
        sokobanDirection = '0';
}

int Vertex::getIndex(){
    return index;
}

std::string Vertex::getName(){
    return name;
}

bool Vertex::getDiamond(){
    return diamond;
}

bool Vertex::getGoal(){
    return goal;
}

bool Vertex::getSokoban(){
    return sokoban;
}

bool Vertex::getgoalReached(){
	return goalReached;
}

void Vertex::setGoal(bool g){
    goal=g;
}

char Vertex::getSokobanDirection(){
    return sokobanDirection;
}

int Vertex::getXPosition(){
    return xPosition;
}

int Vertex::getYPosition(){
    return yPosition;
}

void Vertex::setDiamond(bool d){
    diamond = d;
}

void Vertex::setSokoban(bool s){
    sokoban = s;
}

void Vertex::setgoalReached( bool gr){
	goalReached=gr;
}

void Vertex::setSokobanDirection(char sDir){
    sokobanDirection = sDir;
}

/////////////////////////////////////////////////////////////////////////////////////////////

Edge::Edge(){
    
}

Edge::~Edge(){
    
}

Edge::Edge(Vertex & tar):
target(&tar){
    weight = 1;
}

Edge::Edge(Vertex & tar, unsigned int w):
target(&tar), weight(w){
    
}

Edge::Edge(Vertex & tar, unsigned int w, char d):
target(&tar), weight(w){
    if (!(d == 'n' || d == 's' || d == 'e' || d == 'w')) {
        std::cerr << "ERROR: Direction not correct formate.\n";
        d = '0';
    } else {
        direction = d;
    }
}

Vertex *Edge::getTarget(){
    return target;
}

int Edge::getWeight(){
    return weight;
}

char Edge::getDirection(){
    return direction;
}
