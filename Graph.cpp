//
//  Graph.cpp
//  Graphs
//
//  Created by Olliver Ordell on 29/11/2016.
//  Copyright © 2016 Olliver Ordell. All rights reserved.
//

#include "Graph.hpp"

Graph::Graph(){
    numOfVertex = 0;
}

Graph::~Graph(){
    
}

void Graph::addVertex(){
    adjList.push_back(Vertex(numOfVertex, std::to_string(numOfVertex)));
    numOfVertex++;
}

void Graph::addVertex(std::string& name){
    adjList.push_back(Vertex(numOfVertex, name));
    numOfVertex++;
}

void Graph::addVertex(std::string& name, bool& d, bool& g, bool& s){
    adjList.push_back(Vertex(numOfVertex, name, d, g, s));
    numOfVertex++;
}

void Graph::addEdge(unsigned int source, unsigned int target){
    if (source > numOfVertex || target > numOfVertex)
    {
        std::cerr << "Error: Source and/or target out of range.\n";
    }
    else if (source == target)
    {
        std::cerr << "Error: Source can't connect to source.\n";
    }
    else
    {
        for (int i = 0 ; i < adjList[source].connections.size() ; i++)
        {
            if (adjList[source].connections[i].getTarget()->getIndex() == target)
            {
                std::cerr << "Error: Edge already exist.\n";
                return;
            }
        }
        adjList[source].connections.push_back(Edge(adjList[target]));
    }
}

void Graph::addEdge(unsigned int source, unsigned int target, unsigned int weight){
    if (source > numOfVertex || target > numOfVertex)
    {
        std::cerr << "Error: Source and/or target out of range.\n";
        return;
    }
    else if (source == target)
    {
        std::cerr << "Error: Source can't connect to source.\n";
    }
    else
    {
        for (int i = 0 ; i < adjList[source].connections.size() ; i++)
        {
            if (adjList[source].connections[i].getTarget()->getIndex() == target)
            {
                std::cerr << "Error: Edge already exist.\n";
                return;
            }
        }
        adjList[source].connections.push_back(Edge(adjList[target], weight));
    }
}

void Graph::addEdge(std::string source, std::string target, unsigned int weight){
    if (getVertex(source).getIndex() > numOfVertex || getVertex(target).getIndex() > numOfVertex)
    {
        std::cerr << "Error: Source and/or target out of range.\n";
        return;
    }
    else if (source == target)
    {
        std::cerr << "Error: Source can't connect to source.\n";
    }
    else
    {
        for (int i = 0 ; i < adjList[getVertex(source).getIndex()].connections.size() ; i++)
        {
            if (getVertex(source).connections[i].getTarget()->getIndex() == getVertex(target).getIndex())
            {
                std::cerr << "Error: Edge already exist.\n";
                return;
            }
        }
        getVertex(source).connections.push_back(Edge(getVertex(target), weight));
    }
}

void Graph::addEdge(std::string source, std::string target, unsigned int weight, char direction){
    if (getVertex(source).getIndex() > numOfVertex || getVertex(target).getIndex() > numOfVertex)
    {
        std::cerr << "Error: Source and/or target out of range.\n";
        return;
    }
    else if (source == target)
    {
        std::cerr << "Error: Source can't connect to source.\n";
    }
    else
    {
        for (int i = 0 ; i < adjList[getVertex(source).getIndex()].connections.size() ; i++)
        {
            if (getVertex(source).connections[i].getTarget()->getIndex() == getVertex(target).getIndex())
            {
                std::cerr << "Error: Edge already exist.\n";
                return;
            }
        }
        getVertex(source).connections.push_back(Edge(getVertex(target), weight, direction));
    }
}

void Graph::removeEdge(unsigned int source, unsigned int target){
    if (source > numOfVertex || target > numOfVertex)
    {
        std::cerr << "Error: Source and/or target out of range.\n";
    }
    else if (source == target)
    {
        std::cerr << "Error: Source is never connect to source.\n";
    }
    else
    {
        std::cout << "Removing edge...\n";
        for (int i = 0 ; i < adjList[source].connections.size() ; i++)
        {
            if (adjList[source].connections[i].getTarget()->getIndex() == target)
            {
                adjList[source].connections.erase(adjList[source].connections.begin()+i);
                std::cout << "Edge removed...\n";
                return;
            }
        }
        std::cerr << "Error: Edge not found.\n";
    }
}

bool Graph::vertexExist(std::string n){
    for (int i = 0 ; i < numOfVertex ; i++){
        if (getVertex(i).getName() == n) {
            return true;
        }
    }
    return false;
}

Vertex &Graph::getVertex(unsigned int i){
    return adjList[i];
}

Vertex &Graph::getVertex(std::string n){
    for (int i = 0 ; i < numOfVertex ; i++) {
        if (getVertex(i).getName() == n) {
            return adjList[i];
        }
    }
    std::cout << "ERROR: No vertex with the name " << n << "." << std::endl;
    return adjList[0];
}

void Graph::printGraph(){
    std::cout << "Printing the graph:\n\n";
    std::cout << std::setw(6) << "Index:";
    std::cout << std::setw(7) << "Name:";
    std::cout << std::setw(10) << "Dimond:";
    std::cout << std::setw(7) << "Goal:";
    std::cout << std::setw(10) << "Sokoban:";
    std::cout << std::setw(15) << "Connection:";
    std::cout << std::setw(11) << "Weight:";
    std::cout << std::setw(15) << "Direction:\n";
    for (int i = 0 ; i < numOfVertex ; i++)
    {
        std::cout << std::setw(6) << adjList[i].getIndex();
        std::cout << std::setw(7) << adjList[i].getName();
        std::cout << std::setw(10) << adjList[i].getDimond();
        std::cout << std::setw(7) << adjList[i].getGoal();
        std::cout << std::setw(10) << adjList[i].getSokoban();
        for (int j = 0 ; j < getVertex(i).connections.size() ; j++)
        {
            if (j > 0)
            {
                std::cout << std::setw(6) << "  ";
                std::cout << std::setw(7) << "  ";
                std::cout << std::setw(10) << "  ";
                std::cout << std::setw(7) << "  ";
                std::cout << std::setw(10) << "  ";
            }
            std::cout << std::setw(15) << adjList[i].connections[j].getTarget()->getName();
            std::cout << std::setw(11) << adjList[i].connections[j].getWeight();
            std::cout << std::setw(14);
            char temp = adjList[i].connections[j].getDirection();
            if (temp == 'n')
                std::cout << "North" << std::endl;
            else if (temp == 'e')
                std::cout << "East" << std::endl;
            else if (temp == 's')
                std::cout << "South" << std::endl;
            else if (temp == 'w')
                std::cout << "West" << std::endl;
        }
        if (getVertex(i).connections.size() == 0)
        {
            std::cout << std::endl;
        }
    }
}

void Graph::topSort(Vertex &tempV, std::vector<Vertex *> & stack, std::vector<Vertex *> & visited){
    
    visited.push_back(&tempV);
    
    bool checked;
    
    for (int i = 0 ; i < tempV.connections.size() ; i++)
    {
        checked = false;
        for (int j = 0 ; j < visited.size() && !checked ; j++)
        {
            if (visited[j] == tempV.connections[i].getTarget())
            {
                checked = true;
            }
        }
        if (!checked)
        {
            topSort(*tempV.connections[i].getTarget(), stack, visited);
        }
    }
    stack.push_back(&tempV);
}


