//
//  Graph.cpp
//  Graphs
//
//  Created by Olliver Ordell on 29/11/2016.
//  Copyright © 2016 Olliver Ordell. All rights reserved.
//
#include <algorithm>
#include <limits.h>
#include <stddef.h>
#include "Graph.hpp"
#include "Node.h"

Graph::Graph() {
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

void Graph::addVertex(std::string& name, bool& d, bool& g, bool& s, char sDirection){
    adjList.push_back(Vertex(numOfVertex, name, d, g, s, sDirection));
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

unsigned int Graph::getNumOfVertex(){
    return numOfVertex;
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
    	vertexMap[adjList[i].getIndex()]=getVertex(adjList[i].getName());
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

int** Graph::printAdjMatrix(){
	int size=numOfVertex;
	int** matrix=0;
	matrix = new int*[size];


	//fill the matrix
	for(int i=0;i<size;i++){
		matrix[i] = new int[size];
		for(int j=0;j<size;j++)
			matrix[i][j]=0;
	}
	for(int i=0;i<numOfVertex;i++){
			for(int k=0;k<getVertex(i).connections.size();k++){
				int destination=adjList[i].connections[k].getTarget()->getIndex();
				matrix[i][destination]=adjList[i].connections[k].getWeight();
				//matrix[i][destination]=adjList[i].connections[k];
			}
	}

	//print the matrix
	std::cout<<"Adjacent matrix: "<<std::endl;
	for(int i=0;i<size;i++){
		for(int j=0;j<size;j++){
			std::cout<<matrix[i][j]<<", ";
		}
		std::cout<<std::endl;
	}
	return matrix;

}

int Graph::minDistance(int dist[], bool sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;

   for (int v = 0; v < numOfVertex; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;

   return min_index;
}


void Graph::printSolution(int dist[])
{
   std::cout<<" Vertex distance from source"<<std::endl;
   std::cout<<"Vertex         Distance"<<std::endl;
   for (int i = 0; i < numOfVertex; i++)
	   std::cout<<i<<"             "<<dist[i]<<std::endl;

}

// return the index of the node with the min distance
int Graph::findMinDist(int dist[]){
	int min=INT_MAX;
	for(int i=0;i<numOfVertex;i++){
		if(dist[i]<dist[min] && dist[i]>0){

			min=i;
		}
	}
	return min;
}
std::vector<int > Graph::getDiamondsID(){
	std::vector<int> result;
	for(int i=0;i<numOfVertex;i++){
		if(vertexMap[i].getDimond())
			result.push_back(i);
	}
	return result;
}


//return the ID of the vertex with the min distance and the path from src to that vertex
//flag==0 destination=goal
//flag==1 destination=diamond
std::pair<int,std::vector<char>> Graph::dijkstra(int**matrix,int src,bool flag)
{
	std::vector<char> pathDirection;  // vector of the direction of the path
	if(src<0 || src>=numOfVertex )
	{
		std::cout<<"Insert a correct value. ";
		return std::make_pair(-1,pathDirection);
	}

		 int dist[numOfVertex];     //  dist[i] will hold the shortest distance from src to i
		 bool sptSet[numOfVertex]; // sptSet[i] is true if vertex i has already been processed
    //Node* nodes;
    //nodes = (Node*) calloc(numOfVertex, sizeof(Node));
    Node *nodes = new Node[numOfVertex];
		 // init
		 for (int i = 0; i < numOfVertex; i++)
			dist[i] = INT_MAX, sptSet[i] = false, nodes[i]=Node(i);

		 // Distance of source vertex from itself is always 0
		 dist[src] = 0;
		 nodes[src].setPrev(NULL);  //the src doesn't have a prev node

		 // start of algorithm
		 for (int count = 0; count < numOfVertex-1; count++)
		 {
		   int u = minDistance(dist, sptSet);
		   // Mark the picked vertex as processed
		   sptSet[u] = true;


		   // Update dist value of the adjacent vertices
		   for (int v = 0; v < numOfVertex; v++){

			 if (!sptSet[v] && matrix[u][v] && dist[u] != INT_MAX && dist[u]+matrix[u][v] < dist[v]
				){
				 // std::cout<<u<<", "<<!vertexMap[u].getDimond() <<std::endl;
				 if(!vertexMap[v].getgoalReached()){
					 dist[v] = dist[u] + matrix[u][v];
					nodes[v].setPrev(&nodes[u]); // u is the temporary prev node of v
				 }
			 }
		   }
		 }


		 if(flag==0)
		 //keep only the vertexs which have a goal
		 for(int i=0;i<numOfVertex;i++){
			 if(!vertexMap[i].getGoal()){
				 dist[i]=-1;
			 }
		 }

		 if(flag==1){
			 for(int i=0;i<numOfVertex;i++){
			 		 if(!vertexMap[i].getDimond()){
			 			 dist[i]=-1;
			 		 }
			 	 }
		 }

		 //printSolution(dist); // print the costs from the source to all the others nodes.
		 	 	 	 	 	 	//if a node isn't a goal the distance is -1


		 int dest=findMinDist(dist);
		 std::vector<int> path; //vector of the index of the vertex of the path

		 Node last=nodes[dest];
		 while(last.hasPrev()){
			 path.insert(path.begin(),last.getPrev()->getValue());
			// std::cout<< last.getPrev()->getValue()<<", ";
			 last=*last.getPrev();
		 }
		 path.insert(path.end(),dest);

		 std::cout<<" Shortest path from "<<src<<" ("<<vertexMap[src].getName()<<") to "<<dest<<"("<<vertexMap[dest].getName()<<")"<<std::endl;

		  //print the path ==> index+ vertex name
		 for(int i=0;i<path.size();i++){
			 vertexMap[i].getName();
			 std::cout<< path[i]<<" ("<<vertexMap[path[i]].getName()<<") ,";
		 }

		 //print the directions
		std::cout<<std::endl<<"Directions: "<<std::endl;
		 for(int i=0;i<path.size();i++){
			 for(int j=0; j < adjList[path[i]].connections.size();j++)
			 {
				 ///search the connection in the list with the same target and print the weight
				 if(adjList[path[i]].connections[j].getTarget()->getIndex()==path[i+1]){
					 pathDirection.insert(pathDirection.end(),adjList[path[i]].connections[j].getDirection());
					 std::cout<<"from "<<path[i]<<" to "<<path[i+1]<<": "<<pathDirection[i]<<std::endl;
				 }

			 }
		 }
		 return std::make_pair(dest,pathDirection);


}

void Graph::updateMap(int dest,int src){

	vertexMap[src].setDimond(false);
	vertexMap[dest].setGoal(false);
	vertexMap[dest].setgoalReached(true);

}

int Graph::getSource(){
	for(int i=0;i<numOfVertex;i++)
	{
		if(vertexMap[i].getSokoban())
			return i;
	}
	return -1;
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


