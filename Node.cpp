/*
 * Node.cpp
 *
 *  Created on: 23 set 2017
 *      Author: sara
 */

#include "Node.h"
#include <iostream>

Node::Node() {
    prev = NULL;
}

Node::Node(int v) {

    value = v;
}

Node::~Node() {

}

bool Node::hasPrev(){
    if (prev != NULL)
        return true;
    else
        return false;
}

Node* Node::getPrev(){
    return prev;
}

void Node::setPrev(Node* newPrev){
    if(newPrev == NULL)
    	prev = NULL;
    else
    	prev = newPrev;
}

int Node::getValue(){
    return value;
}
