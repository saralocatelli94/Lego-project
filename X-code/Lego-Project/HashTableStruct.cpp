//
//  HashTableStruct.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 30/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "HashTableStruct.hpp"

HashTableStruct::HashTableStruct(std::vector<int> dPos, int rPos, int dist):
diamondPosition(dPos), robotPosition(rPos), distance(dist){
}

bool HashTableStruct::operator==(HashTableStruct rhs){
    bool equal = true;
    for (int i = 0 ; i < this->diamondPosition.size() ; i++) {
        if (this->diamondPosition.at(i) != rhs.diamondPosition.at(i)) {
            equal = false;
        }
    }
    if (this->robotPosition != rhs.robotPosition) {
        equal = false;
    }
    return equal;
}

bool HashTableStruct::operator!=(HashTableStruct rhs){
    return (*this == rhs) ? false : true;
}

bool HashTableStruct::dPosEqual(HashTableStruct h){
    return dPosEqual(h.diamondPosition);
}

bool HashTableStruct::dPosEqual(std::vector<int> d){
    if (d.size() != this->diamondPosition.size()) {
        return false;
    }
    for (int i = 0 ; i < d.size() ; i++) {
        if (d.at(i) != this->diamondPosition.at(i)) {
            return false;
        }
    }
    return true;
}

bool HashTableStruct::rPosEqual(HashTableStruct h){
    return rPosEqual(h.robotPosition);
}

bool HashTableStruct::rPosEqual(int r){
    return (r == this->robotPosition) ? true : false;
}
