//
//  HashTableStruct.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 30/11/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef HashTableStruct_hpp
#define HashTableStruct_hpp

#include <stdio.h>
#include <vector>

struct HashTableStruct {
    HashTableStruct(std::vector<int> dPos, int rPos, int dist);
    
    std::vector<int> diamondPosition;
    int robotPosition;
    int distance;
    
    bool operator==(HashTableStruct rhs);
    bool operator!=(HashTableStruct rhs);
    
    bool dPosEqual(HashTableStruct h);
    bool dPosEqual(std::vector<int> d);
    
    bool rPosEqual(HashTableStruct h);
    bool rPosEqual(int r);
};

#endif /* HashTableStruct_hpp */
