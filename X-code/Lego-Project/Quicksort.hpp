//
//  Quicksort.hpp
//  Lego-Project
//
//  Created by Olliver Ordell on 04/12/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#ifndef Quicksort_hpp
#define Quicksort_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include "SolverNode_v2.hpp"

class Quicksort{
public:
    Quicksort();
    ~Quicksort();
    
    void quickSort(std::vector<SolverNode_v2> & l);
    
private:
    int median3(std::vector<SolverNode_v2> & l, int left, int right);
    void quickSort(std::vector<SolverNode_v2> & l, int left, int right);
    void insertionSort(std::vector<SolverNode_v2> & l, int left, int right);
};

#endif /* Quicksort_hpp */
