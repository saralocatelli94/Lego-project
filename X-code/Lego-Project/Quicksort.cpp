//
//  Quicksort.cpp
//  Lego-Project
//
//  Created by Olliver Ordell on 04/12/2017.
//  Copyright © 2017 Olliver Ordell. All rights reserved.
//

#include "Quicksort.hpp"

Quicksort::Quicksort(){
    
}

Quicksort::~Quicksort(){
    
}

void Quicksort::quickSort(std::vector<SolverNode_v2> & list){
    quickSort(list, 0, list.size()-1);
}

int Quicksort::median3(std::vector<SolverNode_v2> &l, int left, int right){
    int center = (left+right)/2;
    
    if (l[center].distanceTotal < l[left].distanceTotal)
        std::swap(l[left], l[center]);
    if (l[right].distanceTotal < l[left].distanceTotal)
        std::swap(l[left], l[right]);
    if (l[right].distanceTotal < l[center].distanceTotal)
        std::swap(l[center], l[right]);
    
    std::swap(l[center], l[right - 1]);
    return l[right - 1].distanceTotal;
}

void Quicksort::quickSort(std::vector<SolverNode_v2> &list, int left, int right){
    
    if (left+10 <= right)
    {
        int pivot = median3(list, left, right);
        
        int i = left + 1, j = right - 2;
        for ( ; ; )                             //infinite loop
        {
            while (list[i].distanceTotal < pivot) i++;        //
            while (list[j].distanceTotal > pivot) j--;        //
            if (i < j)
            {
                std::swap(list[i], list[j]);
            }
            else
            {
                break;
            }
        }
        // restore pivot:
        std::swap(list[i], list[right-1]);
        
        quickSort(list, left, i-1);
        quickSort(list, i+1, right);
    }
    else
    {
        insertionSort(list, left, right);
    }
    
}

void Quicksort::insertionSort(std::vector<SolverNode_v2> & list, int left, int right){
    for (int i = left+1 ; i < right+1 ; ++i)
    {
        SolverNode_v2 temp = std::move(list[i]);
        int j;
        for (j = i ; j > 0 && temp.distanceTotal < list[j-1].distanceTotal; --j)
        {
            list[j] = std::move(list[j-1]);
        }
        list[j] = std::move(temp);
    }
}
